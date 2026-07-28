using UnityEngine;

/// <summary>
/// HoverController_Foundation v1.0
///
/// Holds the chassis at hover height via per-point spring-dampers, levels it to
/// the ground normal, damps tilt, and runs two recovery paths:
///
///   Upright unstick: chassis touching ground but not flipped. A short tapering
///                    upward push frees it. Speed is intentionally NOT gated;
///                    unstick must work after any landing velocity.
///
///   Flip recovery:   chassis touching ground, flipped, and slow. Righting
///                    torque rotates it back upright. Longer delay so a flip
///                    reads as a real consequence.
///
/// Both paths can be suspended via SetRecoveryEnabled(bool) for EMP, scripted
/// events, or ability hooks.
///
/// Tuning lives on a VehicleTuningProfile asset (profile.foundation). Scene refs
/// (hoverPoints, groundLayers) and runtime state stay on the component.
///
/// Physics contract: never writes to rb.angularVelocity, rb.rotation, or
/// rb.linearVelocity. All motion is AddForce / AddTorque.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController_Foundation : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 📦 Tuning Profile
    // -------------------------------------------------------------------------
    [Header("📦 Tuning")]
    [Tooltip("Vehicle tuning profile (shared SO). All numeric tuning lives here. Required.")]
    [SerializeField] private VehicleTuningProfile profile;

    /// <summary>Shorthand for profile.foundation. Used at every read site below.</summary>
    private FoundationTuning F => profile.foundation;

    // -------------------------------------------------------------------------
    // 🧩 Hover Points
    // -------------------------------------------------------------------------
    [Header("🧩 Hover Points")]
    [Tooltip("Empty transforms positioned at the bottom of the chassis (one per corner is typical). " +
             "Each casts a ray straight down to find the ground. Assign by hand from the hover point children.")]
    [SerializeField] private Transform[] hoverPoints;

    // -------------------------------------------------------------------------
    // 🌍 Ground Interaction
    // -------------------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Layers treated as ground for hover detection.")]
    [SerializeField] private LayerMask groundLayers = ~0;

    [Tooltip("Draws hover rays in the scene view.")]
    [SerializeField] private bool drawDebugRays = true;

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug Rays. " +
             "Create via Assets > Create > Hover > Debug Settings.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.enableDebugGizmos : drawDebugRays;

    // -------------------------------------------------------------------------
    // 📡 Collision Tracking
    // -------------------------------------------------------------------------
    /// <summary>
    /// Tracks ground-layer collision contact. OnCollisionStay fires every physics
    /// step while contact persists, refreshing a timestamp. Contact state is a
    /// timestamp, not a boolean, per the timestamps-over-booleans principle:
    /// an OnCollisionExit-cleared bool goes false when exiting ONE of two
    /// simultaneously touching ground colliders, and needs the exit callback to
    /// fire at all (not guaranteed on collider destruction). The timestamp simply
    /// goes stale. groundContactNormal is only read behind IsContactingGround, so
    /// staleness there is harmless.
    /// Both flip recovery and unstick read these values; neither writes them.
    /// </summary>
    private void OnCollisionStay(Collision collision)
    {
        if ((groundLayers.value & (1 << collision.gameObject.layer)) == 0)
            return;

        lastGroundContactTime = Time.time;

        int count = collision.GetContacts(_contactBuffer);
        if (count > 0)
        {
            Vector3 normalSum = Vector3.zero;
            for (int i = 0; i < count; i++)
                normalSum += _contactBuffer[i].normal;
            groundContactNormal = (normalSum / count).normalized;
        }
    }

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private Rigidbody rb;
    private HoverController_Energy energy;

    // Pre-allocated buffer for OnCollisionStay contact reads. Avoids per-call
    // ContactPoint[] allocation on every physics tick.
    private readonly System.Collections.Generic.List<ContactPoint> _contactBuffer = new(8);

    private float   unstickTimer;            // counts up while upright and contacting ground
    private float   flipTimer;               // counts up while flipped, slow, and contacting ground
    private float   lastGroundContactTime = float.NegativeInfinity; // refreshed by OnCollisionStay
    private Vector3 groundContactNormal;

    /// <summary>
    /// True while ground contact is fresh. OnCollisionStay refreshes the
    /// timestamp every physics step during contact; two fixed steps of grace
    /// covers callback ordering within a step.
    /// </summary>
    private bool IsContactingGround => Time.time - lastGroundContactTime <= Time.fixedDeltaTime * 2f;

    // Aim pitch target, written by Propulsion via SetAimPitch each FixedUpdate
    // while strafe mode is active. Degrees in Unity euler-X convention
    // (negative = nose up). Weight is the strafe blend (0 = no target).
    private float aimPitchDegrees;
    private float aimPitchWeight;
    private float   unstickFiredFlashTimer;  // drives the fired-impulse gizmo
    private bool    rightingAuthorized;      // true after flip timer threshold; cleared when craft rights
    private bool    recoveryEnabled = true;
    private float   unstickForceTimer;       // counts down while sustained unstick lift is being applied
    private Vector3 unstickForceDir;         // direction cached at trigger time, held for duration

    /// <summary>True when at least one hover point has a ground hit this frame.</summary>
    public bool IsHoverGrounded { get; private set; }

    /// <summary>Average ground normal this frame (Vector3.up when airborne).</summary>
    public Vector3 AverageGroundNormal { get; private set; }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// <summary>
    /// Sets the aim pitch target for the leveling torque. Called by Propulsion
    /// every FixedUpdate while strafe mode is active (weight = strafe blend),
    /// and with (0, 0) on strafe exit or EMP freeze.
    ///
    /// Degrees use Unity's euler-X convention: negative = nose up.
    /// While weight > 0, leveling drives toward the ground normal rotated by
    /// this pitch, at aimPitchTrackingStrength on the pitch axis only. This
    /// makes leveling the single torque authority over attitude; Propulsion
    /// applies no pitch torque of its own (competing forces cause jitter).
    /// </summary>
    public void SetAimPitch(float degrees, float weight)
    {
        aimPitchDegrees = degrees;
        aimPitchWeight  = Mathf.Clamp01(weight);
    }

    /// <summary>
    /// Enables or disables both flip recovery and ground unstick.
    /// Use for EMP effects, scripted events, or ability interactions.
    /// When disabled, timers are reset so recovery doesn't fire immediately on re-enable.
    /// </summary>
    public void SetRecoveryEnabled(bool enabled)
    {
        recoveryEnabled = enabled;
        if (!enabled)
        {
            unstickTimer       = 0f;
            flipTimer          = 0f;
            rightingAuthorized = false;
        }
    }

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        rb     = GetComponent<Rigidbody>();
        energy = GetComponent<HoverController_Energy>();

        if (profile == null)
        {
            Debug.LogError(
                $"[Foundation] '{name}': VehicleTuningProfile is not assigned. " +
                $"Assign one in the inspector. Hover disabled.",
                this
            );
            enabled = false;
            return;
        }

        ValidateHoverPoints();
    }

    private void ValidateHoverPoints()
    {
        if (hoverPoints == null || hoverPoints.Length == 0)
        {
            Debug.LogError(
                $"[Foundation] '{name}': hoverPoints array is empty. " +
                $"Assign HoverPoint transforms in the Inspector. Hover disabled.",
                this
            );
            enabled = false;
            return;
        }

        for (int i = 0; i < hoverPoints.Length; i++)
        {
            if (hoverPoints[i] == null)
            {
                Debug.LogError(
                    $"[Foundation] '{name}': hoverPoints[{i}] is null. " +
                    $"Fix the missing reference in the Inspector. Hover disabled.",
                    this
                );
                enabled = false;
                return;
            }
        }
    }

    private void FixedUpdate()
    {
        // EMP freeze: all Foundation forces off. Default Unity gravity still pulls
        // the chassis down; existing angular velocity persists with no leveling or
        // damping, so the vehicle tumbles for the freeze duration.
        if (energy != null && energy.IsEmpFrozen)
        {
            IsHoverGrounded     = false;
            AverageGroundNormal = Vector3.up;
            return;
        }

        ApplyHoverForces();
        ApplyExtraGravity();
        ApplyLevelingTorque();
        ApplyPitchRollDamping();
        HandleRecovery();
        ApplyUnstickForce();
    }

    private void Update()
    {
        // Tick the debug flash timer in Update so it runs at game framerate.
        // OnDrawGizmos runs on the editor's schedule (unpredictable, sometimes
        // multiple times per frame, sometimes suppressed) and must not mutate state.
        if (unstickFiredFlashTimer > 0f)
            unstickFiredFlashTimer = Mathf.Max(0f, unstickFiredFlashTimer - Time.deltaTime);
    }

    // -------------------------------------------------------------------------
    // 🧠 Per-Point Spring-Damper Lift
    // -------------------------------------------------------------------------
    private void ApplyHoverForces()
    {
        IsHoverGrounded     = false;
        AverageGroundNormal = Vector3.up;

        Vector3 normalSum     = Vector3.zero;
        int     groundedCount = 0;

        foreach (Transform point in hoverPoints)
        {
            Vector3 rayDir = -point.up;

            // QueryTriggerInteraction.Ignore: trigger volumes (pickups, ability
            // fields) must never feed the hover springs.
            if (!Physics.Raycast(point.position, rayDir, out RaycastHit hit, F.sensorRange, groundLayers,
                                 QueryTriggerInteraction.Ignore))
            {
                if (ShouldDrawDebug)
                    Debug.DrawRay(point.position, rayDir * F.sensorRange, Color.red);
                continue;
            }

            float compression         = F.hoverHeight - hit.distance;
            float velocityAlongNormal = Vector3.Dot(rb.GetPointVelocity(point.position), hit.normal);
            float springForce         = compression * F.liftStrength - velocityAlongNormal * F.liftDamping;

            if (F.enableSlopeLiftCompensation)
            {
                float slopeFactor = Mathf.Clamp01(1f - hit.normal.y);
                springForce *= Mathf.Lerp(1f, F.slopeLiftMultiplier, slopeFactor);
            }

            // Clamp at zero: the spring only pushes, never pulls. Load-bearing for
            // jump feel — without it the damping term would fight jump takeoff.
            springForce = Mathf.Max(springForce, 0f);

            // ForceMode.Acceleration: mass independent, like every other force in
            // the controllers (jump, dodge, unstick, gravity, all torques).
            // liftStrength/liftDamping are accelerations, so one tuning profile
            // behaves identically across vehicle masses. Values retuned from
            // Force-mode by dividing by vehicle mass (1000): 10000 -> 10,
            // 1500 -> 1.5. Identical forces at mass 1000; feel unchanged.
            rb.AddForceAtPosition(hit.normal * springForce, point.position, ForceMode.Acceleration);

            normalSum    += hit.normal;
            groundedCount++;

            if (ShouldDrawDebug)
                Debug.DrawRay(point.position, rayDir * hit.distance, Color.green);
        }

        if (groundedCount > 0)
        {
            IsHoverGrounded     = true;
            AverageGroundNormal = (normalSum / groundedCount).normalized;
        }
    }

    // -------------------------------------------------------------------------
    // 🌎 Extra Gravity
    // -------------------------------------------------------------------------
    /// <summary>
    /// extraGravityMultiplier always applies. It increases the effective weight of the
    /// chassis and helps it sit firmly on the hover springs.
    /// extraAirGravity only applies airborne. It pulls the chassis down faster after
    /// jumps and ramps without affecting grounded feel.
    /// Both are tuning knobs for physical character, not player input.
    /// </summary>
    private void ApplyExtraGravity()
    {
        if (F.extraGravityMultiplier > 0f)
            rb.AddForce(Physics.gravity * F.extraGravityMultiplier, ForceMode.Acceleration);

        if (!IsHoverGrounded && F.extraAirGravity > 0f)
            rb.AddForce(Vector3.down * F.extraAirGravity, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // ⚖️ Torque-Based Attitude Alignment (ground normal + aim pitch)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Single torque authority over attitude. Two responsibilities, one torque:
    ///
    ///   Grounded leveling: align chassis up to AverageGroundNormal (original role,
    ///               unchanged behavior when no aim target is set).
    ///
    ///   Aim pitch:  while Propulsion sets a target via SetAimPitch (strafe mode),
    ///               the target up is the ground normal rotated by the aim pitch.
    ///               Axis split: the pitch-axis torque component runs at
    ///               aimPitchTrackingStrength, everything else (roll, bump
    ///               following) stays at base leveling strength, so FPS-snappy aim
    ///               tuning (150 vs 12) never stiffens the ride over bumps.
    ///               Replaces the old design where Propulsion torqued the nose
    ///               *against* this leveling (competing forces cause jitter).
    ///               pitchRollDamping is the oscillation killer.
    ///
    /// Airborne with an aim target, the pitch axis still tracks (aiming mid-jump
    /// matters in combat); AverageGroundNormal is Vector3.up there, and the align
    /// component gets zero strength, preserving free airborne attitude.
    /// </summary>
    private void ApplyLevelingTorque()
    {
        // Aim tracking is honored when hover springs are engaged or the craft is
        // truly in the air. While scraping ground without hover (belly/roof
        // contact), recovery paths own attitude and aim must not fight them.
        bool aiming = aimPitchWeight > 0.001f
                   && (IsHoverGrounded || !IsContactingGround);

        float baseStrength = IsHoverGrounded ? F.levelingTorqueStrength : 0f;

        if (!aiming && baseStrength <= 0f)
            return;

        Vector3 targetUp = aiming
            ? Quaternion.AngleAxis(aimPitchDegrees, transform.right) * AverageGroundNormal
            : AverageGroundNormal;

        Vector3 torqueAxis = Vector3.Cross(transform.up, targetUp);

        if (aiming)
        {
            Vector3 pitchPart = Vector3.Project(torqueAxis, transform.right);
            Vector3 alignPart = torqueAxis - pitchPart;

            float aimStrength = Mathf.Lerp(baseStrength, F.aimPitchTrackingStrength, aimPitchWeight);
            rb.AddTorque(pitchPart * aimStrength + alignPart * baseStrength, ForceMode.Acceleration);
        }
        else
        {
            rb.AddTorque(torqueAxis * baseStrength, ForceMode.Acceleration);
        }
    }

    // -------------------------------------------------------------------------
    // ⚙️ Torque-Based Pitch/Roll Damping
    // -------------------------------------------------------------------------
    private void ApplyPitchRollDamping()
    {
        if (F.pitchRollDamping <= 0f)
            return;

        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);

        Vector3 dampingLocal = new Vector3(
            -localAngVel.x * F.pitchRollDamping,
            0f,
            -localAngVel.z * F.pitchRollDamping
        );

        rb.AddRelativeTorque(dampingLocal, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // 🔄 Flip Recovery + 📌 Ground Unstick  (split paths)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Two fully independent recovery paths:
    ///
    ///   Upright unstick: contacting ground AND not flipped. Speed not gated.
    ///                    After unstickRecoveryDelay, begins a sustained upward push
    ///                    window via ApplyUnstickForce.
    ///
    ///   Flip recovery:   contacting ground AND flipped AND slow.
    ///                    After flipRecoveryDelay, sets rightingAuthorized which gates
    ///                    the righting torque each FixedUpdate until the craft rights
    ///                    or hover springs re-engage.
    ///
    /// 180° degeneracy: Cross(transform.up, Vector3.up) is zero when perfectly inverted.
    /// A forward pitch bias breaks the symmetry.
    /// </summary>
    private void HandleRecovery()
    {
        if (!recoveryEnabled)
            return;

        float tiltAngle = Vector3.Angle(transform.up, Vector3.up);
        bool  isFlipped = tiltAngle >= F.flipRecoveryAngleThreshold;
        bool  isSlow    = rb.linearVelocity.sqrMagnitude < F.flipRecoverySpeedThreshold * F.flipRecoverySpeedThreshold;

        // --- Righting torque (runs every frame while authorized) ---
        if (!isFlipped || IsHoverGrounded)
            rightingAuthorized = false;

        if (rightingAuthorized && F.flipRecoveryTorque > 0f)
        {
            Vector3 torqueAxis = Vector3.Cross(transform.up, Vector3.up);

            if (torqueAxis.sqrMagnitude < 0.001f)
                torqueAxis = transform.right * 0.1f;

            rb.AddTorque(torqueAxis.normalized * F.flipRecoveryTorque, ForceMode.Acceleration);
        }

        // --- Upright unstick path ---
        // Gate: contacting ground AND upright. Speed intentionally not gated.
        if (IsContactingGround && !isFlipped)
        {
            unstickTimer += Time.fixedDeltaTime;

            if (unstickTimer >= F.unstickRecoveryDelay)
            {
                unstickTimer = 0f;

                unstickForceDir = groundContactNormal.sqrMagnitude > 0.01f
                    ? groundContactNormal
                    : Vector3.up;

                unstickForceTimer      = F.unstickLiftDuration;
                unstickFiredFlashTimer = 0.5f;

                if (ShouldDrawDebug)
                    Debug.DrawRay(transform.position, unstickForceDir * 2f, Color.cyan);
            }
        }
        else
        {
            unstickTimer = 0f;
        }

        // --- Flip recovery path ---
        // Gate: contacting ground AND flipped AND slow.
        if (IsContactingGround && isFlipped && isSlow)
        {
            flipTimer += Time.fixedDeltaTime;

            if (flipTimer >= F.flipRecoveryDelay)
            {
                flipTimer          = 0f;
                rightingAuthorized = true;

                unstickFiredFlashTimer = 0.5f;

                if (ShouldDrawDebug)
                    Debug.DrawRay(transform.position, Vector3.up * 2f, Color.magenta);
            }
        }
        else
        {
            flipTimer = 0f;
        }
    }

    // -------------------------------------------------------------------------
    // 📌 Sustained Unstick Lift
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies the unstick lift over a short window each FixedUpdate.
    /// Force is front-loaded: strongest on the first frame, tapering to zero.
    /// Reads as a gentle invisible lift rather than a visible punch.
    /// Mass independent so behavior is consistent across vehicles.
    /// </summary>
    private void ApplyUnstickForce()
    {
        if (unstickForceTimer <= 0f)
            return;

        float progress       = unstickForceTimer / Mathf.Max(0.01f, F.unstickLiftDuration);
        float forceMagnitude = F.unstickLiftForce * progress;

        rb.AddForce(unstickForceDir * forceMagnitude, ForceMode.Acceleration);

        unstickForceTimer = Mathf.Max(0f, unstickForceTimer - Time.fixedDeltaTime);
    }

#if UNITY_EDITOR
    // -------------------------------------------------------------------------
    // 🎨 Unstick Debug Gizmos
    // -------------------------------------------------------------------------
    /// <summary>
    /// Live recovery state in the Scene view during play.
    ///
    ///   Hover point spheres: green (ray hitting ground) / red (no hit)
    ///   Cyan line:           groundContactNormal direction while contacting
    ///   Yellow bar + label:  unstickTimer progress toward unstickRecoveryDelay
    ///   Orange bar + label:  flipTimer progress toward flipRecoveryDelay
    ///   Magenta sphere:      recovery fired flash, visible for 0.5s
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!ShouldDrawDebug || !Application.isPlaying)
            return;

        // unstickFiredFlashTimer is ticked in Update. Read-only here.

        // --- Hover point spheres ---
        if (hoverPoints != null)
        {
            foreach (Transform point in hoverPoints)
            {
                if (point == null) continue;
                Gizmos.color = IsHoverGrounded ? Color.green : Color.red;
                Gizmos.DrawWireSphere(point.position, 0.15f);
            }
        }

        // --- Ground contact normal ---
        if (IsContactingGround && groundContactNormal.sqrMagnitude > 0.01f)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(transform.position, groundContactNormal * 1.5f);
            Gizmos.DrawWireSphere(transform.position + groundContactNormal * 1.5f, 0.1f);
        }

        // --- Upright unstick timer bar (yellow) ---
        if (IsContactingGround && unstickTimer > 0f && profile != null)
        {
            float progress  = Mathf.Clamp01(unstickTimer / Mathf.Max(0.01f, F.unstickRecoveryDelay));
            Gizmos.color    = Color.Lerp(Color.yellow, new Color(1f, 0.8f, 0f), progress);
            Vector3 start   = transform.position + Vector3.up * 0.3f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);
            Gizmos.DrawWireSphere(end, 0.08f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 1.5f,
                $"Unstick {unstickTimer:F2} / {F.unstickRecoveryDelay:F2}s"
            );
        }

        // --- Flip recovery timer bar (orange) ---
        if (IsContactingGround && flipTimer > 0f && profile != null)
        {
            float progress  = Mathf.Clamp01(flipTimer / Mathf.Max(0.01f, F.flipRecoveryDelay));
            Gizmos.color    = Color.Lerp(new Color(1f, 0.4f, 0f), Color.red, progress);
            Vector3 start   = transform.position + Vector3.up * 0.6f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);
            Gizmos.DrawWireSphere(end, 0.08f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 2.0f,
                $"Flip {flipTimer:F2} / {F.flipRecoveryDelay:F2}s"
            );
        }

        // --- Fired-impulse flash ---
        if (unstickFiredFlashTimer > 0f)
        {
            float alpha     = unstickFiredFlashTimer / 0.5f;
            Gizmos.color    = new Color(1f, 0f, 1f, alpha);
            Gizmos.DrawWireSphere(transform.position, 0.6f);
            Gizmos.DrawWireSphere(transform.position, 0.9f);

            UnityEditor.Handles.color = new Color(1f, 0f, 1f, alpha);
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 2f,
                "UNSTICK FIRED"
            );
        }
    }
#endif
}
