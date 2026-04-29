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
/// Physics contract: never writes to rb.angularVelocity, rb.rotation, or
/// rb.linearVelocity. All motion is AddForce / AddTorque.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController_Foundation : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 🧩 Hover Points
    // -------------------------------------------------------------------------
    [Header("🧩 Hover Points")]
    [Tooltip("Empty transforms positioned at the bottom of the chassis (one per corner is typical). " +
             "Each casts a ray straight down to find the ground. Assign by hand from the hover point children.")]
    [SerializeField] private Transform[] hoverPoints;

    // -------------------------------------------------------------------------
    // 🚀 Hover Lift
    // -------------------------------------------------------------------------
    [Header("🚀 Hover Lift")]
    [Tooltip("How far above the ground the chassis floats at rest. Higher values lift the body further off the surface.")]
    [SerializeField] private float hoverHeight = 3f;

    [Tooltip("Stiffness of the hover spring. Higher values make the chassis correct height faster and feel firmer. " +
             "Push too high and the vehicle starts bouncing.")]
    [SerializeField] private float liftStrength = 50000f;

    [Tooltip("Damping on the hover spring. Higher values absorb bounce after landings or slope transitions. " +
             "Pair with Lift Strength.")]
    [SerializeField] private float liftDamping = 5000f;

    [Tooltip("How far the hover rays look down for ground. Set this generous enough that the chassis can drop a small distance before losing lift.")]
    [SerializeField] private float sensorRange = 5f;

    // -------------------------------------------------------------------------
    // 🧮 Slope Lift Compensation
    // -------------------------------------------------------------------------
    [Header("🧮 Slope Lift Compensation")]
    [Tooltip("Adds extra lift on slopes so the chassis doesn't sag into hills. Recommended on for any non-flat terrain.")]
    [SerializeField] private bool enableSlopeLiftCompensation = true;

    [Tooltip("How much extra lift the slope correction adds at the steepest angle. " +
             "1.0 disables the boost, 1.3 is subtle, 1.6 is strong.")]
    [Range(1f, 2f)]
    [SerializeField] private float slopeLiftMultiplier = 1.3f;

    // -------------------------------------------------------------------------
    // ⚖️ Leveling
    // -------------------------------------------------------------------------
    [Header("⚖️ Leveling")]
    [Tooltip("How aggressively the chassis rotates to match the ground beneath it. " +
             "Higher levels out faster but can wobble. Start around 8 to 15.")]
    [Range(0f, 50f)]
    [SerializeField] private float levelingTorqueStrength = 12f;

    [Tooltip("Resistance to tilting on the pitch and roll axes. Pair with Leveling Torque Strength to kill wobble. " +
             "Higher values feel more planted.")]
    [Range(0f, 30f)]
    [SerializeField] private float pitchRollDamping = 8f;

    // -------------------------------------------------------------------------
    // 📌 Ground Unstick  (upright only)
    // -------------------------------------------------------------------------
    // Fires when the chassis is touching ground AND is not flipped.
    // Speed is NOT gated. Unstick must work regardless of landing velocity.
    [Header("📌 Ground Unstick")]
    [Tooltip("How long the chassis can rest belly-down upright before the unstick lift fires. " +
             "Keep this short. This is a physics correction, not a gameplay moment. Try 0.1 to 0.3.")]
    [Min(0f)]
    [SerializeField] private float unstickRecoveryDelay = 0.2f;

    [Tooltip("Strength of the upward push that frees a stuck chassis. " +
             "Front-loaded and fades over the lift window. Mass independent. Try 40 to 80.")]
    [Min(0f)]
    [SerializeField] private float unstickLiftForce = 25f;

    [Tooltip("How long the unstick lift lasts as it tapers to zero. " +
             "Longer reads as a gentle bump, shorter is a quick snap. Try 0.1 to 0.25.")]
    [Min(0.01f)]
    [SerializeField] private float unstickLiftDuration = 0.15f;

    // -------------------------------------------------------------------------
    // 🔄 Flip Recovery  (flipped only)
    // -------------------------------------------------------------------------
    // Fires when the chassis is touching ground AND flipped AND slow.
    // Longer delay is intentional. A flipped vehicle should feel like a setback.
    [Header("🔄 Flip Recovery")]
    [Tooltip("Tilt angle that counts as flipped. 90 is on its side, 180 is fully upside down. Try 70 to 100.")]
    [Range(10f, 180f)]
    [SerializeField] private float flipRecoveryAngleThreshold = 80f;

    [Tooltip("How long the chassis stays flipped before the righting torque kicks in. " +
             "Longer values let the player feel the flip as a real setback. Try 0.75 to 1.5.")]
    [Min(0f)]
    [SerializeField] private float flipRecoveryDelay = 1.0f;

    [Tooltip("Strength of the righting torque that flips the chassis upright. " +
             "Must be stronger than Leveling Torque Strength to win at extreme angles. Try 20 to 40.")]
    [Range(0f, 250f)]
    [SerializeField] private float flipRecoveryTorque = 28f;

    [Tooltip("Speed below which flip recovery is allowed to start. " +
             "Prevents recovery from firing while the chassis is still tumbling. Try 0.5.")]
    [Min(0f)]
    [SerializeField] private float flipRecoverySpeedThreshold = 0.5f;

    // -------------------------------------------------------------------------
    // 🌎 Gravity
    // -------------------------------------------------------------------------
    [Header("🌎 Gravity")]
    [Tooltip("Extra gravity applied at all times. Pulls the chassis down harder onto the hover springs for a heavier feel. 0 disables.")]
    [Range(0f, 5f)]
    [SerializeField] private float extraGravityMultiplier = 0f;

    [Tooltip("Extra downward pull that only applies while airborne. " +
             "Reduces hangtime after jumps and ramps without affecting grounded feel.")]
    [Range(0f, 30f)]
    [SerializeField] private float extraAirGravity = 0f;

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
    /// Tracks ground-layer collision contact. OnCollisionStay fires every physics step
    /// while contact persists, so isContactingGround stays live without polling.
    /// Both flip recovery and unstick read these values; neither writes them.
    /// </summary>
    private void OnCollisionStay(Collision collision)
    {
        if ((groundLayers.value & (1 << collision.gameObject.layer)) == 0)
            return;

        isContactingGround = true;

        int count = collision.GetContacts(_contactBuffer);
        if (count > 0)
        {
            Vector3 normalSum = Vector3.zero;
            for (int i = 0; i < count; i++)
                normalSum += _contactBuffer[i].normal;
            groundContactNormal = (normalSum / count).normalized;
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if ((groundLayers.value & (1 << collision.gameObject.layer)) == 0)
            return;

        isContactingGround  = false;
        groundContactNormal = Vector3.zero;
    }

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private Rigidbody rb;

    // Pre-allocated buffer for OnCollisionStay contact reads. Avoids per-call
    // ContactPoint[] allocation on every physics tick.
    private readonly System.Collections.Generic.List<ContactPoint> _contactBuffer = new(8);

    private float   unstickTimer;            // counts up while upright and contacting ground
    private float   flipTimer;               // counts up while flipped, slow, and contacting ground
    private bool    isContactingGround;      // written by OnCollisionStay/Exit
    private Vector3 groundContactNormal;
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
        rb = GetComponent<Rigidbody>();
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

            if (!Physics.Raycast(point.position, rayDir, out RaycastHit hit, sensorRange, groundLayers))
            {
                if (ShouldDrawDebug)
                    Debug.DrawRay(point.position, rayDir * sensorRange, Color.red);
                continue;
            }

            float compression         = hoverHeight - hit.distance;
            float velocityAlongNormal = Vector3.Dot(rb.GetPointVelocity(point.position), hit.normal);
            float springForce         = compression * liftStrength - velocityAlongNormal * liftDamping;

            if (enableSlopeLiftCompensation)
            {
                float slopeFactor = Mathf.Clamp01(1f - hit.normal.y);
                springForce *= Mathf.Lerp(1f, slopeLiftMultiplier, slopeFactor);
            }

            springForce = Mathf.Max(springForce, 0f);
            rb.AddForceAtPosition(hit.normal * springForce, point.position, ForceMode.Force);

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
        if (extraGravityMultiplier > 0f)
            rb.AddForce(Physics.gravity * extraGravityMultiplier, ForceMode.Acceleration);

        if (!IsHoverGrounded && extraAirGravity > 0f)
            rb.AddForce(Vector3.down * extraAirGravity, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // ⚖️ Torque-Based Ground-Normal Alignment
    // -------------------------------------------------------------------------
    private void ApplyLevelingTorque()
    {
        if (!IsHoverGrounded || levelingTorqueStrength <= 0f)
            return;

        Vector3 torqueAxis = Vector3.Cross(transform.up, AverageGroundNormal);
        rb.AddTorque(torqueAxis * levelingTorqueStrength, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // ⚙️ Torque-Based Pitch/Roll Damping
    // -------------------------------------------------------------------------
    private void ApplyPitchRollDamping()
    {
        if (pitchRollDamping <= 0f)
            return;

        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);

        Vector3 dampingLocal = new Vector3(
            -localAngVel.x * pitchRollDamping,
            0f,
            -localAngVel.z * pitchRollDamping
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
        bool  isFlipped = tiltAngle >= flipRecoveryAngleThreshold;
        bool  isSlow    = rb.linearVelocity.sqrMagnitude < flipRecoverySpeedThreshold * flipRecoverySpeedThreshold;

        // --- Righting torque (runs every frame while authorized) ---
        if (!isFlipped || IsHoverGrounded)
            rightingAuthorized = false;

        if (rightingAuthorized && flipRecoveryTorque > 0f)
        {
            Vector3 torqueAxis = Vector3.Cross(transform.up, Vector3.up);

            if (torqueAxis.sqrMagnitude < 0.001f)
                torqueAxis = transform.right * 0.1f;

            rb.AddTorque(torqueAxis.normalized * flipRecoveryTorque, ForceMode.Acceleration);
        }

        // --- Upright unstick path ---
        // Gate: contacting ground AND upright. Speed intentionally not gated.
        if (isContactingGround && !isFlipped)
        {
            unstickTimer += Time.fixedDeltaTime;

            if (unstickTimer >= unstickRecoveryDelay)
            {
                unstickTimer = 0f;

                unstickForceDir = groundContactNormal.sqrMagnitude > 0.01f
                    ? groundContactNormal
                    : Vector3.up;

                unstickForceTimer      = unstickLiftDuration;
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
        if (isContactingGround && isFlipped && isSlow)
        {
            flipTimer += Time.fixedDeltaTime;

            if (flipTimer >= flipRecoveryDelay)
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

        float progress       = unstickForceTimer / Mathf.Max(0.01f, unstickLiftDuration);
        float forceMagnitude = unstickLiftForce * progress;

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
        if (isContactingGround && groundContactNormal.sqrMagnitude > 0.01f)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(transform.position, groundContactNormal * 1.5f);
            Gizmos.DrawWireSphere(transform.position + groundContactNormal * 1.5f, 0.1f);
        }

        // --- Upright unstick timer bar (yellow) ---
        if (isContactingGround && unstickTimer > 0f)
        {
            float progress  = Mathf.Clamp01(unstickTimer / Mathf.Max(0.01f, unstickRecoveryDelay));
            Gizmos.color    = Color.Lerp(Color.yellow, new Color(1f, 0.8f, 0f), progress);
            Vector3 start   = transform.position + Vector3.up * 0.3f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);
            Gizmos.DrawWireSphere(end, 0.08f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 1.5f,
                $"Unstick {unstickTimer:F2} / {unstickRecoveryDelay:F2}s"
            );
        }

        // --- Flip recovery timer bar (orange) ---
        if (isContactingGround && flipTimer > 0f)
        {
            float progress  = Mathf.Clamp01(flipTimer / Mathf.Max(0.01f, flipRecoveryDelay));
            Gizmos.color    = Color.Lerp(new Color(1f, 0.4f, 0f), Color.red, progress);
            Vector3 start   = transform.position + Vector3.up * 0.6f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);
            Gizmos.DrawWireSphere(end, 0.08f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 2.0f,
                $"Flip {flipTimer:F2} / {flipRecoveryDelay:F2}s"
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
