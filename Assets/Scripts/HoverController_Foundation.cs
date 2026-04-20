using UnityEngine;

/// <summary>
/// HoverController_Foundation v4.3
/// --------------------------------
/// Responsibilities:
///   • Per-point spring-damper hover lift
///   • Slope lift compensation
///   • Torque-based ground-normal leveling
///   • Torque-based pitch/roll damping
///   • Flip recovery   — rights the craft when tilt exceeds threshold (torque-based)
///   • Ground unstick  — sustained upward AddForce when craft is pinned upright
///   • Extra gravity   — persistent downforce and air gravity tuning knobs
///
/// Physics contract: zero direct writes to rb.angularVelocity, rb.rotation,
/// or rb.linearVelocity. All motion expressed as AddForce / AddTorque.
///
/// Recovery is split into two independent paths with separate delays:
///   • Upright stuck  — isContactingGround AND NOT isFlipped. Speed not gated.
///                      Fires a sustained upward AddForce window. Near-instant delay.
///   • Flipped        — isContactingGround AND isFlipped AND isSlow.
///                      Fires righting torque. Longer delay for consequence feel.
///
/// External disable hook: SetRecoveryEnabled(bool) suppresses both paths.
/// Use for EMP effects, scripted events, or ability interactions.
///
/// v4.3 changes:
///   • unstickFiredFlashTimer decrement moved from OnDrawGizmos into a new Update()
///     method. OnDrawGizmos is called on the editor's schedule (not game framerate)
///     and should never mutate state. Timer now ticks reliably at game framerate.
///
/// v4.2 changes:
///   • unstickSpeedThreshold renamed to flipRecoverySpeedThreshold — clarifies
///     it only gates the flip path. Tooltip updated to match.
///   • unstickLiftDuration added as inspector field — exposes the lift window
///     duration that was previously hardcoded at 0.15f in ApplyUnstickForce.
///     Allows per-vehicle tuning without touching code.
///
/// v4.1 changes:
///   • Recovery split into two independent paths (upright stuck vs flipped).
///   • Separate inspector delays: unstickRecoveryDelay / flipRecoveryDelay.
///   • unstickImpulseStrength renamed to unstickLiftForce — now drives sustained
///     AddForce rather than a single-frame linearVelocity addition.
///   • unstickSpeedThreshold retained for flipped path only — removed from upright path.
///   • rightingAuthorized is now set by the flipped timer directly, not as a
///     side effect of the unstick impulse.
///   • rb.linearVelocity write removed entirely. Physics contract now fully clean.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController_Foundation : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 🧩 Hover Points
    // -------------------------------------------------------------------------
    [Header("🧩 Hover Points")]
    [Tooltip("Ray origins used for ground detection (e.g. HoverPoint_FL/FR/RL/RR). " +
             "Must be assigned manually — no runtime auto-fill.")]
    [SerializeField] private Transform[] hoverPoints;

    // -------------------------------------------------------------------------
    // 🚀 Hover Lift
    // -------------------------------------------------------------------------
    [Header("🚀 Hover Lift")]
    [Tooltip("Target hover height above the ground (metres).")]
    [SerializeField] private float hoverHeight = 3f;

    [Tooltip("Spring stiffness. Higher = snappier height correction.")]
    [SerializeField] private float liftStrength = 50000f;

    [Tooltip("Spring damping. Higher = less vertical bounce.")]
    [SerializeField] private float liftDamping = 5000f;

    [Tooltip("Maximum raycast distance for ground detection.")]
    [SerializeField] private float sensorRange = 5f;

    // -------------------------------------------------------------------------
    // 🧮 Slope Lift Compensation
    // -------------------------------------------------------------------------
    [Header("🧮 Slope Lift Compensation")]
    [Tooltip("Boosts lift on steep slopes to prevent the craft sinking into inclines.")]
    [SerializeField] private bool enableSlopeLiftCompensation = true;

    [Tooltip("Max lift multiplier on a vertical surface (1.0 = off, 1.3 = subtle, 1.6 = strong).")]
    [Range(1f, 2f)]
    [SerializeField] private float slopeLiftMultiplier = 1.3f;

    // -------------------------------------------------------------------------
    // ⚖️ Leveling
    // -------------------------------------------------------------------------
    [Header("⚖️ Leveling")]
    [Tooltip("Proportional gain driving transform.up toward the ground normal. " +
             "Higher = faster correction, but risks oscillation. Start: 8–15.")]
    [Range(0f, 50f)]
    [SerializeField] private float levelingTorqueStrength = 12f;

    [Tooltip("Counter-torque on pitch and roll axes. Pair with levelingTorqueStrength " +
             "to kill oscillation. Higher = more resistance to tilting.")]
    [Range(0f, 30f)]
    [SerializeField] private float pitchRollDamping = 8f;

    // -------------------------------------------------------------------------
    // 📌 Ground Unstick  (upright only)
    // -------------------------------------------------------------------------
    // Fires when the craft is contacting ground AND is not flipped.
    // Speed is not gated — unstick must work regardless of landing velocity.
    // A sustained upward AddForce window lifts the craft free without a visible punch.
    [Header("📌 Ground Unstick")]
    [Tooltip("Seconds of ground contact while upright before unstick lift fires. " +
             "Near-instant recommended — this is a physics correction, not a gameplay consequence. " +
             "Recommended: 0.1–0.3.")]
    [Min(0f)]
    [SerializeField] private float unstickRecoveryDelay = 0.2f;

    [Tooltip("Upward force magnitude (m/s²) applied during the unstick window. " +
             "Uses ForceMode.Acceleration — mass-independent. " +
             "Front-loaded and tapering. Recommended: 40–80.")]
    [Min(0f)]
    [SerializeField] private float unstickLiftForce = 25f;

    [Tooltip("Duration (seconds) of the sustained lift window. " +
             "Force tapers from full to zero over this time. " +
             "Longer = gentler and more visible. Shorter = snappier. Recommended: 0.1–0.25.")]
    [Min(0.01f)]
    [SerializeField] private float unstickLiftDuration = 0.15f;

    // -------------------------------------------------------------------------
    // 🔄 Flip Recovery  (flipped only)
    // -------------------------------------------------------------------------
    // Fires when the craft is contacting ground AND is flipped AND is slow.
    // Longer delay is intentional — a flipped vehicle should feel like a consequence.
    [Header("🔄 Flip Recovery")]
    [Tooltip("Tilt angle from upright (degrees) that classifies the craft as flipped. " +
             "90° = sideways, 180° = fully inverted. Recommended: 70–100.")]
    [Range(10f, 180f)]
    [SerializeField] private float flipRecoveryAngleThreshold = 80f;

    [Tooltip("Seconds of ground contact while flipped and slow before righting torque fires. " +
             "Longer values give the flip weight and consequence. Recommended: 0.75–1.5.")]
    [Min(0f)]
    [SerializeField] private float flipRecoveryDelay = 1.0f;

    [Tooltip("Proportional torque strength driving the craft back to upright. " +
             "Needs to be stronger than levelingTorqueStrength to overcome extreme angles. " +
             "Recommended: 20–40.")]
    [Range(0f, 250f)]
    [SerializeField] private float flipRecoveryTorque = 28f;

    [Tooltip("Speed (m/s) below which flip recovery will fire. " +
             "Guards against triggering while the craft is still tumbling. Recommended: 0.5.")]
    [Min(0f)]
    [SerializeField] private float flipRecoverySpeedThreshold = 0.5f;

    // -------------------------------------------------------------------------
    // 🌎 Gravity
    // -------------------------------------------------------------------------
    [Header("🌎 Gravity")]
    [Tooltip("Multiplier added on top of Unity gravity (Acceleration mode). " +
             "Increases overall downforce at all times. 0 = no extra.")]
    [Range(0f, 5f)]
    [SerializeField] private float extraGravityMultiplier = 0f;

    [Tooltip("Additional downward acceleration (m/s²) applied only while airborne. " +
             "Reduces floatiness after jumps and off ramps without affecting grounded feel.")]
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

    [Tooltip("Optional global debug toggle. When assigned, overrides drawDebugRays. " +
             "Create via Assets > Create > Hover > Debug Settings.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.enableDebugGizmos : drawDebugRays;

    // -------------------------------------------------------------------------
    // 📡 Collision Tracking
    // -------------------------------------------------------------------------
    /// <summary>
    /// Tracks ground-layer collision contact. OnCollisionStay fires every physics step
    /// while contact persists — isContactingGround stays live without polling.
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

    // Pre-allocated buffer for OnCollisionStay contact point reads.
    // Avoids per-call ContactPoint[] allocation on every physics tick.
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
    /// Use this for EMP effects, scripted events, or ability interactions.
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
        // Tick the debug flash timer in Update so it runs at game framerate,
        // not at editor-gizmo-draw rate (which is unpredictable and can be
        // called multiple times per frame or suppressed entirely).
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
    /// extraGravityMultiplier applies at all times — increases the effective weight
    /// of the craft and helps it sit more firmly on the hover springs.
    /// extraAirGravity applies only while airborne — pulls the craft back down faster
    /// after jumps and off ramps without affecting grounded stability.
    /// Both are tuning knobs for the physical character of the hover, not player input.
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
    ///   Upright unstick — isContactingGround AND NOT isFlipped.
    ///                     Speed not gated — must recover regardless of landing velocity.
    ///                     After unstickRecoveryDelay, begins a sustained upward AddForce
    ///                     window (ApplyUnstickForce). Near-instant delay recommended.
    ///
    ///   Flip recovery   — isContactingGround AND isFlipped AND isSlow.
    ///                     After flipRecoveryDelay, sets rightingAuthorized which gates
    ///                     the righting torque applied each FixedUpdate until the craft
    ///                     rights itself or hover springs re-engage.
    ///                     Longer delay gives the flip consequence feel.
    ///
    /// rightingAuthorized is now set directly by the flip timer — no longer a
    /// side effect of the unstick path. The two systems are fully decoupled.
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
                torqueAxis = transform.forward * 0.1f;

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
    /// Applies the unstick lift force over a short window each FixedUpdate.
    /// unstickForceTimer is set by HandleRecovery when the upright stuck condition is met.
    /// Force is front-loaded — strongest on the first frame, tapering to zero.
    /// This reads as a gentle invisible lift rather than a visible punch.
    /// ForceMode.Acceleration keeps the behavior mass-independent across all vehicles.
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
    /// Draws live recovery state in the Scene view during play mode.
    /// All gizmos are suppressed when ShouldDrawDebug is false.
    ///
    ///   Hover point spheres — green: hover ray hitting ground / red: no hit
    ///   Cyan line           — groundContactNormal direction when isContactingGround
    ///   Yellow bar + label  — unstickTimer progress toward unstickRecoveryDelay (upright)
    ///   Orange bar + label  — flipTimer progress toward flipRecoveryDelay (flipped)
    ///   Magenta sphere      — recovery fired flash, visible for 0.5s
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!ShouldDrawDebug || !Application.isPlaying)
            return;

        // unstickFiredFlashTimer is ticked in Update — read-only here.

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
