using UnityEngine;

/// <summary>
/// HoverController_Foundation v3.0
/// --------------------------------
/// Responsibilities:
///   • Per-point spring-damper hover lift
///   • Slope lift compensation
///   • Torque-based ground-normal leveling
///   • Torque-based pitch/roll damping
///   • Flip recovery   — rights the craft when tilt exceeds threshold (torque-based)
///   • Ground unstick  — velocity impulse when craft is pinned against a ground-layer surface
///
/// Physics contract: zero direct writes to rb.angularVelocity or rb.rotation.
/// rb.linearVelocity is written once, additively, only in the ground unstick path.
/// That exception is intentional and documented at the call site.
///
/// External disable hook: SetRecoveryEnabled(bool) suppresses both flip recovery
/// and ground unstick. Use for EMP, special abilities, or scripted events.
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
    // 🔄 Flip Recovery + 📌 Ground Unstick  (shared timer)
    // -------------------------------------------------------------------------
    // Both systems share a single recoveryTimer and a single recoveryDelay threshold.
    // Righting torque and the unstick impulse fire together when the timer is reached.
    // The torque is continuous — it keeps applying after the impulse fires — so no
    // artificial sequencing delay is needed between the two.
    [Header("🔄 Flip Recovery + 📌 Ground Unstick")]
    [Tooltip("Tilt angle from upright (degrees) that activates the recovery sequence. " +
             "90° = sideways, 180° = fully inverted. Recommended: 70–100.")]
    [Range(10f, 180f)]
    [SerializeField] private float flipRecoveryAngleThreshold = 80f;

    [Tooltip("Seconds of ground contact while stuck or flipped before recovery fires. " +
             "Both righting torque and the unstick impulse trigger at this threshold simultaneously. " +
             "Grace window prevents twitchy response on hard impacts. Recommended: 0.5–1.0.")]
    [Min(0f)]
    [SerializeField] private float recoveryDelay = 0.75f;

    [Tooltip("Proportional torque strength driving the craft back to upright. " +
             "Needs to be stronger than levelingTorqueStrength to overcome extreme angles. " +
             "Recommended: 20–40.")]
    [Range(0f, 80f)]
    [SerializeField] private float flipRecoveryTorque = 28f;

    [Tooltip("Speed (m/s) below which unstick will fire. Guards against triggering during active movement.")]
    [Min(0f)]
    [SerializeField] private float unstickSpeedThreshold = 0.5f;

    [Tooltip("Speed (m/s) added along the contact normal on unstick.")]
    [Min(0f)]
    [SerializeField] private float unstickImpulseStrength = 4f;

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

        Vector3 normalSum = Vector3.zero;
        foreach (ContactPoint cp in collision.contacts)
            normalSum += cp.normal;

        if (collision.contactCount > 0)
            groundContactNormal = (normalSum / collision.contactCount).normalized;
    }

    private void OnCollisionExit(Collision collision)
    {
        if ((groundLayers.value & (1 << collision.gameObject.layer)) == 0)
            return;

        isContactingGround  = false;
        groundContactNormal = Vector3.zero;
    }


    // -------------------------------------------------------------------------
    // 🌍 Ground Interaction
    // -------------------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Layers treated as ground for hover detection.")]
    [SerializeField] private LayerMask groundLayers = ~0;

    [Tooltip("Draws hover rays in the scene view.")]
    [SerializeField] private bool drawDebugRays = true;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private Rigidbody rb;

    private float   recoveryTimer;           // shared by flip recovery and ground unstick
    private bool    isContactingGround;      // written by OnCollisionStay/Exit
    private Vector3 groundContactNormal;
    private float   unstickFiredFlashTimer;  // drives the fired-impulse gizmo
    private bool    rightingAuthorized;      // true after unstick fires; cleared when craft rights or lands
    private bool    recoveryEnabled = true;

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
            recoveryTimer      = 0f;
            rightingAuthorized = false;
        }
    }

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
        ApplyLevelingTorque();
        ApplyPitchRollDamping();
        HandleRecovery();
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
                if (drawDebugRays)
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

            if (drawDebugRays)
                Debug.DrawRay(point.position, rayDir * hit.distance, Color.green);
        }

        if (groundedCount > 0)
        {
            IsHoverGrounded     = true;
            AverageGroundNormal = (normalSum / groundedCount).normalized;
        }
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
    // 🔄 Flip Recovery + 📌 Ground Unstick
    // -------------------------------------------------------------------------
    /// <summary>
    /// Two systems with independent preconditions:
    ///
    ///   Unstick impulse  — fires once after recoveryDelay seconds of isContactingGround
    ///                      && isSlow. Resets timer after firing. Sets rightingAuthorized
    ///                      if the craft is flipped at the moment of impulse.
    ///
    ///   Righting torque  — only runs while rightingAuthorized is true. This flag is set
    ///                      exclusively by the unstick impulse, so torque is a direct
    ///                      consequence of being launched — not a general airborne corrector.
    ///                      Clears when the craft rights itself or hover springs re-engage.
    ///
    /// This prevents torque from firing during any unrelated airborne tumble. It only
    /// activates when the craft was specifically launched by the unstick system.
    ///
    /// 180° degeneracy: Cross(transform.up, Vector3.up) is zero when perfectly inverted.
    /// A forward pitch bias breaks the symmetry.
    ///
    /// Unstick uses rb.linearVelocity addition rather than AddForce — an impulse force
    /// large enough to overcome static friction and spring compression simultaneously
    /// would fail across multiple frames. The addition is additive, not a hard set.
    /// </summary>
    private void HandleRecovery()
    {
        if (!recoveryEnabled)
            return;

        float tiltAngle = Vector3.Angle(transform.up, Vector3.up);
        bool  isFlipped = tiltAngle >= flipRecoveryAngleThreshold;
        bool  isSlow    = rb.linearVelocity.sqrMagnitude < unstickSpeedThreshold * unstickSpeedThreshold;

        // --- Righting authorization ---
        // rightingAuthorized is set when an unstick impulse fires.
        // It clears when the craft is no longer flipped, or when it re-establishes
        // hover contact (springs engaged = successfully righted and landed).
        if (!isFlipped || IsHoverGrounded)
            rightingAuthorized = false;

        // --- Righting torque ---
        // Only runs when explicitly authorized by a prior unstick impulse.
        // This prevents torque from firing during any airborne tumble — it's
        // only active as a direct consequence of the craft being launched off the ground.
        if (rightingAuthorized && flipRecoveryTorque > 0f)
        {
            Vector3 torqueAxis = Vector3.Cross(transform.up, Vector3.up);

            // Break 180° degeneracy with a forward pitch bias.
            if (torqueAxis.sqrMagnitude < 0.001f)
                torqueAxis = transform.forward * 0.1f;

            rb.AddTorque(torqueAxis.normalized * flipRecoveryTorque, ForceMode.Acceleration);
        }

        // --- Unstick impulse ---
        // Precondition: in ground contact and slow. Timer-gated.
        // Grace window prevents triggering on hard-but-fast landings still playing out.
        if (!isContactingGround || !isSlow)
        {
            recoveryTimer = 0f;
            return;
        }

        recoveryTimer += Time.fixedDeltaTime;

        if (recoveryTimer < recoveryDelay)
            return;

        recoveryTimer = 0f;

        Vector3 impulseDir = groundContactNormal.sqrMagnitude > 0.01f
            ? groundContactNormal
            : AverageGroundNormal;

        rb.linearVelocity += impulseDir * unstickImpulseStrength;
        unstickFiredFlashTimer = 0.5f;

        // Authorize righting torque for the duration of this launch.
        if (isFlipped)
            rightingAuthorized = true;

        if (drawDebugRays)
            Debug.DrawRay(transform.position, impulseDir * 2f, Color.cyan);
    }

#if UNITY_EDITOR
    // -------------------------------------------------------------------------
    // 🎨 Unstick Debug Gizmos
    // -------------------------------------------------------------------------
    /// <summary>
    /// Draws live unstick state in the Scene view during play mode.
    /// All gizmos are suppressed when drawDebugRays is false.
    ///
    ///   Hover point spheres — green: hover ray hitting ground / red: no hit
    ///   Cyan line           — groundContactNormal direction when isContactingGround
    ///   Yellow line + label — recoveryTimer progress toward recoveryDelay
    ///   Magenta sphere      — fired-impulse flash, visible for 0.5s after impulse
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!drawDebugRays || !Application.isPlaying)
            return;

        // Tick down the flash timer (OnDrawGizmos runs in editor update, not FixedUpdate,
        // so we use unscaled delta time to keep it frame-rate independent).
        unstickFiredFlashTimer = Mathf.Max(0f, unstickFiredFlashTimer - Time.deltaTime);

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

        // --- Recovery timer progress bar ---
        if (isContactingGround && recoveryTimer > 0f)
        {
            float progress  = Mathf.Clamp01(recoveryTimer / Mathf.Max(0.01f, recoveryDelay));
            // Blend yellow -> orange as timer fills
            Gizmos.color    = Color.Lerp(Color.yellow, new Color(1f, 0.4f, 0f), progress);
            Vector3 start   = transform.position + Vector3.up * 0.3f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);

            // Endpoint sphere so it's visible even when progress is small
            Gizmos.DrawWireSphere(end, 0.08f);

            // Label above the craft showing timer value
            UnityEditor.Handles.color = Gizmos.color;
            // Show which phase the shared timer is currently in
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 1.5f,
                $"Recovery {recoveryTimer:F2} / {recoveryDelay:F2}s"
            );
        }

        // --- Fired-impulse flash ---
        if (unstickFiredFlashTimer > 0f)
        {
            float alpha     = unstickFiredFlashTimer / 0.5f; // fade out over 0.5s
            Gizmos.color    = new Color(1f, 0f, 1f, alpha);  // magenta
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
