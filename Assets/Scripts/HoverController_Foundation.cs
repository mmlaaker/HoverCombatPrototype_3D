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
    // 🔄 Flip Recovery
    // -------------------------------------------------------------------------
    [Header("🔄 Flip Recovery")]
    [Tooltip("Tilt angle from upright (degrees) that triggers flip recovery. " +
             "90° = sideways, 180° = fully inverted. Recommended: 70–100.")]
    [Range(10f, 180f)]
    [SerializeField] private float flipRecoveryAngleThreshold = 80f;

    [Tooltip("Seconds past the flip threshold before recovery torque fires. " +
             "0 = immediate. A small value (0.5–1s) gives collisions a grace window " +
             "before recovery kicks in, which prevents twitchy response on hard impacts.")]
    [Min(0f)]
    [SerializeField] private float flipRecoveryDelay = 0.75f;

    [Tooltip("Proportional torque strength driving the craft back to upright. " +
             "Needs to be stronger than levelingTorqueStrength to overcome extreme angles. " +
             "Recommended: 20–40.")]
    [Range(0f, 80f)]
    [SerializeField] private float flipRecoveryTorque = 28f;

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // 📡 Collision Tracking (feeds HandleGroundUnstick)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Tracks whether the craft body is currently in contact with a ground-layer object.
    /// OnCollisionStay fires every FixedUpdate while contact persists, so isContactingGround
    /// stays current without a separate polling pass.
    ///
    /// Layer check: we check each contact point's other collider against groundLayers.
    /// This matches the same mask used by the hover raycasts, so "ground" means the same
    /// thing to both systems.
    /// </summary>
    private void OnCollisionStay(Collision collision)
    {
        // Check whether the colliding object is on a ground layer.
        if ((groundLayers.value & (1 << collision.gameObject.layer)) == 0)
            return;

        isContactingGround = true;

        // Cache the average contact normal for potential use in the impulse direction.
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

    // 📌 Ground Unstick
    // -------------------------------------------------------------------------
    [Header("📌 Ground Unstick")]
    [Tooltip("Speed (m/s) below which the craft is considered stationary for unstick purposes. " +
             "Only evaluated when grounded. Recommended: 0.3–0.8.")]
    [Min(0f)]
    [SerializeField] private float unstickSpeedThreshold = 0.5f;

    [Tooltip("Seconds the craft must remain slow and grounded before an unstick impulse fires. " +
             "Prevents triggering on normal low-speed ground contact.")]
    [Min(0f)]
    [SerializeField] private float unstickDelay = 0.4f;

    [Tooltip("Speed (m/s) added along the average ground normal on unstick. " +
             "Enough to break the pin; too high feels teleporty. Recommended: 3–6.")]
    [Min(0f)]
    [SerializeField] private float unstickImpulseStrength = 4f;


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

    private float flipTimer;
    private float   unstickTimer;
    private bool    isContactingGround;  // set by OnCollisionStay/Exit, consumed by HandleGroundUnstick
    private Vector3 groundContactNormal;
    private float   unstickFiredFlashTimer; // drives the fired-impulse gizmo
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
            flipTimer    = 0f;
            unstickTimer = 0f;
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
        HandleFlipRecovery();
        HandleGroundUnstick();
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
    // 🔄 Flip Recovery
    // -------------------------------------------------------------------------
    /// <summary>
    /// When the craft tilts past flipRecoveryAngleThreshold for longer than
    /// flipRecoveryDelay seconds AND is in contact with a ground-layer surface,
    /// applies a proportional righting torque toward Vector3.up.
    ///
    /// Ground contact is required: an airborne inverted craft is a legitimate physics
    /// state that should resolve on landing. Recovering mid-air would fight the
    /// physics solver and feel wrong. isContactingGround is used rather than
    /// IsHoverGrounded because hover rays point away from ground when inverted
    /// and won't hit — collision contact is the only reliable grounded signal here.
    ///
    /// Degeneracy note: at exactly 180° tilt, Cross(transform.up, Vector3.up) is
    /// a zero vector — no preferred righting axis exists mathematically. We break
    /// this by nudging along transform.forward, which biases the craft to pitch
    /// forward rather than spin unpredictably.
    /// </summary>
    private void HandleFlipRecovery()
    {
        if (!recoveryEnabled || flipRecoveryTorque <= 0f)
            return;

        float tiltAngle = Vector3.Angle(transform.up, Vector3.up);

        if (tiltAngle < flipRecoveryAngleThreshold || !isContactingGround)
        {
            flipTimer = 0f;
            return;
        }

        flipTimer += Time.fixedDeltaTime;

        if (flipTimer < flipRecoveryDelay)
            return;

        Vector3 torqueAxis = Vector3.Cross(transform.up, Vector3.up);

        // Break the 180° degeneracy with a forward pitch bias.
        if (torqueAxis.sqrMagnitude < 0.001f)
            torqueAxis = transform.forward * 0.1f;

        rb.AddTorque(torqueAxis.normalized * flipRecoveryTorque, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // 📌 Ground Unstick
    // -------------------------------------------------------------------------
    /// <summary>
    /// Detects when the craft is grounded, nearly stationary, AND in active contact
    /// with a ground-layer surface — indicating it's physically pinned, not just idling
    /// at normal hover height. After unstickDelay seconds in this state, fires a
    /// one-shot velocity addition along the contact normal (or hover normal as fallback).
    ///
    /// Collision is the correct discriminator here: a craft hovering correctly at target
    /// height has no collider contact with the ground. Only a pinned craft does.
    /// Spring compression was the previous discriminator but had false-positive edge cases
    /// on slopes and with asymmetric hover point layouts.
    ///
    /// Why rb.linearVelocity here instead of AddForce:
    /// An impulse force large enough to overcome static friction and spring compression
    /// simultaneously would need to be enormous and often fails across multiple frames
    /// while the spring pushes back. The velocity addition is additive (not a hard set),
    /// preserves lateral momentum, and fires exactly once per stuck event.
    /// </summary>
    private void HandleGroundUnstick()
    {
        if (!recoveryEnabled)
            return;

        bool isSlow      = rb.linearVelocity.sqrMagnitude < unstickSpeedThreshold * unstickSpeedThreshold;

        // isContactingGround is the grounded discriminator here, not IsHoverGrounded.
        // When inverted, hover rays point away from the ground and will never hit —
        // IsHoverGrounded will always be false in exactly the state we need to recover from.
        // Physical collision contact is the only reliable signal when the craft is flipped.
        // isSlow guards against firing during a valid fast landing or active movement.
        if (!isSlow || !isContactingGround)
        {
            unstickTimer = 0f;
            return;
        }

        unstickTimer += Time.fixedDeltaTime;

        if (unstickTimer < unstickDelay)
            return;

        // Reset before applying so the impulse fires once, not continuously.
        unstickTimer = 0f;

        // Prefer the contact normal if valid; fall back to average hover normal.
        // The contact normal points directly away from the surface pressing on us,
        // which is the most reliable escape direction when the craft is pinned.
        Vector3 impulseDir = groundContactNormal.sqrMagnitude > 0.01f
            ? groundContactNormal
            : AverageGroundNormal;

        rb.linearVelocity += impulseDir * unstickImpulseStrength;
        unstickFiredFlashTimer = 0.5f; // keep gizmo visible for half a second

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
    ///   Yellow line + label — unstickTimer progress (grows toward unstickDelay)
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

        // --- Unstick timer progress bar (line that grows toward unstickDelay) ---
        if (isContactingGround && unstickTimer > 0f)
        {
            float progress  = Mathf.Clamp01(unstickTimer / Mathf.Max(0.01f, unstickDelay));
            // Blend yellow -> orange as timer fills
            Gizmos.color    = Color.Lerp(Color.yellow, new Color(1f, 0.4f, 0f), progress);
            Vector3 start   = transform.position + Vector3.up * 0.3f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);

            // Endpoint sphere so it's visible even when progress is small
            Gizmos.DrawWireSphere(end, 0.08f);

            // Label above the craft showing timer value
            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 1.5f,
                $"Unstick: {unstickTimer:F2} / {unstickDelay:F2}s"
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
