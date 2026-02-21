using UnityEngine;

/// <summary>
/// HoverController_Foundation v2.0
/// --------------------------------
/// Responsibilities:
///   • Per-point spring-damper hover lift
///   • Slope lift compensation
///   • Torque-based ground-normal leveling  (replaces MoveRotation)
///   • Torque-based pitch/roll damping      (replaces angularVelocity write)
///
/// Physics contract: zero direct writes to rb.angularVelocity or rb.rotation.
/// All stabilization is expressed as AddTorque so the physics solver owns integration.
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
    // ⚖️ Leveling (torque-based)
    // -------------------------------------------------------------------------
    [Header("⚖️ Leveling")]
    [Tooltip("How aggressively the craft torques toward the ground normal. " +
             "Acts as a proportional gain — higher values correct faster but may oscillate.")]
    [Range(0f, 50f)]
    [SerializeField] private float levelingTorqueStrength = 12f;

    [Tooltip("Damping applied to pitch/roll angular velocity. " +
             "Pair with levelingTorqueStrength to avoid oscillation. " +
             "Higher = more resistance to tilting.")]
    [Range(0f, 30f)]
    [SerializeField] private float pitchRollDamping = 8f;

    // -------------------------------------------------------------------------
    // 🌍 Ground Interaction
    // -------------------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Layers treated as ground for hover detection.")]
    [SerializeField] private LayerMask groundLayers = ~0;

    [Tooltip("Draws hover rays in the scene view.")]
    [SerializeField] private bool drawDebugRays = true;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private Rigidbody rb;

    /// <summary>True when at least one hover point has a ground hit this frame.</summary>
    public bool IsHoverGrounded { get; private set; }

    /// <summary>Average ground normal this frame (Vector3.up when airborne).</summary>
    public Vector3 AverageGroundNormal { get; private set; }

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
    }

    // -------------------------------------------------------------------------
    // 🧠 Per-Point Spring-Damper Lift
    // -------------------------------------------------------------------------
    private void ApplyHoverForces()
    {
        IsHoverGrounded  = false;
        AverageGroundNormal = Vector3.up;

        Vector3 normalSum  = Vector3.zero;
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

            // --- Spring-damper ---
            float compression         = hoverHeight - hit.distance;
            float velocityAlongNormal = Vector3.Dot(rb.GetPointVelocity(point.position), hit.normal);
            float springForce         = compression * liftStrength - velocityAlongNormal * liftDamping;

            // Slope compensation: lift loss on inclines is proportional to how far the
            // normal tilts from vertical (normal.y == 1 on flat, 0 on a wall).
            if (enableSlopeLiftCompensation)
            {
                float slopeFactor  = Mathf.Clamp01(1f - hit.normal.y);
                springForce *= Mathf.Lerp(1f, slopeLiftMultiplier, slopeFactor);
            }

            springForce = Mathf.Max(springForce, 0f); // never pull toward ground
            rb.AddForceAtPosition(hit.normal * springForce, point.position, ForceMode.Force);

            normalSum += hit.normal;
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
    /// <summary>
    /// Computes the shortest rotation from transform.up to the ground normal,
    /// then applies it as a proportional torque. The physics solver integrates this
    /// normally, so it respects collisions and other forces — unlike MoveRotation.
    /// </summary>
    private void ApplyLevelingTorque()
    {
        if (!IsHoverGrounded || levelingTorqueStrength <= 0f)
            return;

        // Axis-angle of the error rotation in world space.
        Vector3 torqueAxis = Vector3.Cross(transform.up, AverageGroundNormal);

        // torqueAxis.magnitude ≈ sin(angle); small at low tilt, large at high tilt.
        // Proportional control: torque = k * error.
        rb.AddTorque(torqueAxis * levelingTorqueStrength, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // ⚙️ Torque-Based Pitch/Roll Damping
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies a counter-torque proportional to pitch and roll angular velocity.
    /// Yaw (y-axis) is intentionally excluded — that's Propulsion's domain.
    /// Uses AddTorque so the solver integrates it with everything else.
    /// </summary>
    private void ApplyPitchRollDamping()
    {
        if (pitchRollDamping <= 0f)
            return;

        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);

        // Counter-torque on X (pitch) and Z (roll) only.
        Vector3 dampingLocal = new Vector3(
            -localAngVel.x * pitchRollDamping,
            0f,
            -localAngVel.z * pitchRollDamping
        );

        rb.AddRelativeTorque(dampingLocal, ForceMode.Acceleration);
    }
}
