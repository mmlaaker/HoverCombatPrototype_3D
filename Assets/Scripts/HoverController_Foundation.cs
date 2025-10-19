using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class HoverController_Foundation : MonoBehaviour
{
    // ------------------------------------------------------------
    // 🚀 Hover Lift Settings
    // ------------------------------------------------------------
    [Header("🚀 Hover Lift Settings")]
    [Tooltip("Target hover height above the ground (in meters).")]
    [SerializeField] private float hoverHeight = 3f;

    [Tooltip("Spring strength controlling how strongly the vehicle maintains hover height.")]
    [SerializeField] private float liftStrength = 50000f;

    [Tooltip("Damping applied to hover spring movement. Higher = less bounce.")]
    [SerializeField] private float liftDamping = 5000f;

    [Tooltip("Maximum raycast distance for ground detection.")]
    [SerializeField] private float sensorRange = 5f;

    [Tooltip("Controls how strongly the craft aligns to the average ground normal (0–1).")]
    [Range(0f, 1f)] [SerializeField] private float selfLevelStrength = 0.5f;

    // ------------------------------------------------------------
    // 🧲 Stability & Recovery
    // ------------------------------------------------------------
    [Header("🧲 Stability & Recovery")]
    [Tooltip("Maximum step offset the hover can recover from before resetting.")]
    [SerializeField] private float maxStepRecovery = 0.05f;

    [Tooltip("Baseline lift applied even when not grounded (keeps craft from sinking).")]
    [SerializeField] private float idleLiftBias = 0.015f;

    [Tooltip("Min/Max ground effect multiplier range for extra stability near surfaces.")]
    [SerializeField] private Vector2 groundEffectRange = new Vector2(0.35f, 2f);

    [Tooltip("How quickly the hovercraft realigns upright after tilting.")]
    [SerializeField] private float uprightRecoverySpeed = 3f;

    [Tooltip("Maximum velocity at which separation recovery still applies.")]
    [SerializeField] private float separationVelocityLimit = 5f;

    // ------------------------------------------------------------
    // 🧮 Slope Lift Compensation
    // ------------------------------------------------------------
    [Header("🧮 Slope Lift Compensation")]
    [Tooltip("If enabled, increases lift on steep slopes to prevent power loss.")]
    [SerializeField] private bool enableSlopeLiftCompensation = true;

    [Tooltip("Multiplier for lift boost on steep slopes (1.0 = none, 1.3 = subtle, 1.6 = strong).")]
    [Range(1f, 2f)] [SerializeField] private float slopeLiftMultiplier = 1.3f;

    // ------------------------------------------------------------
    // ⚙️ Torque Damping & Limits
    // ------------------------------------------------------------
    [Header("⚙️ Torque Damping & Limits")]
    [Tooltip("Damps pitch and roll oscillation without affecting yaw turning.")]
    [Range(0f, 10f)] [SerializeField] private float angularDampingMultiplier = 5f;

    [Tooltip("Clamp torque magnitude applied per kg of mass (helps avoid flipping).")]
    [SerializeField] private float torqueClampPerKg = 10f;

    // ------------------------------------------------------------
    // 🌍 Ground Interaction
    // ------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Layers considered as ground for hover detection.")]
    [SerializeField] private LayerMask groundLayers = ~0;

    [Tooltip("Draws debug rays and normals in scene view.")]
    [SerializeField] private bool drawDebugRays = true;

    // ------------------------------------------------------------
    // Runtime fields
    // ------------------------------------------------------------
    private Rigidbody rb;
    private Transform[] hoverPoints;
    public bool IsHoverGrounded { get; private set; }
    private Vector3 avgNormal;
    private float lastHitDistance;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        // Auto-collect hover points from children tagged "HoverPoint" or similar.
        hoverPoints = GetComponentsInChildren<Transform>();
        if (hoverPoints == null || hoverPoints.Length <= 1)
            Debug.LogWarning("[Foundation] No hover points found — defaulting to object transform.");
    }

    void FixedUpdate()
    {
        ApplyHoverForces();
        ApplyTorqueDamping();
    }

    // ------------------------------------------------------------
    // 🧠 Core Hover Logic
    // ------------------------------------------------------------
    private void ApplyHoverForces()
    {
        if (hoverPoints == null || hoverPoints.Length == 0)
        {
            hoverPoints = new Transform[] { transform };
        }

        Vector3 totalForce = Vector3.zero;
        avgNormal = Vector3.zero;
        IsHoverGrounded = false;

        foreach (var point in hoverPoints)
        {
            if (point == transform) continue;

            if (Physics.Raycast(point.position, -point.up, out RaycastHit hit, sensorRange, groundLayers))
            {
                IsHoverGrounded = true;
                avgNormal += hit.normal;

                // Spring physics
                float compression = hoverHeight - hit.distance;
                float velocityAlongNormal = Vector3.Dot(rb.GetPointVelocity(point.position), hit.normal);

                float springForce = (compression * liftStrength) - (velocityAlongNormal * liftDamping);

                // 🧮 Apply slope compensation
                if (enableSlopeLiftCompensation)
                {
                    float slopeFactor = Mathf.Clamp01(1f - hit.normal.y);
                    float slopeBoost = Mathf.Lerp(1f, slopeLiftMultiplier, slopeFactor);
                    springForce *= slopeBoost;
                }

                springForce = Mathf.Max(springForce, 0f);
                rb.AddForceAtPosition(hit.normal * springForce, point.position, ForceMode.Force);

                if (drawDebugRays)
                    Debug.DrawRay(point.position, -point.up * hit.distance, Color.green);
            }
            else
            {
                if (drawDebugRays)
                    Debug.DrawRay(point.position, -point.up * sensorRange, Color.red);
            }
        }

        if (IsHoverGrounded && avgNormal != Vector3.zero)
        {
            avgNormal.Normalize();
            ApplyLevelingTorque(avgNormal);
        }
    }

    // ------------------------------------------------------------
    // 🧍 Leveling
    // ------------------------------------------------------------
    private void ApplyLevelingTorque(Vector3 groundNormal)
    {
        Quaternion currentRot = transform.rotation;
        Quaternion targetRot = Quaternion.FromToRotation(transform.up, groundNormal) * currentRot;
        Quaternion newRot = Quaternion.Slerp(currentRot, targetRot, selfLevelStrength);
        rb.MoveRotation(newRot);
    }

    // ------------------------------------------------------------
    // ⚖️ Angular Damping
    // ------------------------------------------------------------
    private void ApplyTorqueDamping()
    {
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        localAngVel.x *= 1f / (1f + angularDampingMultiplier * Time.fixedDeltaTime);
        localAngVel.z *= 1f / (1f + angularDampingMultiplier * Time.fixedDeltaTime);
        rb.angularVelocity = transform.TransformDirection(localAngVel);
    }
}
