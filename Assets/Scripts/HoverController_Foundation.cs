using UnityEngine;

/// <summary>
/// HoverController_Foundation v3.4 (Force Mode Edition)
/// ----------------------------------------------------
/// Core hover physics with selectable force application mode:
/// • SingleSpring → one averaged spring (smooth & stable)
/// • MultiPoint   → per-hover-point springs (physical & reactive)
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Traversal))]
public class HoverController_Foundation : MonoBehaviour
{
    // ------------------------------------------------------------
    // 💡 Hover Force Mode
    // ------------------------------------------------------------
    public enum HoverForceMode { SingleSpring, MultiPoint }

    [Header("💡 Hover Force Mode")]
    [Tooltip("SingleSpring: one averaged lift force at the center. MultiPoint: applies spring forces at each hover point.")]
    public HoverForceMode forceMode = HoverForceMode.SingleSpring;

    // ------------------------------------------------------------
    // 🚀 Hover Lift Settings
    // ------------------------------------------------------------
    [Header("🚀 Hover Lift Settings")]
    [Tooltip("Target hover height above the ground.")]
    public float hoverHeight = 3f;

    [Tooltip("How stiff the hover spring feels. Higher = firmer, faster rebound.")]
    public float liftStrength = 40000f;

    [Tooltip("How much vertical damping reduces oscillations. Higher = heavier feel.")]
    public float liftDamping = 8000f;

    [Tooltip("Maximum sensor distance below hover points.")]
    public float sensorRange = 5f;

    [Tooltip("How strongly the craft aligns upright to the ground normal.")]
    [Range(0f, 1f)] public float selfLevelStrength = 0.3f;

    // ------------------------------------------------------------
    // 🧲 Stability & Recovery
    // ------------------------------------------------------------
    [Header("🧲 Stability & Recovery")]
    [Tooltip("Minimum constant lift applied even when at max extension.")]
    public float idleLiftBias = 0.015f;

    [Tooltip("Range scaling for lift falloff near ground.")]
    public Vector2 groundEffectRange = new Vector2(0.35f, 2f);

    [Tooltip("How quickly the craft returns upright after losing ground contact.")]
    public float uprightRecoverySpeed = 3f;

    [Tooltip("Vertical velocity threshold before recovery lift engages.")]
    public float separationVelocityLimit = 5f;

    // ------------------------------------------------------------
    // ⚙️ Torque Damping & Limits
    // ------------------------------------------------------------
    [Header("⚙️ Torque Damping & Limits")]
    [Tooltip("Extra damping applied to pitch and roll. Yaw remains free for steering.")]
    [Range(0f, 5f)] public float angularDampingMultiplier = 2.5f;

    [Tooltip("Caps the maximum corrective torque applied for leveling (N·m/kg).")]
    [Range(0f, 50f)] public float torqueClampPerKg = 10f;

    // ------------------------------------------------------------
    // 🌍 Ground Interaction
    // ------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Physics layers considered valid ground.")]
    public LayerMask groundLayers = -1;

    [Tooltip("Draw debug rays and orientation vectors in Scene view.")]
    public bool drawDebugRays = true;

    // ------------------------------------------------------------
    // Runtime
    // ------------------------------------------------------------
    private Rigidbody rb;
    private HoverController_Traversal traversal;
    private Vector3 smoothedNormal = Vector3.up;
    private bool isGrounded;

    public bool IsHoverGrounded => isGrounded;
    public Vector3 CurrentNormal => smoothedNormal;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        traversal = GetComponent<HoverController_Traversal>();
    }

    void FixedUpdate() => UpdateHoverPhysics();

    // ------------------------------------------------------------
    // Core Hover Physics
    // ------------------------------------------------------------
    private void UpdateHoverPhysics()
    {
        smoothedNormal = traversal.GroundNormal;
        isGrounded = traversal.IsGrounded;

        if (!isGrounded)
        {
            ApplyRecoveryLift();
            return;
        }

        switch (forceMode)
        {
            case HoverForceMode.SingleSpring:
                ApplySingleSpringLift();
                break;

            case HoverForceMode.MultiPoint:
                ApplyMultiPointLift();
                break;
        }

        ApplyLevelingTorque(smoothedNormal);
    }

    // ------------------------------------------------------------
    // 🌀 Single-Spring Mode
    // ------------------------------------------------------------
    private void ApplySingleSpringLift()
    {
        Vector3 worldUp = smoothedNormal;
        float currentHeight = Mathf.Clamp(traversal.AverageGroundHeight, 0f, sensorRange);
        float heightError = hoverHeight - currentHeight;

        float springForce = liftStrength * heightError;
        float verticalVelocity = Vector3.Dot(rb.linearVelocity, worldUp);
        float dampingForce = -liftDamping * verticalVelocity;

        Vector3 totalLift = worldUp * (springForce + dampingForce);
        totalLift = Vector3.ClampMagnitude(totalLift, liftStrength * hoverHeight);
        rb.AddForce(totalLift, ForceMode.Force);

        if (drawDebugRays)
            Debug.DrawRay(transform.position, worldUp * hoverHeight, Color.green);
    }

    // ------------------------------------------------------------
    // 🧱 Multi-Point Mode
    // ------------------------------------------------------------
    private void ApplyMultiPointLift()
    {
        if (traversal.hoverPoints == null || traversal.hoverPoints.Length == 0)
            return;

        foreach (Transform p in traversal.hoverPoints)
        {
            if (Physics.Raycast(p.position, Vector3.down, out RaycastHit hit, sensorRange, groundLayers))
            {
                float heightError = hoverHeight - hit.distance;
                float springForce = liftStrength * heightError;

                float verticalVel = Vector3.Dot(rb.GetPointVelocity(p.position), transform.up);
                float dampingForce = -liftDamping * verticalVel;

                Vector3 lift = transform.up * (springForce + dampingForce);
                rb.AddForceAtPosition(lift, p.position, ForceMode.Force);

#if UNITY_EDITOR
                if (drawDebugRays)
                    Debug.DrawLine(p.position, hit.point, Color.green);
#endif
            }
            else if (drawDebugRays)
            {
                Debug.DrawLine(p.position, p.position + Vector3.down * sensorRange, Color.red);
            }
        }
    }

    // ------------------------------------------------------------
    // 🔄 Leveling Torque (Directionally Damped)
    // ------------------------------------------------------------
    private void ApplyLevelingTorque(Vector3 targetNormal)
    {
        Vector3 currentUp = transform.up;
        Vector3 torqueAxis = Vector3.Cross(currentUp, targetNormal);
        float alignmentAngle = Mathf.Asin(Mathf.Clamp(torqueAxis.magnitude, -1f, 1f));

        Vector3 correctiveTorque = torqueAxis.normalized *
                                   alignmentAngle *
                                   selfLevelStrength *
                                   liftStrength *
                                   Time.fixedDeltaTime;

        float torqueClamp = rb.mass * torqueClampPerKg;
        correctiveTorque = Vector3.ClampMagnitude(correctiveTorque, torqueClamp);
        rb.AddTorque(correctiveTorque, ForceMode.Acceleration);

        // Directional damping (pitch/roll only)
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        localAngVel.x *= (selfLevelStrength * angularDampingMultiplier);
        localAngVel.z *= (selfLevelStrength * angularDampingMultiplier);
        localAngVel.y = 0f;
        Vector3 worldAngDamp = -transform.TransformDirection(localAngVel);
        rb.AddTorque(worldAngDamp, ForceMode.Acceleration);

#if UNITY_EDITOR
        if (drawDebugRays)
        {
            Vector3 origin = transform.position;
            Debug.DrawRay(origin, currentUp * 2f, Color.yellow);
            Debug.DrawRay(origin, targetNormal * 2f, Color.cyan);
            Debug.DrawRay(origin, torqueAxis * 2f, Color.magenta);
        }
#endif
    }

    // ------------------------------------------------------------
    // 🧯 Recovery Lift (Ungrounded)
    // ------------------------------------------------------------
    private void ApplyRecoveryLift()
    {
        if (rb.linearVelocity.y < -separationVelocityLimit)
        {
            Vector3 recoveryForce = Vector3.up * liftStrength * idleLiftBias;
            rb.AddForce(recoveryForce, ForceMode.Force);
        }

        smoothedNormal = Vector3.Slerp(smoothedNormal, Vector3.up, Time.fixedDeltaTime * uprightRecoverySpeed);

        if (drawDebugRays)
            Debug.DrawRay(transform.position, smoothedNormal * hoverHeight, Color.yellow);
    }
}
