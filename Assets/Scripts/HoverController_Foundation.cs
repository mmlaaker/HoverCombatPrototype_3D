using UnityEngine;

/// <summary>
/// HoverController_Foundation v3.3 (Directionally Damped Edition)
/// ---------------------------------------------------------------
/// Core hover physics for spring–damper lift and leveling torque.
/// • Directional angular damping (pitch/roll only — yaw unaffected)
/// • Same torque clamping for over-rotation safety
/// • Clean Designer/Tech-Art layout with color-coded debug gizmos
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Traversal))]
public class HoverController_Foundation : MonoBehaviour
{
    // ------------------------------------------------------------
    // 🚀 Hover Lift Settings
    // ------------------------------------------------------------
    [Header("🚀 Hover Lift Settings")]
    [Tooltip("Target distance the craft maintains above the ground surface (in meters).")]
    public float hoverHeight = 3f;

    [Tooltip("How stiff the invisible hover springs feel. Higher = firmer, snappier lift.")]
    public float liftStrength = 40000f;

    [Tooltip("How quickly the craft stops bouncing after impacts. Higher = tighter damping.")]
    public float liftDamping = 8000f;

    [Tooltip("Maximum range the hover sensors can detect ground below the craft.")]
    public float sensorRange = 5f;

    [Tooltip("How strongly the craft tries to stay upright relative to ground slope.")]
    [Range(0f, 1f)] public float selfLevelStrength = 0.3f;

    // ------------------------------------------------------------
    // 🧲 Stability & Recovery
    // ------------------------------------------------------------
    [Header("🧲 Stability & Recovery")]
    [Tooltip("Maximum vertical correction per physics step used to recover from loss of contact.")]
    public float maxStepRecovery = 0.05f;

    [Tooltip("Minimum constant lift applied even when the craft is fully extended.")]
    public float idleLiftBias = 0.015f;

    [Tooltip("Distance range for dynamic lift scaling (inner = stronger, outer = weaker).")]
    public Vector2 groundEffectRange = new Vector2(0.35f, 2f);

    [Tooltip("How quickly the craft realigns upright after losing ground.")]
    public float uprightRecoverySpeed = 3f;

    [Tooltip("Velocity threshold beyond which recovery lift starts applying.")]
    public float separationVelocityLimit = 5f;

    // ------------------------------------------------------------
    // ⚙️ Torque Damping & Limits
    // ------------------------------------------------------------
    [Header("⚙️ Torque Damping & Limits")]
    [Tooltip("Extra damping applied to pitch and roll to prevent additive wobble. Yaw is left untouched for steering.")]
    [Range(0f, 5f)] public float angularDampingMultiplier = 2f;

    [Tooltip("Caps the maximum torque magnitude applied for self-leveling (in N·m/kg). Prevents overcorrection on ramps.")]
    [Range(0f, 50f)] public float torqueClampPerKg = 10f;

    // ------------------------------------------------------------
    // 🌍 Ground Interaction
    // ------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Physics layers the hover sensors detect as 'ground'.")]
    public LayerMask groundLayers = -1;

    [Tooltip("Draws debug rays and normals for visualization.")]
    public bool drawDebugRays = true;

    // ------------------------------------------------------------
    // Runtime / Read-only
    // ------------------------------------------------------------
    private Rigidbody rb;
    private HoverController_Traversal traversal;
    private Vector3 smoothedNormal = Vector3.up;
    private float avgDistance = 0f;
    private bool isGrounded = false;

    public bool IsHoverGrounded => isGrounded;
    public bool IsCollidedGrounded { get; private set; } = false;
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

        Vector3 worldUp = smoothedNormal;
        avgDistance = traversal.AverageGroundHeight;

        float currentHeight = Mathf.Clamp(avgDistance, 0f, sensorRange);
        float heightError = hoverHeight - currentHeight;

        float springForce = liftStrength * heightError;
        float verticalVelocity = Vector3.Dot(rb.linearVelocity, worldUp);
        float dampingForce = -liftDamping * verticalVelocity;

        Vector3 totalLift = worldUp * (springForce + dampingForce);
        totalLift = Vector3.ClampMagnitude(totalLift, liftStrength * hoverHeight);
        rb.AddForce(totalLift, ForceMode.Force);

        ApplyLevelingTorque(worldUp);

        if (drawDebugRays)
            Debug.DrawRay(transform.position, worldUp * (hoverHeight + 0.2f), Color.green);
    }

    // ------------------------------------------------------------
    // Leveling & Torque
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

        // 🔧 Directional rotational damping: suppress pitch/roll only (preserves yaw)
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        localAngVel.x *= (selfLevelStrength * angularDampingMultiplier);  // pitch
        localAngVel.z *= (selfLevelStrength * angularDampingMultiplier);  // roll
        localAngVel.y = 0f; // leave yaw free
        Vector3 worldAngDamp = -transform.TransformDirection(localAngVel);
        rb.AddTorque(worldAngDamp, ForceMode.Acceleration);

#if UNITY_EDITOR
        if (drawDebugRays)
        {
            Vector3 origin = transform.position;
            Debug.DrawRay(origin, currentUp * 2f, Color.yellow);    // craft up
            Debug.DrawRay(origin, targetNormal * 2f, Color.cyan);   // ground normal
            Debug.DrawRay(origin, torqueAxis * 2f, Color.magenta);  // correction axis
        }
#endif
    }

    // ------------------------------------------------------------
    // Recovery / Unstick
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

    // ------------------------------------------------------------
    // External Helpers
    // ------------------------------------------------------------
    public void ForceUnground() => isGrounded = false;
}
