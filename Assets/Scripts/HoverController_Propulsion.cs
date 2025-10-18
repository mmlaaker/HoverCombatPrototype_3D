using UnityEngine;

/// <summary>
/// HoverController_Propulsion v5.4 (Vertical Preservation Edition)
/// ----------------------------------------------------------------
/// Hybrid-Assist propulsion controller for hover vehicles.
/// • Prevents vertical damping from momentumRetention (falls naturally)
/// • Keeps all designer-friendly groupings and tooltips
/// • Includes grounded-aware friction with air drag control
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Foundation))]
[RequireComponent(typeof(HoverController_Traversal))]
public class HoverController_Propulsion : MonoBehaviour
{
    // -----------------------------
    //  SECTION: Engine Performance
    // -----------------------------
    [Header("⚙️ Engine Performance")]
    [Tooltip("How strong the vehicle’s forward drive is. Higher = faster acceleration.")]
    public float enginePower = 20000f;

    [Tooltip("How forcefully the craft can rotate (yaw). Higher = sharper steering response.")]
    public float turnAgility = 8000f;

    [Tooltip("How much faster the craft goes while boosting (Spacebar by default).")]
    public float boostMultiplier = 1.5f;

    [Tooltip("Maximum target speed in meters per second before the assist limits acceleration.")]
    public float topSpeed = 40f;

    // -----------------------------
    //  SECTION: Handling Feel
    // -----------------------------
    [Header("🎮 Handling Feel")]
    [Tooltip("Vehicle 'weight class'. Lower = nimble and floaty. Higher = heavy and planted.")]
    [Range(0.5f, 2f)] public float vehicleWeight = 1f;

    [Tooltip("How quickly the craft aligns to player throttle input. Higher = snappier control.")]
    [Range(0f, 10f)] public float accelerationResponsiveness = 2.5f;

    [Tooltip("How quickly the craft aligns to steering input. Higher = tighter turning.")]
    [Range(0f, 10f)] public float steeringResponsiveness = 2f;

    [Tooltip("How much momentum is preserved each tick. 0 = snappy arcade, 1 = floaty drift. Affects horizontal motion only.")]
    [Range(0f, 1f)] public float momentumRetention = 0.85f;

    // -----------------------------
    //  SECTION: Friction & Resistance
    // -----------------------------
    [Header("🧲 Friction & Resistance")]
    [Tooltip("General resistance applied to motion while grounded. Higher = stickier, slower craft.")]
    [Range(0f, 10f)] public float surfaceGrip = 3f;

    [Tooltip("How much of the surfaceGrip applies while airborne. 0 = none, 1 = full drag.")]
    [Range(0f, 1f)] public float airDragMultiplier = 0.1f;

    // -----------------------------
    //  SECTION: Gravity Behavior
    // -----------------------------
    [Header("🌍 Gravity Behavior")]
    [Tooltip("Base gravity multiplier (1 = default Unity gravity).")]
    [Range(0.5f, 5f)] public float gravityStrength = 2.5f;

    [Tooltip("Extra downward pull when airborne. 0 = none, 1 = +100%.")]
    [Range(0f, 2f)] public float extraAirGravity = 0.5f;

    // -----------------------------
    //  SECTION: Input & Debug
    // -----------------------------
    [Header("🎛 Input & Debug")]
    [Tooltip("Optional manual assignment for an input provider. Leave empty for autodetect.")]
    public MonoBehaviour inputProvider;

    [Tooltip("Draws visual gizmos and ground normals for debugging.")]
    public bool drawDebugVectors = true;

    [Tooltip("Logs live throttle/turn/boost values for testing.")]
    public bool debugInput = false;

    // --------------------------------------------------
    // Runtime references
    private Rigidbody rb;
    private HoverController_Foundation foundation;
    private HoverController_Traversal traversal;
    private IHoverInputProvider input;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        foundation = GetComponent<HoverController_Foundation>();
        traversal = GetComponent<HoverController_Traversal>();

        // Manual + auto hybrid detection
        if (inputProvider is IHoverInputProvider manual)
        {
            input = manual;
            Debug.Log($"[Propulsion] Input provider manually assigned: {manual.GetType().Name}");
        }
        else
        {
            input = GetComponent<IHoverInputProvider>();
            if (input != null)
                Debug.Log($"[Propulsion] Input provider autodetected: {input.GetType().Name}");
            else
                Debug.LogWarning("[Propulsion] No input provider found. Assign one implementing IHoverInputProvider.");
        }
    }

    void FixedUpdate()
    {
        bool grounded = foundation.IsHoverGrounded || foundation.IsCollidedGrounded;
        ApplyGravity(grounded);

        if (input == null)
            return;

        if (debugInput)
            Debug.Log($"[Input] Throttle={input.Throttle:F2}, Turn={input.Turn:F2}, Boost={input.Boost}");

        ApplyMovement(input.Throttle, input.Turn, input.Boost, grounded);
        ApplyFriction(grounded);
    }

    // --------------------------------------------------
    // Physics Core
    // --------------------------------------------------

    private void ApplyGravity(bool grounded)
    {
        // baseline gravity correction (adds only difference from default)
        rb.AddForce(Physics.gravity * rb.mass * (gravityStrength - 1f), ForceMode.Force);

        // extra gravity when airborne for more grounded feel
        if (!grounded)
        {
            float extraMult = Mathf.Lerp(0.5f, 1.2f, vehicleWeight - 0.5f);
            float delta = (gravityStrength * (1f + extraAirGravity * extraMult)) - gravityStrength;
            rb.AddForce(Physics.gravity * rb.mass * delta, ForceMode.Acceleration);
        }
    }

    private void ApplyMovement(float throttle, float turn, bool boost, bool grounded)
    {
        float boostMult = boost ? boostMultiplier : 1f;

        // Shape input for smoother low-end response
        float curvedThrottle = Mathf.Sign(throttle) * Mathf.Pow(Mathf.Abs(throttle), 1.2f);
        float curvedTurn = Mathf.Sign(turn) * Mathf.Pow(Mathf.Abs(turn), 1.1f);

        // Forward direction aligned to slope
        Vector3 forwardDir = Vector3.ProjectOnPlane(transform.forward, traversal.GroundNormal).normalized;

        // Desired target velocity
        Vector3 desiredVel = forwardDir * (curvedThrottle * topSpeed * boostMult);

        // Momentum retention for horizontal components only
        Vector3 preserved = rb.linearVelocity * momentumRetention;
        preserved.y = rb.linearVelocity.y; // ✅ Preserve vertical speed for natural gravity/falls

        desiredVel.y = preserved.y;
        rb.linearVelocity = Vector3.Lerp(preserved, desiredVel,
            Time.fixedDeltaTime * accelerationResponsiveness / vehicleWeight);

        // Apply yaw control (rotation)
        float targetYawRate = curvedTurn * turnAgility * Mathf.Deg2Rad * 0.001f;
        Vector3 angVel = rb.angularVelocity;
        angVel.y = Mathf.Lerp(angVel.y, targetYawRate,
            Time.fixedDeltaTime * steeringResponsiveness / vehicleWeight);
        rb.angularVelocity = angVel;
    }

    private void ApplyFriction(bool grounded)
    {
        float dragMult = grounded ? 1f : airDragMultiplier;

        // Linear damping
        rb.AddForce(-rb.linearVelocity * surfaceGrip * dragMult, ForceMode.Force);

        // Rotational damping
        rb.AddTorque(-rb.angularVelocity * surfaceGrip * 0.5f * dragMult, ForceMode.Force);

        // Lateral stabilization only when grounded
        if (grounded)
        {
            Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
            localVel.x = Mathf.Lerp(localVel.x, 0f, Time.fixedDeltaTime * surfaceGrip);
            rb.linearVelocity = transform.TransformDirection(localVel);
        }
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (!drawDebugVectors || traversal == null) return;
        Vector3 origin = transform.position;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(origin, origin + transform.forward * 2f);
        Gizmos.DrawSphere(origin + transform.forward * 2f, 0.05f);
        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(origin, traversal.GroundNormal * 2f);
    }
#endif
}
