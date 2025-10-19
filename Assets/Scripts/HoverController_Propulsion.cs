using UnityEngine;

/// <summary>
/// HoverController_Propulsion v5.9 (Assist Scaling Edition)
/// --------------------------------------------------------
/// • Hybrid assist is now gated by engine power  
/// • Optional scaling mode blends assist intensity with engine output  
/// • Maintains all designer-friendly naming and grouping
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Foundation))]
public class HoverController_Propulsion : MonoBehaviour
{
    // ------------------------------------------------------------
    // 🚀 Engine Performance
    // ------------------------------------------------------------
    [Header("🚀 Engine Performance")]
    [Tooltip("Primary engine power (Newtons). Controls how fast the craft accelerates.")]
    [SerializeField] private float enginePower = 20000f;

    [Tooltip("Turning agility (torque applied to yaw rotation).")]
    [SerializeField] private float turnAgility = 8000f;

    [Tooltip("Maximum forward speed (m/s).")]
    [SerializeField] private float topSpeed = 40f;

    [Tooltip("Speed boost multiplier for power-ups or temporary overdrive.")]
    [Range(1f, 3f)] [SerializeField] private float boostMultiplier = 1.5f;

    // ------------------------------------------------------------
    // 🕹 Handling Feel
    // ------------------------------------------------------------
    [Header("🕹 Handling Feel")]
    [Tooltip("Relative mass scaling used for responsiveness calculations.")]
    [Range(0.1f, 5f)] [SerializeField] private float vehicleWeight = 1f;

    [Tooltip("How quickly acceleration responds to input changes.")]
    [Range(0.1f, 10f)] [SerializeField] private float accelerationResponsiveness = 2.5f;

    [Tooltip("How quickly steering responds (also affects yaw damping).")]
    [Range(0.1f, 10f)] [SerializeField] private float steeringResponsiveness = 2f;

    [Tooltip("How much velocity is preserved each frame (1 = full momentum).")]
    [Range(0f, 1f)] [SerializeField] private float momentumRetention = 1f;

    // ------------------------------------------------------------
    // 🧲 Friction & Resistance
    // ------------------------------------------------------------
    [Header("🧲 Friction & Resistance")]
    [Tooltip("Surface grip. Higher = more traction, less side-sliding.")]
    [Range(0f, 10f)] [SerializeField] private float surfaceGrip = 3f;

    [Tooltip("Drag applied to forward motion through air.")]
    [Range(0f, 5f)] [SerializeField] private float airDragMultiplier = 0.1f;

    // ------------------------------------------------------------
    // 🌎 Gravity Behavior
    // ------------------------------------------------------------
    [Header("🌎 Gravity Behavior")]
    [Tooltip("Base gravity multiplier for hovercraft weight.")]
    [Range(0.1f, 5f)] [SerializeField] private float gravityStrength = 2.5f;

    [Tooltip("Extra gravity applied when airborne (helps prevent floatiness).")]
    [Range(0f, 2f)] [SerializeField] private float extraAirGravity = 0.5f;

    // ------------------------------------------------------------
    // 🤖 Assist & Input
    // ------------------------------------------------------------
    [Header("🤖 Assist & Input")]
    [Tooltip("Enable hybrid assist (blends physics with direct velocity control).")]
    [SerializeField] private bool hybridAssist = true;

    [Tooltip("When enabled, assist strength scales proportionally with Engine Power.")]
    [SerializeField] private bool scaleAssistWithEnginePower = true;

    [Tooltip("Input provider (PlayerHoverInput, AIHoverInput, etc.) implementing IHoverInputProvider.")]
    [SerializeField] private MonoBehaviour inputProvider;

    // ------------------------------------------------------------
    // 🧭 Debug & Diagnostics
    // ------------------------------------------------------------
    [Header("🧭 Debug & Diagnostics")]
    [Tooltip("Draw debug rays for velocity and thrust direction.")]
    [SerializeField] private bool drawDebugVectors = true;

    [Tooltip("Show live input values in the console.")]
    [SerializeField] private bool debugInput = false;

    // ------------------------------------------------------------
    // Internal references
    // ------------------------------------------------------------
    private IHoverInputProvider input;
    private HoverController_Foundation foundation;
    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        foundation = GetComponent<HoverController_Foundation>();

        if (inputProvider != null)
            input = inputProvider as IHoverInputProvider;

        if (input == null)
            Debug.LogWarning("[Propulsion] No valid input provider found. Vehicle will not respond to input.");
    }

    void FixedUpdate()
    {
        if (input == null) return;

        ApplyGravity();
        ApplyMovement();
        ApplyTurning();
        ApplyFriction();
    }

    // ------------------------------------------------------------
    // 🔽 Gravity
    // ------------------------------------------------------------
    private void ApplyGravity()
    {
        Vector3 customGravity = Physics.gravity * gravityStrength;
        rb.AddForce(customGravity, ForceMode.Acceleration);

        if (!foundation.IsHoverGrounded && extraAirGravity > 0f)
            rb.AddForce(Physics.gravity * extraAirGravity, ForceMode.Acceleration);
    }

    // ------------------------------------------------------------
    // 🚗 Movement
    // ------------------------------------------------------------
    private void ApplyMovement()
    {
        float throttleInput = input.ThrottleInput;
        Vector3 forwardDir = transform.forward;

        // Project velocity on horizontal plane
        Vector3 horizontalVel = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);

        // 1️⃣ Apply physics-based acceleration (true thrust)
        if (enginePower > 0f)
        {
            float accelPerSecond = enginePower / rb.mass;
            float desiredAccel = throttleInput * accelPerSecond;
            rb.AddForce(forwardDir * desiredAccel, ForceMode.Acceleration);
        }

        // 2️⃣ Hybrid assist (now gated by engine power)
        if (hybridAssist && enginePower > 0f)
        {
            float assistFactor = scaleAssistWithEnginePower
                ? Mathf.Clamp01(enginePower / 5000f)   // scaled
                : 1f;                                  // binary

            Vector3 targetVel = forwardDir * (throttleInput * topSpeed * boostMultiplier);
            Vector3 preserved = rb.linearVelocity * momentumRetention;
            preserved.y = rb.linearVelocity.y;

            rb.linearVelocity = Vector3.Lerp(
                preserved,
                targetVel,
                assistFactor * Time.fixedDeltaTime * accelerationResponsiveness / vehicleWeight
            );
        }

        // 3️⃣ Clamp top speed
        Vector3 horizVel = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
        if (horizVel.magnitude > topSpeed)
        {
            horizVel = horizVel.normalized * topSpeed;
            rb.linearVelocity = new Vector3(horizVel.x, rb.linearVelocity.y, horizVel.z);
        }

        if (debugInput)
            Debug.Log($"[Propulsion] Throttle={throttleInput:F2}  Speed={horizVel.magnitude:F1}");

        if (drawDebugVectors)
            Debug.DrawRay(transform.position, rb.linearVelocity, Color.cyan);
    }

    // ------------------------------------------------------------
    // 🔄 Turning (with yaw damping)
    // ------------------------------------------------------------
    private void ApplyTurning()
    {
        float turnInput = input.TurnInput;
        if (Mathf.Abs(turnInput) < 0.001f)
            return;

        float torqueScale = foundation.IsHoverGrounded ? 1f : 0.5f;

        rb.AddRelativeTorque(
            Vector3.up * turnInput * turnAgility * torqueScale * Time.fixedDeltaTime,
            ForceMode.Acceleration
        );

        // 🔧 Yaw damping to stop spin buildup
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        localAngVel.y *= 1f / (1f + steeringResponsiveness * Time.fixedDeltaTime);
        rb.angularVelocity = transform.TransformDirection(localAngVel);
    }

    // ------------------------------------------------------------
    // 🧲 Friction & Drag
    // ------------------------------------------------------------
    private void ApplyFriction()
    {
        if (!foundation.IsHoverGrounded)
            return;

        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        localVel.x *= 1f / (1f + surfaceGrip * Time.fixedDeltaTime);
        localVel.z *= 1f / (1f + airDragMultiplier * Time.fixedDeltaTime);
        rb.linearVelocity = transform.TransformDirection(localVel);
    }
}
