using UnityEngine;

/// <summary>
/// HoverController_Propulsion v6.1 (Adaptive Yaw Damping Edition)
/// ---------------------------------------------------------------
/// • Adds adaptive yaw damping based on speed & boost intensity
/// • Retains smooth boost blending, hybrid assist, and weighted control
/// • Polished inspector organization and designer tooltips
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

    // ------------------------------------------------------------
    // ⚡ Boost Settings
    // ------------------------------------------------------------
    [Header("⚡ Boost Settings")]
    [Tooltip("Enable temporary thrust and top-speed increase when Boost input is active.")]
    [SerializeField] private bool enableBoost = true;

    [Tooltip("How much to multiply Engine Power while boosting.")]
    [Range(1f, 3f)] [SerializeField] private float boostPowerMultiplier = 2f;

    [Tooltip("How much to multiply Top Speed while boosting.")]
    [Range(1f, 3f)] [SerializeField] private float boostSpeedMultiplier = 1.5f;

    [Tooltip("How quickly boost power fades in/out (seconds).")]
    [Range(0.1f, 2f)] [SerializeField] private float boostBlendSpeed = 0.5f;

    private float currentBoostLerp = 1f;

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

    [Tooltip("Show live input values and damping logs in the console.")]
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

        // --- Boost logic ---
        bool boosting = enableBoost && input.Boost;
        float targetBoost = boosting ? boostPowerMultiplier : 1f;
        currentBoostLerp = Mathf.MoveTowards(currentBoostLerp, targetBoost, Time.fixedDeltaTime / boostBlendSpeed);

        float effectiveEnginePower = enginePower * currentBoostLerp;
        float effectiveTopSpeed = topSpeed * (boosting ? boostSpeedMultiplier : 1f);

        // --- Physics thrust ---
        if (effectiveEnginePower > 0f)
        {
            float accelPerSecond = effectiveEnginePower / rb.mass;
            float desiredAccel = throttleInput * accelPerSecond;
            rb.AddForce(forwardDir * desiredAccel, ForceMode.Acceleration);
        }

        // --- Hybrid assist ---
        if (hybridAssist && effectiveEnginePower > 0f)
        {
            float assistFactor = scaleAssistWithEnginePower
                ? Mathf.Clamp01(effectiveEnginePower / 5000f)
                : 1f;

            Vector3 targetVel = forwardDir * (throttleInput * effectiveTopSpeed);
            Vector3 preserved = rb.linearVelocity * momentumRetention;
            preserved.y = rb.linearVelocity.y;

            rb.linearVelocity = Vector3.Lerp(
                preserved,
                targetVel,
                assistFactor * Time.fixedDeltaTime * accelerationResponsiveness / vehicleWeight
            );
        }

        // --- Clamp top speed ---
        Vector3 horizVel = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
        if (horizVel.magnitude > effectiveTopSpeed)
        {
            horizVel = horizVel.normalized * effectiveTopSpeed;
            rb.linearVelocity = new Vector3(horizVel.x, rb.linearVelocity.y, horizVel.z);
        }

        if (debugInput)
            Debug.Log($"[Propulsion] Boost={currentBoostLerp:F2} Power={effectiveEnginePower:F0} TopSpeed={effectiveTopSpeed:F1} Throttle={throttleInput:F2}");

        if (drawDebugVectors)
            Debug.DrawRay(transform.position, rb.linearVelocity, boosting ? Color.yellow : Color.cyan);
    }

    // ------------------------------------------------------------
    // 🔄 Turning (with adaptive yaw damping)
    // ------------------------------------------------------------
    private void ApplyTurning()
    {
        float turnInput = input.TurnInput;

        // --- Torque Application ---
        if (Mathf.Abs(turnInput) > 0.001f)
        {
            float torqueScale = foundation.IsHoverGrounded ? 1f : 0.5f;
            rb.AddRelativeTorque(Vector3.up * turnInput * turnAgility * torqueScale, ForceMode.Acceleration);
        }

        // --- Adaptive Yaw Damping ---
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);

        float forwardSpeed = rb.linearVelocity.magnitude;
        float speedFactor = Mathf.InverseLerp(0f, topSpeed * 1.5f, forwardSpeed);
        float boostFactor = Mathf.Lerp(1f, 1.5f, currentBoostLerp - 1f);

        // stronger when hybrid assist is on, ensuring responsiveness
        float assistMultiplier = hybridAssist ? 1.5f : 1f;
        float adaptiveDamp = steeringResponsiveness * assistMultiplier * boostFactor * (0.5f + speedFactor);

        // exponential damping
        localAngVel.y = Mathf.Lerp(localAngVel.y, 0f, adaptiveDamp * Time.fixedDeltaTime);
        rb.angularVelocity = transform.TransformDirection(localAngVel);

        if (debugInput)
            Debug.Log($"[Turn Damp] yawVel={localAngVel.y:F3}, damp={adaptiveDamp:F2}, assist={assistMultiplier:F1}");
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
