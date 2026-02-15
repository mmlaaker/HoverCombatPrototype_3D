using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Foundation))]
public class HoverController_Propulsion : MonoBehaviour
{
    // ------------------------------------------------------------
    // 🚀 Drive
    // ------------------------------------------------------------
    [Header("🚀 Drive")]
    [Tooltip("Maximum forward acceleration (m/s²) applied at full throttle.")]
    [SerializeField] private float maxForwardAccel = 25f;

    [Tooltip("Maximum reverse acceleration (m/s²) applied at full reverse throttle.")]
    [SerializeField] private float maxReverseAccel = 15f;

    [Tooltip("Maximum forward speed (m/s).")]
    [SerializeField] private float topSpeed = 40f;

    // ------------------------------------------------------------
    // 🔄 Turning
    // ------------------------------------------------------------
    [Header("🔄 Turning")]
    [Tooltip("Yaw acceleration (rad/s²) applied at full turn input.")]
    [SerializeField] private float yawAccel = 8f;

    [Tooltip("Yaw damping strength (higher = more resistant to spinning). Uses torque, not angular-velocity overwrite.")]
    [Range(0f, 20f)] [SerializeField] private float yawDamping = 6f;

    [Tooltip("Air control multiplier for yaw torque (0–1).")]
    [Range(0f, 1f)] [SerializeField] private float airTurnMultiplier = 0.5f;

    // ------------------------------------------------------------
    // ⚡ Boost
    // ------------------------------------------------------------
    [Header("⚡ Boost")]
    [Tooltip("Enable temporary acceleration and top-speed increase while Boost is active.")]
    [SerializeField] private bool enableBoost = true;

    [Tooltip("Multiplier on forward acceleration while boosting.")]
    [Range(1f, 3f)] [SerializeField] private float boostAccelMultiplier = 1.75f;

    [Tooltip("Multiplier on top speed while boosting.")]
    [Range(1f, 3f)] [SerializeField] private float boostSpeedMultiplier = 1.5f;

    [Tooltip("Seconds to blend boost strength in/out.")]
    [Min(0.01f)] [SerializeField] private float boostBlendSeconds = 0.35f;

    private float boostLerp = 0f; // 0..1

    // ------------------------------------------------------------
    // 🎯 Speed Assist (Physical Servo)
    // ------------------------------------------------------------
    [Header("🎯 Speed Assist (Physical Servo)")]
    [Tooltip("Applies additional acceleration to converge toward the target speed without overwriting velocity.")]
    [SerializeField] private bool enableSpeedAssist = true;

    [Tooltip("How strongly we chase the target speed (m/s²). Acts like a proportional controller.")]
    [Range(0f, 100f)] [SerializeField] private float speedAssistStrength = 25f;

    [Tooltip("Limits how much the assist can accelerate (m/s²). Prevents 'teleporty' feel.")]
    [Range(0f, 200f)] [SerializeField] private float speedAssistMaxAccel = 60f;

    [Tooltip("If true, speed assist is reduced in air to preserve jump/impact dynamics.")]
    [SerializeField] private bool reduceAssistInAir = true;

    [Tooltip("Air multiplier for speed assist.")]
    [Range(0f, 1f)] [SerializeField] private float airAssistMultiplier = 0.35f;

    // ------------------------------------------------------------
    // 🧲 Simple Drag (Optional)
    // ------------------------------------------------------------
    [Header("🧲 Simple Drag (Optional)")]
    [Tooltip("Sideways slip damping (m/s²). Uses forces, not velocity overwrite.")]
    [Range(0f, 50f)] [SerializeField] private float lateralDamp = 10f;

    [Tooltip("Forward drag when no throttle (m/s²). Helps settle quickly without fighting collisions too hard.")]
    [Range(0f, 50f)] [SerializeField] private float coastDrag = 4f;

    // ------------------------------------------------------------
    // 🌎 Gravity
    // ------------------------------------------------------------
    [Header("🌎 Gravity")]
    [Tooltip("Additional gravity multiplier applied via AddForce(Acceleration). Unity gravity can remain enabled.")]
    [Range(0f, 5f)] [SerializeField] private float extraGravityMultiplier = 0f;

    [Tooltip("Extra gravity when airborne (Acceleration). Helps prevent floatiness.")]
    [Range(0f, 30f)] [SerializeField] private float extraAirGravity = 0f;

    // ------------------------------------------------------------
    // 🕹 Input
    // ------------------------------------------------------------
    [Header("🕹 Input")]
    [Tooltip("Input provider implementing IHoverInputProvider (PlayerHoverInput, AI, etc.).")]
    [SerializeField] private MonoBehaviour inputProvider;

    // ------------------------------------------------------------
    // Debug
    // ------------------------------------------------------------
    [Header("🧭 Debug")]
    [SerializeField] private bool drawDebug = false;

    private IHoverInputProvider input;
    private HoverController_Foundation foundation;
    private Rigidbody rb;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        foundation = GetComponent<HoverController_Foundation>();

        input = inputProvider as IHoverInputProvider;
        if (input == null)
            Debug.LogWarning("[Propulsion] No valid input provider found. Vehicle will not respond.");
    }

    private void FixedUpdate()
    {
        if (input == null) return;

        ApplyBoostBlend();
        ApplyDrive();
        ApplyTurning();
        ApplyAssistDrag();
        ApplyExtraGravity();
        ClampTopSpeed();
    }

    private void ApplyBoostBlend()
    {
        bool boosting = enableBoost && input.Boost;
        float target = boosting ? 1f : 0f;
        float step = Time.fixedDeltaTime / Mathf.Max(0.01f, boostBlendSeconds);
        boostLerp = Mathf.MoveTowards(boostLerp, target, step);
    }

    private float EffectiveTopSpeed()
    {
        return topSpeed * Mathf.Lerp(1f, boostSpeedMultiplier, boostLerp);
    }

    private float EffectiveForwardAccel()
    {
        return maxForwardAccel * Mathf.Lerp(1f, boostAccelMultiplier, boostLerp);
    }

    // ------------------------------------------------------------
    // Drive: pure acceleration forces (mass-independent tuning)
    // ------------------------------------------------------------
    private void ApplyDrive()
    {
        float throttle = Mathf.Clamp(input.ThrottleInput, -1f, 1f);

        float accel =
            throttle >= 0f
                ? throttle * EffectiveForwardAccel()
                : throttle * maxReverseAccel; // negative

        rb.AddForce(transform.forward * accel, ForceMode.Acceleration);

        // Speed assist (physical servo toward target velocity)
        if (enableSpeedAssist)
        {
            float targetSpeed = throttle * EffectiveTopSpeed();

            // current forward speed component
            float currentForward = Vector3.Dot(rb.linearVelocity, transform.forward);

            float error = targetSpeed - currentForward; // m/s
            float assistAccel = Mathf.Clamp(error * speedAssistStrength, -speedAssistMaxAccel, speedAssistMaxAccel);

            if (!foundation.IsHoverGrounded && reduceAssistInAir)
                assistAccel *= airAssistMultiplier;

            rb.AddForce(transform.forward * assistAccel, ForceMode.Acceleration);

            if (drawDebug)
            {
                Debug.DrawRay(transform.position, transform.forward * assistAccel, Color.yellow);
            }
        }
    }

    // ------------------------------------------------------------
    // Turning: yaw torque via acceleration-like control, plus torque damping (no angVel overwrite)
    // ------------------------------------------------------------
    private void ApplyTurning()
    {
        float turn = Mathf.Clamp(input.TurnInput, -1f, 1f);
        float turnScale = foundation.IsHoverGrounded ? 1f : airTurnMultiplier;

        // Apply yaw "angular acceleration" by applying torque proportional to desired yaw accel.
        // Unity torque units are N·m, but we're using ForceMode.Acceleration-like tuning via InertiaTensor approximation.
        // Practical arcade approach: torque = desiredYawAccel * rb.inertiaTensor.y (scaled).
        float inertiaY = rb.inertiaTensorRotation * Vector3.up == Vector3.up ? rb.inertiaTensor.y : rb.inertiaTensor.y;
        float desiredYawAccel = turn * yawAccel * turnScale;

        // Using ForceMode.Acceleration for torque is not available; use ForceMode.Force and scale by inertia.
        rb.AddRelativeTorque(Vector3.up * desiredYawAccel * Mathf.Max(0.001f, inertiaY), ForceMode.Force);

        // Yaw damping torque opposes current yaw angular velocity.
        if (yawDamping > 0f)
        {
            Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
            float dampingTorque = -localAngVel.y * yawDamping * Mathf.Max(0.001f, inertiaY);
            rb.AddRelativeTorque(Vector3.up * dampingTorque, ForceMode.Force);
        }
    }

    // ------------------------------------------------------------
    // Assist Drag: lateral + coasting drag using forces (not velocity overwrite)
    // ------------------------------------------------------------
    private void ApplyAssistDrag()
    {
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);

        // Lateral damping
        if (lateralDamp > 0f)
        {
            float lateralAccel = -localVel.x * lateralDamp;
            rb.AddForce(transform.right * lateralAccel, ForceMode.Acceleration);
        }

        // Forward coasting drag when no throttle
        if (coastDrag > 0f && Mathf.Abs(input.ThrottleInput) < 0.01f)
        {
            float forwardSpeed = localVel.z;
            float dragAccel = -forwardSpeed * coastDrag;
            rb.AddForce(transform.forward * dragAccel, ForceMode.Acceleration);
        }
    }

    private void ApplyExtraGravity()
    {
        if (extraGravityMultiplier > 0f)
            rb.AddForce(Physics.gravity * extraGravityMultiplier, ForceMode.Acceleration);

        if (!foundation.IsHoverGrounded && extraAirGravity > 0f)
            rb.AddForce(Vector3.down * extraAirGravity, ForceMode.Acceleration);
    }

    private void ClampTopSpeed()
    {
        float max = EffectiveTopSpeed();
        Vector3 v = rb.linearVelocity;
        Vector3 horiz = new Vector3(v.x, 0f, v.z);

        if (horiz.sqrMagnitude > max * max)
        {
            horiz = horiz.normalized * max;
            rb.linearVelocity = new Vector3(horiz.x, v.y, horiz.z);
        }
    }
}
