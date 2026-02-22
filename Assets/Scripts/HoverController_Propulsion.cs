using UnityEngine;

/// <summary>
/// HoverController_Propulsion v2.0
/// ---------------------------------
/// Responsibilities:
///   • Unified drive: grounded only — throttle + assist suppressed airborne unless boosting
///   • Yaw torque + torque-based yaw damping
///   • Soft top-speed cap via counter-force  (replaces linearVelocity write)
///   • Lateral drag and coast drag via forces (not velocity overwrite)
///   • Boost blend
///   • Extra gravity / air gravity
///
/// Physics contract: zero direct writes to rb.linearVelocity or rb.angularVelocity.
/// All motion is expressed as AddForce / AddTorque.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Foundation))]
public class HoverController_Propulsion : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 🚀 Drive
    // -------------------------------------------------------------------------
    [Header("🚀 Drive")]
    [Tooltip("Peak forward acceleration (m/s²) at full throttle, before boost.")]
    [SerializeField] private float maxForwardAccel = 25f;

    [Tooltip("Peak reverse acceleration (m/s²) at full reverse throttle.")]
    [SerializeField] private float maxReverseAccel = 15f;

    [Tooltip("Forward top speed (m/s) before boost.")]
    [SerializeField] private float topSpeed = 40f;

    // -------------------------------------------------------------------------
    // 🎯 Speed Assist
    // -------------------------------------------------------------------------
    [Header("🎯 Speed Assist")]
    [Tooltip("Proportional gain driving forward speed toward the throttle-scaled target. " +
             "Combined with raw throttle accel in a single force — no double-counting.")]
    [Range(0f, 100f)]
    [SerializeField] private float speedAssistStrength = 25f;

    [Tooltip("Maximum acceleration the assist can contribute (m/s²).")]
    [Range(0f, 200f)]
    [SerializeField] private float speedAssistMaxAccel = 60f;


    // -------------------------------------------------------------------------
    // 🔄 Turning
    // -------------------------------------------------------------------------
    [Header("🔄 Turning")]
    [Tooltip("Yaw acceleration (rad/s²) at full turn input, scaled by moment of inertia.")]
    [SerializeField] private float yawAccel = 8f;

    [Tooltip("Yaw damping strength. Counter-torque proportional to yaw rate.")]
    [Range(0f, 20f)]
    [SerializeField] private float yawDamping = 6f;

    [Tooltip("Scales yaw torque while airborne (0 = no air turning, 1 = full).")]
    [Range(0f, 1f)]
    [SerializeField] private float airTurnMultiplier = 0.5f;

    // -------------------------------------------------------------------------
    // ⚡ Boost
    // -------------------------------------------------------------------------
    [Header("⚡ Boost")]
    [Tooltip("Enable boost.")]
    [SerializeField] private bool enableBoost = true;

    [Tooltip("Forward acceleration multiplier while boosting.")]
    [Range(1f, 3f)]
    [SerializeField] private float boostAccelMultiplier = 1.75f;

    [Tooltip("Top speed multiplier while boosting.")]
    [Range(1f, 3f)]
    [SerializeField] private float boostSpeedMultiplier = 1.5f;

    [Tooltip("Time (seconds) to blend boost in and out.")]
    [Min(0.01f)]
    [SerializeField] private float boostBlendSeconds = 0.35f;

    private float boostLerp; // 0..1, managed by ApplyBoostBlend

    // -------------------------------------------------------------------------
    // 🔒 Soft Top-Speed Cap
    // -------------------------------------------------------------------------
    [Header("🔒 Soft Top-Speed Cap")]
    [Tooltip("Counter-force (m/s²) applied per m/s of excess horizontal speed. " +
             "Higher = tighter cap but may feel sticky on ramps. Recommended: 20–60.")]
    [Range(0f, 120f)]
    [SerializeField] private float softCapStrength = 40f;

    // -------------------------------------------------------------------------
    // 🧲 Drag
    // -------------------------------------------------------------------------
    [Header("🧲 Drag")]
    [Tooltip("Sideways-slip counter-force (m/s²). Controls how much the craft slides laterally.")]
    [Range(0f, 50f)]
    [SerializeField] private float lateralDamp = 10f;

    [Tooltip("Forward drag while coasting (no throttle, no speed assist). " +
             "Applied only when speed assist is idle so they don't fight each other.")]
    [Range(0f, 50f)]
    [SerializeField] private float coastDrag = 4f;

    // -------------------------------------------------------------------------
    // 🌎 Gravity
    // -------------------------------------------------------------------------
    [Header("🌎 Gravity")]
    [Tooltip("Multiplier added on top of Unity gravity (Acceleration mode). 0 = no extra.")]
    [Range(0f, 5f)]
    [SerializeField] private float extraGravityMultiplier = 0f;

    [Tooltip("Additional downward acceleration while airborne. Reduces floatiness.")]
    [Range(0f, 30f)]
    [SerializeField] private float extraAirGravity = 0f;

    // -------------------------------------------------------------------------
    // 🕹 Input
    // -------------------------------------------------------------------------
    [Header("🕹 Input")]
    [Tooltip("MonoBehaviour implementing IHoverInputProvider (player, AI, network...).")]
    [SerializeField] private MonoBehaviour inputProvider;

    // -------------------------------------------------------------------------
    // 🧭 Debug
    // -------------------------------------------------------------------------
    [Header("🧭 Debug")]
    [SerializeField] private bool drawDebug = false;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private IHoverInputProvider  input;
    private HoverController_Foundation foundation;
    private Rigidbody            rb;

    // -------------------------------------------------------------------------
    private void Awake()
    {
        rb         = GetComponent<Rigidbody>();
        foundation = GetComponent<HoverController_Foundation>();

        input = inputProvider as IHoverInputProvider;
        if (input == null)
        {
            Debug.LogError(
                $"[Propulsion] '{name}': inputProvider is null or does not implement " +
                $"IHoverInputProvider. Vehicle will not respond to input.",
                this
            );
            enabled = false;
        }
    }

    private void FixedUpdate()
    {
        ApplyBoostBlend();
        ApplyDrive();         // throttle accel + speed assist unified
        ApplyTurning();
        ApplyDrag();          // lateral + coast (coast gated against assist)
        ApplyExtraGravity();
        ApplySoftSpeedCap();  // counter-force replaces linearVelocity write
    }

    // -------------------------------------------------------------------------
    // Boost blend
    // -------------------------------------------------------------------------
    private void ApplyBoostBlend()
    {
        float target = (enableBoost && input.Boost) ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, boostBlendSeconds);
        boostLerp = Mathf.MoveTowards(boostLerp, target, step);
    }

    private float EffectiveTopSpeed()    => topSpeed    * Mathf.Lerp(1f, boostSpeedMultiplier, boostLerp);
    private float EffectiveForwardAccel() => maxForwardAccel * Mathf.Lerp(1f, boostAccelMultiplier, boostLerp);

    // -------------------------------------------------------------------------
    // Drive — unified throttle + assist in one force pass
    // -------------------------------------------------------------------------
    /// <summary>
    /// Airborne drive rules:
    ///   • Throttle accel and speed-assist servo are both suppressed when airborne.
    ///   • Boost is the only way to gain speed in the air — boostLerp > 0 re-enables
    ///     both the raw accel and the assist servo scaled by their boost multipliers.
    ///   • This preserves exit velocity as a projectile feel: whatever speed you had
    ///     when you left the ground is what you carry through the air.
    ///
    /// Grounded drive rules:
    ///   • Raw throttle provides the immediate arcade response floor.
    ///   • Speed-assist servo converges forward speed toward the target, clamped to
    ///     avoid fighting raw accel.
    /// </summary>
    private void ApplyDrive()
    {
        bool  grounded = foundation.IsHoverGrounded;
        bool  boosting = boostLerp > 0f;

        // Airborne with no boost — nothing to do. Exit velocity carries unchanged.
        if (!grounded && !boosting)
            return;

        float throttle = Mathf.Clamp(input.ThrottleInput, -1f, 1f);

        // Raw throttle force.
        // Airborne + boosting: boost multipliers are already baked into EffectiveForwardAccel().
        // Reverse is suppressed in air — no reason to decelerate hard mid-flight.
        float rawAccel = 0f;
        if (grounded)
        {
            rawAccel = throttle >= 0f
                ? throttle * EffectiveForwardAccel()
                : throttle * maxReverseAccel;
        }
        else
        {
            // Airborne boost: only forward, scaled by boost lerp to blend in/out smoothly.
            rawAccel = Mathf.Max(throttle, 0f) * EffectiveForwardAccel();
        }

        rb.AddForce(transform.forward * rawAccel, ForceMode.Acceleration);

        // Speed-assist servo — target is throttle * effective top speed.
        // Airborne: only runs when boosting, giving boost its air-speed identity.
        float targetSpeed = throttle * EffectiveTopSpeed();
        float currentFwd  = Vector3.Dot(rb.linearVelocity, transform.forward);
        float error       = targetSpeed - currentFwd;
        float assistAccel = Mathf.Clamp(error * speedAssistStrength, -speedAssistMaxAccel, speedAssistMaxAccel);

        // Suppress deceleration assist in air even during boost — let the craft
        // carry excess speed naturally rather than actively scrubbing it.
        if (!grounded && assistAccel < 0f)
            assistAccel = 0f;

        rb.AddForce(transform.forward * assistAccel, ForceMode.Acceleration);

        if (drawDebug)
            Debug.DrawRay(transform.position, transform.forward * (rawAccel + assistAccel), Color.yellow);
    }

    // -------------------------------------------------------------------------
    // Turning — yaw torque + counter-torque damping
    // -------------------------------------------------------------------------
    private void ApplyTurning()
    {
        float turn      = Mathf.Clamp(input.TurnInput, -1f, 1f);
        float turnScale = foundation.IsHoverGrounded ? 1f : airTurnMultiplier;

        // Scale by inertia so yaw accel stays consistent across different body masses.
        float inertiaY      = Mathf.Max(0.001f, rb.inertiaTensor.y);
        float desiredYawAcc = turn * yawAccel * turnScale;
        rb.AddRelativeTorque(Vector3.up * desiredYawAcc * inertiaY, ForceMode.Force);

        if (yawDamping <= 0f)
            return;

        float localYawRate  = transform.InverseTransformDirection(rb.angularVelocity).y;
        float dampingTorque = -localYawRate * yawDamping * inertiaY;
        rb.AddRelativeTorque(Vector3.up * dampingTorque, ForceMode.Force);
    }

    // -------------------------------------------------------------------------
    // Drag — lateral slip + coast forward drag
    // -------------------------------------------------------------------------
    private void ApplyDrag()
    {
        // Both lateral and coast drag are grounded-only.
        // Airborne: the craft maintains its exit velocity as a projectile.
        // Applying lateral drag in air would kill the natural arc feel.
        if (!foundation.IsHoverGrounded)
            return;

        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);

        // Lateral: reduces sideways sliding while grounded.
        if (lateralDamp > 0f)
            rb.AddForce(transform.right * (-localVel.x * lateralDamp), ForceMode.Acceleration);

        // Coast drag: only when throttle is near-zero so it doesn't fight the servo.
        bool assistIdle = Mathf.Abs(input.ThrottleInput) < 0.01f;
        if (coastDrag > 0f && assistIdle)
            rb.AddForce(transform.forward * (-localVel.z * coastDrag), ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // Extra gravity
    // -------------------------------------------------------------------------
    private void ApplyExtraGravity()
    {
        if (extraGravityMultiplier > 0f)
            rb.AddForce(Physics.gravity * extraGravityMultiplier, ForceMode.Acceleration);

        if (!foundation.IsHoverGrounded && extraAirGravity > 0f)
            rb.AddForce(Vector3.down * extraAirGravity, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // Soft top-speed cap — counter-force replaces linearVelocity write
    // -------------------------------------------------------------------------
    /// <summary>
    /// The previous version clamped rb.linearVelocity directly, which discards
    /// physics-solver momentum on the frame it's written — most visible as a
    /// "sticky wall" feel on collision frames.
    ///
    /// This version applies a counter-force proportional to the excess horizontal speed.
    /// The cap is "soft": impacts and ramps can briefly breach it, which is correct
    /// physics behaviour and feels better. softCapStrength controls how tightly it's
    /// enforced — tune higher for a stricter cap, lower for more natural overflow.
    /// </summary>
    private void ApplySoftSpeedCap()
    {
        if (softCapStrength <= 0f)
            return;

        float max       = EffectiveTopSpeed();
        Vector3 vel     = rb.linearVelocity;
        Vector3 horiz   = new Vector3(vel.x, 0f, vel.z);
        float   speed   = horiz.magnitude;

        if (speed <= max)
            return;

        float excess         = speed - max;
        Vector3 counterForce = -horiz.normalized * (excess * softCapStrength);
        rb.AddForce(counterForce, ForceMode.Acceleration);
    }
}
