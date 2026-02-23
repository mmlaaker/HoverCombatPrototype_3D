using UnityEngine;

/// <summary>
/// HoverController_Propulsion v3.0 (Drift Support)
/// -------------------------------------------------
/// Responsibilities:
///   • Unified drive: grounded only — throttle + assist suppressed airborne unless boosting
///   • Yaw torque + torque-based yaw damping
///   • Soft top-speed cap via counter-force
///   • Lateral drag via force (grounded only)
///   • Drift state: reduces lateral damp, boosts yaw, banks mesh root visually
///   • Boost blend
///   • Extra gravity / air gravity
///
/// Physics contract: zero direct writes to rb.linearVelocity or rb.angularVelocity.
/// All motion is expressed as AddForce / AddTorque.
///
/// Drift state modifies two physics values (lateralDamp, yawAccel) and one
/// visual transform (meshRoot local Z rotation). No new forces are introduced.
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
    [Tooltip("Sideways-slip counter-force (m/s²) during normal flight. " +
             "Controls how tightly the craft tracks its heading. " +
             "0 = fully slippery, 10 = sticky. Your testing found 0–2 feels best.")]
    [Range(0f, 50f)]
    [SerializeField] private float lateralDamp = 2f;

    // -------------------------------------------------------------------------
    // 🌀 Drift
    // -------------------------------------------------------------------------
    [Header("🌀 Drift")]
    [Tooltip("Lateral damp value while fully in drift state. " +
             "Lower than lateralDamp — the craft slides through the turn. " +
             "0 = fully free, your testing found this feels good.")]
    [Range(0f, 50f)]
    [SerializeField] private float driftLateralDamp = 0f;

    [Tooltip("Yaw acceleration multiplier while fully in drift state. " +
             "Higher than 1 — nose rotates faster than the momentum vector, " +
             "creating the shouldering angle. Recommended: 1.2–1.6.")]
    [Range(1f, 3f)]
    [SerializeField] private float driftYawMultiplier = 1.4f;

    [Tooltip("Minimum absolute turn input required to engage drift. " +
             "Prevents drift activating on gentle steering. Recommended: 0.3–0.5.")]
    [Range(0f, 1f)]
    [SerializeField] private float driftTurnThreshold = 0.4f;

    [Tooltip("Time (seconds) to blend drift in and out. " +
             "Faster = snappier entry/exit. Recommended: 0.1–0.25.")]
    [Min(0.01f)]
    [SerializeField] private float driftBlendSeconds = 0.15f;

    [Tooltip("Mesh parent object (HoverCar). Rotated on local Z for chassis bank visual. " +
             "Assign the HoverCar object in the inspector — not the root or individual meshes.")]
    [SerializeField] private Transform meshRoot;

    [Tooltip("Maximum chassis bank angle (degrees) at full drift. " +
             "Recommended: 15–25. Exaggerate slightly — readability matters at speed.")]
    [Range(0f, 45f)]
    [SerializeField] private float maxBankAngle = 20f;

    [Tooltip("Speed (lerp t per second) at which the chassis bank visually catches up. " +
             "Recommended: 6–10.")]
    [Range(1f, 20f)]
    [SerializeField] private float bankLerpSpeed = 8f;

    private float driftLerp; // 0..1, managed by ApplyDriftBlend

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
    private IHoverInputProvider      input;
    private HoverController_Foundation foundation;
    private Rigidbody                rb;

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

        if (meshRoot == null)
            Debug.LogWarning(
                $"[Propulsion] '{name}': meshRoot is not assigned. Chassis bank will not display. " +
                $"Assign the HoverCar object to the Mesh Root field in the Inspector.",
                this
            );
    }

    private void FixedUpdate()
    {
        ApplyBoostBlend();
        ApplyDriftBlend();
        ApplyDrive();
        ApplyTurning();
        ApplyDrag();
        ApplyExtraGravity();
        ApplySoftSpeedCap();
    }

    private void Update()
    {
        // Chassis bank is visual only — runs in Update for smooth interpolation
        // independent of the physics timestep.
        ApplyChassisBank();
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

    private float EffectiveTopSpeed()     => topSpeed        * Mathf.Lerp(1f, boostSpeedMultiplier,  boostLerp);
    private float EffectiveForwardAccel() => maxForwardAccel * Mathf.Lerp(1f, boostAccelMultiplier,  boostLerp);

    // -------------------------------------------------------------------------
    // Drift blend
    // -------------------------------------------------------------------------
    /// <summary>
    /// Drift engages when:
    ///   1. Drift button is held
    ///   2. Turn input exceeds driftTurnThreshold
    ///   3. Craft is grounded (no drift airborne — carry is already free in air)
    ///
    /// driftLerp drives all drift-state values: lateralDamp, yawAccel, chassis bank.
    /// Using the same MoveTowards pattern as boostLerp for consistency.
    /// </summary>
    private void ApplyDriftBlend()
    {
        bool driftCondition = input.Drift
                           && Mathf.Abs(input.TurnInput) >= driftTurnThreshold
                           && foundation.IsHoverGrounded;

        float target = driftCondition ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, driftBlendSeconds);
        driftLerp = Mathf.MoveTowards(driftLerp, target, step);
    }

    // Effective lateral damp lerps between normal and drift value.
    private float EffectiveLateralDamp() => Mathf.Lerp(lateralDamp, driftLateralDamp, driftLerp);

    // Effective yaw multiplier lerps between 1 and driftYawMultiplier.
    private float EffectiveYawMultiplier() => Mathf.Lerp(1f, driftYawMultiplier, driftLerp);

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
        bool grounded = foundation.IsHoverGrounded;
        bool boosting = boostLerp > 0f;

        // Airborne with no boost — nothing to do. Exit velocity carries unchanged.
        if (!grounded && !boosting)
            return;

        float throttle = Mathf.Clamp(input.ThrottleInput, -1f, 1f);

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
        float targetSpeed = throttle * EffectiveTopSpeed();
        float currentFwd  = Vector3.Dot(rb.linearVelocity, transform.forward);
        float error       = targetSpeed - currentFwd;
        float assistAccel = Mathf.Clamp(error * speedAssistStrength, -speedAssistMaxAccel, speedAssistMaxAccel);

        // Suppress deceleration assist in air even during boost.
        if (!grounded && assistAccel < 0f)
            assistAccel = 0f;

        rb.AddForce(transform.forward * assistAccel, ForceMode.Acceleration);

        if (drawDebug)
            Debug.DrawRay(transform.position, transform.forward * (rawAccel + assistAccel), Color.yellow);
    }

    // -------------------------------------------------------------------------
    // Turning — yaw torque + counter-torque damping
    // -------------------------------------------------------------------------
    /// <summary>
    /// Drift state increases yaw via EffectiveYawMultiplier(), causing the nose
    /// to rotate faster than the momentum vector — this is what creates the
    /// shouldering angle where the craft is traveling one direction but facing another.
    /// </summary>
    private void ApplyTurning()
    {
        float turn      = Mathf.Clamp(input.TurnInput, -1f, 1f);
        float turnScale = foundation.IsHoverGrounded ? 1f : airTurnMultiplier;

        float inertiaY      = Mathf.Max(0.001f, rb.inertiaTensor.y);
        float desiredYawAcc = turn * yawAccel * turnScale * EffectiveYawMultiplier();
        rb.AddRelativeTorque(Vector3.up * desiredYawAcc * inertiaY, ForceMode.Force);

        if (yawDamping <= 0f)
            return;

        float localYawRate  = transform.InverseTransformDirection(rb.angularVelocity).y;
        float dampingTorque = -localYawRate * yawDamping * inertiaY;
        rb.AddRelativeTorque(Vector3.up * dampingTorque, ForceMode.Force);
    }

    // -------------------------------------------------------------------------
    // Drag — lateral slip (grounded only)
    // -------------------------------------------------------------------------
    /// <summary>
    /// lateralDamp is lerped toward driftLateralDamp during drift state.
    /// At driftLateralDamp = 0, lateral velocity is completely unresisted —
    /// the craft slides freely through the turn, carrying its momentum arc.
    /// CoastDrag removed — zero was the correct value and dead code adds noise.
    /// </summary>
    private void ApplyDrag()
    {
        if (!foundation.IsHoverGrounded)
            return;

        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);

        float effectiveDamp = EffectiveLateralDamp();
        if (effectiveDamp > 0f)
            rb.AddForce(transform.right * (-localVel.x * effectiveDamp), ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // Chassis bank — visual only, runs in Update
    // -------------------------------------------------------------------------
    /// <summary>
    /// Rotates meshRoot on its local Z axis to bank the chassis into the drift.
    /// Bank direction is determined by turn input sign — left turn banks left, etc.
    /// Bank magnitude scales with driftLerp so it fades in/out with the drift state.
    ///
    /// This runs in Update (not FixedUpdate) so the visual is smooth at any frame rate.
    /// The physics body is unaffected — only the mesh parent rotates.
    /// </summary>
    private void ApplyChassisBank()
    {
        if (meshRoot == null)
            return;

        // Bank toward the turn direction, scaled by how deep into drift state we are.
        // Negative Z rotation = right bank (right turn), positive = left bank (left turn).
        float turnSign    = Mathf.Sign(input.TurnInput);
        float targetAngle = -turnSign * maxBankAngle * driftLerp;

        Quaternion targetRot = Quaternion.Euler(0f, 0f, targetAngle);
        meshRoot.localRotation = Quaternion.Lerp(
            meshRoot.localRotation,
            targetRot,
            bankLerpSpeed * Time.deltaTime
        );
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
    /// Applies a counter-force proportional to excess horizontal speed.
    /// Soft cap allows brief breaches on impacts and ramps — correct and feels better
    /// than a hard clamp which creates a sticky wall feel on collision frames.
    /// </summary>
    private void ApplySoftSpeedCap()
    {
        if (softCapStrength <= 0f)
            return;

        float   max   = EffectiveTopSpeed();
        Vector3 vel   = rb.linearVelocity;
        Vector3 horiz = new Vector3(vel.x, 0f, vel.z);
        float   speed = horiz.magnitude;

        if (speed <= max)
            return;

        float   excess       = speed - max;
        Vector3 counterForce = -horiz.normalized * (excess * softCapStrength);
        rb.AddForce(counterForce, ForceMode.Acceleration);
    }
}
