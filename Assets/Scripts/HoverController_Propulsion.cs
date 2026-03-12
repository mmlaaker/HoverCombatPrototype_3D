using UnityEngine;

/// <summary>
/// HoverController_Propulsion v4.4 (Cleanup)
/// -------------------------------------------------
/// Responsibilities:
///   • Unified drive: grounded only — throttle + assist suppressed airborne unless boosting
///   • Yaw torque + torque-based yaw damping
///   • Soft top-speed cap via counter-force (grounded only)
///   • Lateral drag via force (grounded only)
///   • Drift state: reduces lateral damp, boosts yaw, banks mesh root visually
///   • Boost blend — energy-gated via HoverController_Energy.TryConsume
///   • Jump — charge-based impulse, one air jump token, energy-gated
///
/// Physics contract: zero direct writes to rb.linearVelocity or rb.angularVelocity.
/// All motion is expressed as AddForce / AddTorque.
/// Exception: jump fires rb.AddForce(VelocityChange) — intentional, documented at call site.
///
/// Drift state modifies two physics values (lateralDamp, yawAccel) and one
/// visual transform (meshRoot local Z rotation). No new forces are introduced.
///
/// v4.4 changes (no behavior change):
///   • IsHoverGrounded cached once per FixedUpdate as a local and passed to all methods.
///     Previously read 5 separate times across one FixedUpdate tick.
///   • EffectiveTopSpeed() and EffectiveForwardAccel() cached as locals in FixedUpdate
///     and passed to ApplyDrive/ApplySoftSpeedCap. Previously recomputed on each call.
///   • EffectiveLateralDamp() and EffectiveYawMultiplier() inlined at their single
///     call sites in ApplyDrag and ApplyTurning. Both were one-line lerps called
///     in exactly one place — named methods added noise without readability benefit.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Foundation))]
[RequireComponent(typeof(HoverController_Energy))]
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

    [Tooltip("Energy consumed per second while boost is active. " +
             "Default tuned so a full tank (~100 energy, regenRate ~20/s) lasts ~5s of pure boost.")]
    [Min(0f)]
    [SerializeField] private float boostEnergyPerSecond = 20f;

    private float boostLerp; // 0..1, managed by ApplyBoostBlend

    // -------------------------------------------------------------------------
    // 🦘 Jump
    // -------------------------------------------------------------------------
    [Header("🦘 Jump")]
    [Tooltip("Enable jump.")]
    [SerializeField] private bool enableJump = true;

    [Tooltip("Upward velocity added (m/s) at minimum charge (tap). " +
             "Uses VelocityChange — mass-independent. Same jump height on all vehicles.")]
    [Min(0f)]
    [SerializeField] private float jumpImpulseMin = 4f;

    [Tooltip("Upward velocity added (m/s) at maximum charge (full hold). " +
             "Uses VelocityChange — mass-independent. Same jump height on all vehicles.")]
    [Min(0f)]
    [SerializeField] private float jumpImpulseMax = 12f;

    [Tooltip("Seconds of hold required to reach full charge. " +
             "Charge caps and holds at max — releases on button up.")]
    [Min(0.05f)]
    [SerializeField] private float jumpMaxChargeTime = 2f;

    [Tooltip("Seconds after landing before another jump is allowed.")]
    [Min(0f)]
    [SerializeField] private float jumpGroundedLockout = 0.2f;

    [Tooltip("Energy cost for a grounded jump. Flat cost on fire regardless of charge level.")]
    [Min(0f)]
    [SerializeField] private float jumpGroundedEnergyCost = 25f;

    [Tooltip("Energy cost for an air jump.")]
    [Min(0f)]
    [SerializeField] private float jumpAirEnergyCost = 25f;

    [Tooltip("Upward velocity added (m/s) for the air jump. Not charge-based. " +
             "Uses VelocityChange — mass-independent. Same jump height on all vehicles.")]
    [Min(0f)]
    [SerializeField] private float airJumpImpulse = 7f;

    // ── Jump runtime state ──────────────────────────────────────────────────

    /// <summary>
    /// How long the jump button has been held this press.
    /// Clamped to jumpMaxChargeTime. Resets on fire or when airborne without input.
    /// </summary>
    private float jumpChargeTimer;

    /// <summary>
    /// True while the jump button was held last FixedUpdate.
    /// Used only for air jump edge detection.
    /// </summary>
    private bool jumpHeldLastFrame;

    /// <summary>Seconds remaining in the post-land lockout. Jump blocked while > 0.</summary>
    private float jumpLockoutTimer;

    /// <summary>
    /// True when the air jump token is available.
    /// Granted on airborne->grounded transition. Consumed on air jump fire.
    /// </summary>
    private bool airJumpAvailable;

    /// <summary>
    /// Whether the craft was grounded last FixedUpdate.
    /// Initialized to true in Awake to prevent a false transition grant on frame 0.
    /// </summary>
    private bool wasGroundedLastFrame;

    // -------------------------------------------------------------------------
    // 🔒 Soft Top-Speed Cap
    // -------------------------------------------------------------------------
    [Header("🔒 Soft Top-Speed Cap")]
    [Tooltip("Counter-force (m/s²) applied per m/s of excess horizontal speed. " +
             "Grounded only — airborne speed is uncapped so exit velocity carries. " +
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

    /// <summary>
    /// Current drift blend value (0 = no drift, 1 = full drift).
    /// Read by HoverCameraController for shoulder shift magnitude.
    /// </summary>
    public float DriftLerp => driftLerp;

    // -------------------------------------------------------------------------
    // 🎯 Strafe Mode
    // -------------------------------------------------------------------------
    [Header("🎯 Strafe Mode")]
    [Tooltip("Enable strafe/aim mode (Left Trigger).")]
    [SerializeField] private bool enableStrafe = true;

    [Tooltip("Maximum speed (m/s) the vehicle can sustain or build to while in strafe mode. " +
             "Entry speed above this bleeds off naturally via the soft cap. " +
             "Cannot be re-built above this threshold in strafe.")]
    [Min(1f)]
    [SerializeField] private float strafeTopSpeed = 20f;

    [Tooltip("Acceleration (m/s²) for omni-directional strafe movement on the ground plane. " +
             "Lower than normal forward accel — strafe is maneuvering, not charging.")]
    [Min(0f)]
    [SerializeField] private float strafeAccel = 15f;

    [Tooltip("Maximum pitch angle (degrees) the vehicle nose can tilt up or down in strafe mode. " +
             "Applied via torque — Foundation's leveling works against this, creating natural resistance.")]
    [Range(5f, 45f)]
    [SerializeField] private float strafePitchLimit = 15f;

    [Tooltip("Torque strength driving the vehicle toward the target pitch angle in strafe mode. " +
             "Keep this LOW — Foundation's levelingTorqueStrength fights it, so values above 5 " +
             "will oscillate. Recommended: 2–5.")]
    [Range(0.5f, 15f)]
    [SerializeField] private float strafePitchTorque = 3f;

    [Tooltip("Damping counter-torque applied to the pitch angular velocity in strafe mode. " +
             "Critical for preventing oscillation. Pair with strafePitchTorque. Recommended: 4–8.")]
    [Range(0f, 20f)]
    [SerializeField] private float strafePitchDamping = 6f;

    [Tooltip("Time (seconds) to blend strafe movement in when entering strafe mode.")]
    [Min(0.05f)]
    [SerializeField] private float strafeModeBlendSeconds = 0.2f;

    private float _strafeModeBlend; // 0..1, blends strafe movement authority in/out

    /// <summary>
    /// Maximum pitch angle (degrees) the vehicle nose can tilt in strafe mode.
    /// Read by HoverController_Aim to keep weapon aim range consistent with vehicle pitch range.
    /// </summary>
    public float StrafePitchLimit => strafePitchLimit;

    /// <summary>
    /// Current strafe mode blend weight (0 = drive mode, 1 = full strafe).
    /// Read by HoverController_Aim to scale aim pitch in sync with strafe entry/exit.
    /// </summary>
    public float StrafeModeBlend => _strafeModeBlend;

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
    private IHoverInputProvider        input;
    private HoverController_Foundation foundation;
    private HoverController_Energy     energy;
    private Rigidbody                  rb;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        rb         = GetComponent<Rigidbody>();
        foundation = GetComponent<HoverController_Foundation>();
        energy     = GetComponent<HoverController_Energy>();

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

        wasGroundedLastFrame = true;
        airJumpAvailable     = false;
    }

    private void FixedUpdate()
    {
        // Cache once per tick — read by multiple methods below.
        bool  grounded      = foundation.IsHoverGrounded;
        float effectiveTopSpeed    = topSpeed        * Mathf.Lerp(1f, boostSpeedMultiplier,  boostLerp);
        float effectiveForwardAccel = maxForwardAccel * Mathf.Lerp(1f, boostAccelMultiplier, boostLerp);

        ApplyBoostBlend();
        ApplyDriftBlend();
        ApplyStrafeModeBlend();
        ApplyDrive(grounded, effectiveTopSpeed, effectiveForwardAccel);
        ApplyStrafe(grounded, effectiveTopSpeed, effectiveForwardAccel);
        ApplyTurning(grounded);
        ApplyDrag(grounded);
        ApplySoftSpeedCap(grounded, effectiveTopSpeed);
        ApplyStrafePitch();
        HandleJump(grounded);
    }

    private void Update()
    {
        // Chassis bank is visual only — runs in Update for smooth interpolation
        // independent of the physics timestep.
        ApplyChassisBank();
    }

    // -------------------------------------------------------------------------
    // ⚡ Boost blend — energy-gated
    // -------------------------------------------------------------------------
    /// <summary>
    /// Boost advances boostLerp toward 1 only when:
    ///   1. enableBoost is true.
    ///   2. The player is holding boost input.
    ///   3. Energy can cover the cost this frame (TryConsume succeeds).
    ///
    /// If energy is depleted or EMP-frozen mid-boost, TryConsume returns false,
    /// target drops to 0, and boostLerp fades out over boostBlendSeconds.
    /// The blend fade preserves the smooth feel even on a hard energy cutoff.
    /// </summary>
    private void ApplyBoostBlend()
    {
        bool wantsBoost    = enableBoost && input.Boost;
        bool energyGranted = wantsBoost &&
                             energy.TryConsume(boostEnergyPerSecond * Time.fixedDeltaTime);

        float target = energyGranted ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, boostBlendSeconds);
        boostLerp    = Mathf.MoveTowards(boostLerp, target, step);
    }

    // -------------------------------------------------------------------------
    // 🌀 Drift blend
    // -------------------------------------------------------------------------
    /// <summary>
    /// Drift engages when:
    ///   1. Drift button is held.
    ///   2. Turn input exceeds driftTurnThreshold.
    ///   3. Craft is grounded (no drift airborne — carry is already free in air).
    ///
    /// driftLerp drives all drift-state values: lateralDamp, yawAccel, chassis bank.
    /// </summary>
    private void ApplyDriftBlend()
    {
        bool driftCondition = input.Drift
                           && Mathf.Abs(input.TurnInput) >= driftTurnThreshold
                           && foundation.IsHoverGrounded;

        float target = driftCondition ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, driftBlendSeconds);
        driftLerp    = Mathf.MoveTowards(driftLerp, target, step);
    }

    // -------------------------------------------------------------------------
    // 🦘 Jump — charge-based grounded + fixed air jump, both energy-gated
    // -------------------------------------------------------------------------
    /// <summary>
    /// Grounded jump:
    ///   • Hold jump button: jumpChargeTimer accumulates, clamped at jumpMaxChargeTime.
    ///   • Release (jumpHeld == false AND chargeTimer > 0): fire.
    ///   • Release detection is stateless — no dependency on jumpHeldLastFrame.
    ///     This avoids the FixedUpdate/Update timing mismatch that caused the
    ///     previous edge detection to silently fail.
    ///
    /// Air jump:
    ///   • Token granted on airborne->grounded transition. wasGroundedLastFrame is
    ///     initialized to true in Awake to prevent a false grant on frame 0.
    ///   • Fires on button press while airborne.
    ///   • Token not consumed on energy denial — player can retry when reserves recover.
    ///
    /// Physics note:
    ///   • ForceMode.VelocityChange adds m/s directly, ignoring Rigidbody mass.
    ///     Guarantees identical jump height across all vehicles regardless of mass.
    ///     jumpImpulseMin/Max and airJumpImpulse are direct m/s values.
    ///   • This is the only direct velocity modification in Propulsion and is intentional.
    /// </summary>
    private void HandleJump(bool grounded)
    {
        if (!enableJump)
            return;

        bool jumpHeld    = input.Jump;
        bool jumpPressed = jumpHeld && !jumpHeldLastFrame;

        // ── Lockout countdown ────────────────────────────────────────────────
        if (jumpLockoutTimer > 0f)
            jumpLockoutTimer = Mathf.Max(0f, jumpLockoutTimer - Time.fixedDeltaTime);

        // ── Air jump token: grant on airborne -> grounded transition only ────
        if (grounded && !wasGroundedLastFrame)
            airJumpAvailable = true;

        wasGroundedLastFrame = grounded;

        // ── Grounded jump ────────────────────────────────────────────────────
        if (grounded && jumpLockoutTimer <= 0f)
        {
            if (jumpHeld)
            {
                jumpChargeTimer = Mathf.Min(
                    jumpChargeTimer + Time.fixedDeltaTime,
                    jumpMaxChargeTime
                );
            }
            else if (jumpChargeTimer > 0f)
            {
                // Button is not held AND charge has built — fire.
                // Stateless: no edge detection, no dependency on last frame.
                // Correct regardless of FixedUpdate/Update timing.
                FireGroundedJump();
            }
        }

        // ── Air jump ─────────────────────────────────────────────────────────
        if (!grounded)
        {
            // Reset any phantom charge carried from before leaving the ground.
            if (!jumpHeld && jumpChargeTimer > 0f)
                jumpChargeTimer = 0f;

            if (jumpPressed && airJumpAvailable)
                FireAirJump();
        }

        jumpHeldLastFrame = jumpHeld;
    }

    /// <summary>
    /// Fires the grounded jump. Charge maps linearly from min to max impulse.
    /// Charge resets regardless of energy outcome.
    /// Lockout timer starts on successful fire only.
    /// </summary>
    private void FireGroundedJump()
    {
        float chargeT = Mathf.Clamp01(jumpChargeTimer / jumpMaxChargeTime);
        float impulse = Mathf.Lerp(jumpImpulseMin, jumpImpulseMax, chargeT);

        jumpChargeTimer = 0f; // always reset, even if energy denies

        if (!energy.TryConsume(jumpGroundedEnergyCost))
            return;

        // ForceMode.VelocityChange: adds m/s directly, ignoring mass.
        // Guarantees identical jump height across all vehicle Rigidbody masses.
        rb.AddForce(Vector3.up * impulse, ForceMode.VelocityChange);
        jumpLockoutTimer = jumpGroundedLockout;

        if (drawDebug)
            Debug.DrawRay(transform.position, Vector3.up * impulse * 0.5f, Color.cyan, 0.5f);
    }

    /// <summary>
    /// Fires the air jump. Fixed impulse, no charge.
    /// Token is NOT consumed on energy denial — player retains it for retry.
    /// </summary>
    private void FireAirJump()
    {
        if (!energy.TryConsume(jumpAirEnergyCost))
            return;

        airJumpAvailable = false;

        // ForceMode.VelocityChange: adds m/s directly, ignoring mass.
        // Guarantees identical air jump height across all vehicle Rigidbody masses.
        rb.AddForce(Vector3.up * airJumpImpulse, ForceMode.VelocityChange);

        if (drawDebug)
            Debug.DrawRay(transform.position, Vector3.up * airJumpImpulse * 0.5f, Color.magenta, 0.5f);
    }

    // -------------------------------------------------------------------------
    // Drive — unified throttle + assist in one force pass
    // -------------------------------------------------------------------------
    /// <summary>
    /// Airborne drive rules:
    ///   • Throttle accel and speed-assist servo are both suppressed when airborne.
    ///   • Boost is the only way to gain speed in the air — boostLerp > 0 re-enables
    ///     both the raw accel and the assist servo scaled by their boost multipliers.
    ///   • This preserves exit velocity as a projectile feel.
    ///
    /// Grounded drive rules:
    ///   • Raw throttle provides the immediate arcade response floor.
    ///   • Speed-assist servo converges forward speed toward the target.
    /// </summary>
    private void ApplyDrive(bool grounded, float effectiveTopSpeed, float effectiveForwardAccel)
    {
        bool boosting = boostLerp > 0f;

        if (!grounded && !boosting)
            return;

        float throttle = Mathf.Clamp(input.ThrottleInput, -1f, 1f);

        float rawAccel = 0f;
        if (grounded)
        {
            rawAccel = throttle >= 0f
                ? throttle * effectiveForwardAccel
                : throttle * maxReverseAccel;
        }
        else
        {
            rawAccel = Mathf.Max(throttle, 0f) * effectiveForwardAccel;
        }

        rb.AddForce(transform.forward * rawAccel, ForceMode.Acceleration);

        // Speed assist is suppressed in strafe mode — it assumes the vehicle wants
        // to maintain speed along forward, which conflicts with omni-directional
        // intent where the player may be deliberately crossing their own momentum.
        if (_strafeModeBlend < 1f)
        {
            float targetSpeed = throttle * effectiveTopSpeed;
            float currentFwd  = Vector3.Dot(rb.linearVelocity, transform.forward);
            float error       = targetSpeed - currentFwd;
            float assistAccel = Mathf.Clamp(error * speedAssistStrength, -speedAssistMaxAccel, speedAssistMaxAccel);
            assistAccel *= (1f - _strafeModeBlend);

            if (!grounded && assistAccel < 0f)
                assistAccel = 0f;

            rb.AddForce(transform.forward * assistAccel, ForceMode.Acceleration);

            if (drawDebug)
                Debug.DrawRay(transform.position, transform.forward * (rawAccel + assistAccel), Color.yellow);
        }
        else if (drawDebug)
        {
            Debug.DrawRay(transform.position, transform.forward * rawAccel, Color.yellow);
        }
    }

    // -------------------------------------------------------------------------
    // Turning — yaw torque + counter-torque damping
    // -------------------------------------------------------------------------
    private void ApplyTurning(bool grounded)
    {
        float turn      = Mathf.Clamp(input.TurnInput, -1f, 1f);
        float turnScale = grounded ? 1f : airTurnMultiplier;

        // Effective yaw multiplier inlined — lerp between 1 and driftYawMultiplier.
        float effectiveYawMult = Mathf.Lerp(1f, driftYawMultiplier, driftLerp);

        float inertiaY      = Mathf.Max(0.001f, rb.inertiaTensor.y);
        float desiredYawAcc = turn * yawAccel * turnScale * effectiveYawMult;
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
    private void ApplyDrag(bool grounded)
    {
        if (!grounded)
            return;

        // Omni-directional drag — applied symmetrically to both lateral (X) and
        // forward (Z) axes so deceleration feels consistent in all directions.
        // lateralDamp is now effectively an omni-damp value; rename in Inspector
        // if desired. Drift state reduces it via driftLateralDamp on both axes.
        float effectiveDamp = Mathf.Lerp(lateralDamp, driftLateralDamp, driftLerp);

        if (effectiveDamp <= 0f)
            return;

        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        rb.AddForce(transform.right   * (-localVel.x * effectiveDamp), ForceMode.Acceleration);
        rb.AddForce(transform.forward * (-localVel.z * effectiveDamp), ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // Chassis bank — visual only, runs in Update
    // -------------------------------------------------------------------------
    private void ApplyChassisBank()
    {
        if (meshRoot == null)
            return;

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
    // 🎯 Strafe Mode Blend
    // -------------------------------------------------------------------------
    /// <summary>
    /// Smoothly blends strafe mode authority in and out on trigger hold/release.
    /// _strafeModeBlend is the weight applied to strafe-specific forces.
    /// Drive forces are attenuated by (1 - _strafeModeBlend) to prevent
    /// double-force during the blend transition.
    /// </summary>
    private void ApplyStrafeModeBlend()
    {
        if (!enableStrafe)
        {
            _strafeModeBlend = 0f;
            return;
        }

        float target = input.StrafeHeld ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, strafeModeBlendSeconds);
        _strafeModeBlend = Mathf.MoveTowards(_strafeModeBlend, target, step);
    }

    // -------------------------------------------------------------------------
    // 🎯 Omni-Directional Lateral Movement (both modes)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Left Stick X drives lateral movement in both drive and strafe mode.
    /// Left Stick Y (ThrottleInput) handles forward/back in both modes via ApplyDrive.
    ///
    /// In drive mode, ApplyDrag's lateral damp partially resists sideways movement
    /// so the vehicle still feels directional. In strafe mode, lateral damp is
    /// suppressed so movement is fully free.
    ///
    /// The per-axis speed cap only kicks in when a single local axis exceeds
    /// strafeTopSpeed — forward and lateral are capped independently so entry
    /// momentum doesn't strangle lateral acceleration.
    /// </summary>
    private void ApplyStrafe(bool grounded, float effectiveTopSpeed, float effectiveLateralAccel)
    {
        if (!grounded)
            return;

        float stickX = Mathf.Clamp(input.StrafeX, -1f, 1f);

        if (Mathf.Abs(stickX) < 0.001f)
            return;

        // Lateral accel and top speed scale with boost the same way forward does.
        // effectiveLateralAccel = strafeAccel * boostAccelMultiplier during boost.
        // effectiveLateralTopSpeed = strafeTopSpeed * boostSpeedMultiplier during boost.
        float effectiveLateralAccelScaled = strafeAccel * (effectiveLateralAccel / maxForwardAccel);
        float effectiveLateralTopSpeed    = strafeTopSpeed * (effectiveTopSpeed / topSpeed);

        rb.AddForce(transform.right * (stickX * effectiveLateralAccelScaled), ForceMode.Acceleration);

        // Per-axis lateral speed cap — only resists if lateral velocity alone
        // exceeds the boost-scaled lateral top speed.
        Vector3 localVel     = transform.InverseTransformDirection(rb.linearVelocity);
        float   localLateral = localVel.x;

        if (Mathf.Abs(localLateral) > effectiveLateralTopSpeed)
        {
            float excess = Mathf.Abs(localLateral) - effectiveLateralTopSpeed;
            rb.AddForce(-transform.right * (Mathf.Sign(localLateral) * excess * softCapStrength),
                        ForceMode.Acceleration);
        }
    }

    // -------------------------------------------------------------------------
    // 🎯 Strafe Pitch — nose tilt up/down via right stick Y
    // -------------------------------------------------------------------------
    /// <summary>
    /// In strafe mode, right stick Y (CameraLookY) tilts the vehicle nose
    /// up or down within strafePitchLimit. This is Interpretation A from the
    /// design discussion — vehicle body IS the turret. Weapons fire along forward.
    ///
    /// Implementation: proportional torque drives toward target pitch; a separate
    /// damping term kills angular velocity on the pitch axis to prevent oscillation.
    /// Foundation's leveling torque naturally snaps back to level on stick release.
    ///
    /// Tuning note: strafePitchTorque must stay LOW relative to Foundation's
    /// levelingTorqueStrength — they're competing forces. strafePitchDamping
    /// is the primary oscillation killer. Start with torque=3, damping=6.
    /// </summary>
    private void ApplyStrafePitch()
    {
        if (_strafeModeBlend <= 0f)
            return;

        float aimY = input.CameraLookY;

        // Small deadzone to prevent jitter at stick center
        if (Mathf.Abs(aimY) < 0.1f)
            aimY = 0f;

        // Target pitch: stick up (+1) = nose up = negative local X euler
        float targetPitch  = -aimY * strafePitchLimit;
        float currentPitch = NormalizeAngle(transform.localEulerAngles.x);
        float pitchError   = targetPitch - currentPitch;

        // Proportional drive toward target pitch
        float driveTorque = pitchError * strafePitchTorque * _strafeModeBlend;

        // Damping: counter-torque opposing current pitch angular velocity
        float localPitchRate = transform.InverseTransformDirection(rb.angularVelocity).x;
        float dampTorque     = -localPitchRate * strafePitchDamping * _strafeModeBlend;

        rb.AddRelativeTorque(Vector3.right * (driveTorque + dampTorque), ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // Soft top-speed cap — grounded only
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies a counter-force proportional to excess horizontal speed.
    /// Grounded only — airborne speed is intentionally uncapped so exit velocity
    /// carries through jumps and off ramps.
    /// Soft cap allows brief breaches on impacts — correct and feels better
    /// than a hard clamp which creates a sticky wall feel on collision frames.
    /// </summary>
    private void ApplySoftSpeedCap(bool grounded, float effectiveTopSpeed)
    {
        if (softCapStrength <= 0f || !grounded)
            return;

        Vector3 vel   = rb.linearVelocity;
        Vector3 horiz = new Vector3(vel.x, 0f, vel.z);
        float   speed = horiz.magnitude;

        if (speed <= effectiveTopSpeed)
            return;

        float   excess       = speed - effectiveTopSpeed;
        Vector3 counterForce = -horiz.normalized * (excess * softCapStrength);
        rb.AddForce(counterForce, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /// <summary>
    /// Normalizes a Unity euler angle from 0..360 to -180..180.
    /// Required for strafe pitch error calculation — Unity returns 350 for -10 degrees.
    /// </summary>
    private static float NormalizeAngle(float angle)
    {
        if (angle > 180f) angle -= 360f;
        return angle;
    }
}
