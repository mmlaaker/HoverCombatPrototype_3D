using UnityEngine;

/// <summary>
/// HoverController_Propulsion v6.0
///
/// v6.0 changes:
///   • Strafe dodge added. In strafe mode, boost button + non-forward stick fires a
///     tapering force burst (jet dodge) instead of continuous boost. Separate inspector
///     fields: dodgeForce, dodgeDuration, dodgeEnergyCost, dodgeCooldown.
///   • Continuous boost now requires meaningful throttle input (>= 0.15 stick deflection).
///     In drive mode: forward or backward. In strafe mode: forward-dominant only
///     (forward must exceed lateral magnitude to prevent stick bleed).
///   • Over-speed bleed extracted to ApplyOverSpeedBleed — proportional counter-force
///     when forward speed exceeds effectiveTopSpeed (e.g. boost fading while throttle
///     is held). Suppressed during drift to avoid false triggers from lateral velocity.
///   • Forward drag forced on during drift regardless of throttle. The drive-drag mutual
///     exclusion assumes heading == velocity, which breaks during drift. Drift's own
///     driftForwardDamp controls the damping feel.
///
/// v5.1 changes:
///   • Drive and forward drag made mutually exclusive. Throttle held: drive applies,
///     forward drag suppressed. Throttle released: drag applies, drive suppressed.
///   • Top speed enforcement moved into ApplyDrive: drive force suppressed when
///     forward speed meets or exceeds effectiveTopSpeed.
///
/// v5.0 changes:
///   • Speed assist servo removed entirely. Raw drive tuning handles speed feel directly.
/// -------------------------------------------------
/// Responsibilities:
///   • Unified drive: grounded only — throttle suppressed airborne unless boosting
///   • Yaw torque + torque-based yaw damping
///   • Independent lateral and forward drag via force (grounded only)
///   • Airborne: drag suppressed so exit velocity carries cleanly as inertia
///   • Drift state: reduces lateral damp, boosts yaw, banks mesh root visually
///   • Boost: continuous forward boost (energy-gated) + strafe dodge burst
///   • Jump: charge-based impulse, one air jump token, energy-gated
///   • Over-speed bleed: decelerates when above top speed cap (post-boost fade)
///
/// Physics contract: zero direct writes to rb.linearVelocity or rb.angularVelocity.
/// All motion is expressed as AddForce / AddTorque.
/// Exception: jump fires rb.AddForce(VelocityChange) — intentional, documented at call site.
///
/// Drift state modifies three physics values (lateralDamp, forwardDamp, yawAccel) and one
/// visual transform (meshRoot local Z rotation). No new forces are introduced.
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

    [Tooltip("Reverse top speed (m/s). Independent of forward top speed. " +
             "Tune alongside maxReverseAccel and strafeTopSpeed — they should feel like a matched budget.")]
    [SerializeField] private float reverseTopSpeed = 20f;

    [Tooltip("Forward top speed (m/s) before boost.")]
    [SerializeField] private float topSpeed = 40f;

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
    // 💨 Dodge (Strafe Mode)
    // -------------------------------------------------------------------------
    [Header("💨 Dodge (Strafe Mode)")]
    [Tooltip("Peak acceleration (m/s²) applied during the dodge burst. " +
             "Front-loaded and tapering — strongest at trigger, fades to zero. " +
             "ForceMode.Acceleration — mass-independent.")]
    [Min(0f)]
    [SerializeField] private float dodgeForce = 120f;

    [Tooltip("Duration (seconds) of the dodge burst. Force tapers from full to zero " +
             "over this window. Shorter = snappier. Longer = more jet-like. Recommended: 0.15–0.35.")]
    [Min(0.01f)]
    [SerializeField] private float dodgeDuration = 0.25f;

    [Tooltip("Flat energy cost per dodge burst.")]
    [Min(0f)]
    [SerializeField] private float dodgeEnergyCost = 20f;

    [Tooltip("Seconds between dodge bursts. Prevents spam.")]
    [Min(0f)]
    [SerializeField] private float dodgeCooldown = 0.5f;

    private float   dodgeCooldownTimer;
    private float   dodgeForceTimer;   // counts down while dodge force is being applied
    private Vector3 dodgeForceDir;     // world-space direction cached at trigger time
    private bool    boostHeldLastFrame; // for rising-edge detection on boost button

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
    // 🧲 Drag
    // -------------------------------------------------------------------------
    [Header("🧲 Drag")]
    [Tooltip("Sideways counter-force (m/s²) resisting lateral velocity. " +
             "Controls how tightly the craft tracks its heading and how much " +
             "strafe acceleration must overcome to build lateral speed. " +
             "Tune this independently of forwardDamp — high values here require " +
             "higher strafeAccel to compensate. 0 = fully slippery sideways.")]
    [Range(0f, 50f)]
    [SerializeField] private float lateralDamp = 2f;

    [Tooltip("Forward/reverse counter-force (m/s²) resisting longitudinal velocity. " +
             "Controls coasting bleed-off when throttle is released. " +
             "Independent of lateralDamp — tune for how long the craft coasts " +
             "without affecting strafe feel or fighting the speed-assist servo. " +
             "The servo is target-aware and only active under throttle input; " +
             "forwardDamp applies at all times including zero throttle. " +
             "0 = no forward drag (craft coasts indefinitely until soft cap acts).")]
    [Range(0f, 50f)]
    [SerializeField] private float forwardDamp = 2f;

    // -------------------------------------------------------------------------
    // 🌀 Drift
    // -------------------------------------------------------------------------
    [Header("🌀 Drift")]
    [Tooltip("Lateral damp value while fully in drift state. " +
             "Lower than lateralDamp — the craft slides through the turn. " +
             "0 = fully free sideways.")]
    [Range(0f, 50f)]
    [SerializeField] private float driftLateralDamp = 0f;

    [Tooltip("Forward damp value while fully in drift state. " +
             "Typically kept close to forwardDamp — drift doesn't change " +
             "how the craft coasts forward, only how it slides sideways. " +
             "Reduce slightly if you want the craft to carry more forward momentum through a drift.")]
    [Range(0f, 50f)]
    [SerializeField] private float driftForwardDamp = 2f;

    [Tooltip("Yaw acceleration multiplier while fully in drift state. " +
             "Higher than 1 — nose rotates faster than the momentum vector, " +
             "creating the shouldering angle. Recommended: 1.2–1.6.")]
    [Range(1f, 3f)]
    [SerializeField] private float driftYawMultiplier = 1.4f;

    [Tooltip("Minimum absolute turn input required to engage drift. " +
             "Prevents drift activating on gentle steering. Recommended: 0.3–0.5.")]
    [Range(0f, 1f)]
    [SerializeField] private float driftTurnThreshold = 0.4f;

    [Tooltip("Minimum forward speed (m/s) required to initiate drift. " +
             "Once drifting, speed is not checked — you own the drift until you release the button. " +
             "Aligns naturally with strafeTopSpeed: if you've outpaced strafe mode's ceiling, you can drift.")]
    [Min(0f)]
    [SerializeField] private float minDriftSpeed = 20f;

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

    [Tooltip("Passive bank angle (degrees) applied proportional to turn input during normal non-drift turns. " +
             "Gives the craft a subtle carving look at all times, independent of drift. " +
             "Additive with drift bank — keep low so it doesn't compete visually. Recommended: 3–5.")]
    [Range(0f, 15f)]
    [SerializeField] private float passiveBankAngle = 4f;

    [Tooltip("Speed (lerp t per second) at which the chassis bank visually catches up. " +
             "Recommended: 6–10.")]
    [Range(1f, 20f)]
    [SerializeField] private float bankLerpSpeed = 8f;

    private float driftLerp;   // 0..1, managed by ApplyDriftBlend
    private bool  _isDrifting; // entry-gate state — true once speed+turn initiated drift

    /// <summary>
    /// Current drift blend value (0 = no drift, 1 = full drift).
    /// Read by HoverCameraController for shoulder shift magnitude.
    /// </summary>
    public float DriftLerp => driftLerp;

    /// <summary>
    /// Current boost blend value (0 = no boost, 1 = full boost).
    /// Read by HoverVehicleVFX to modulate particle emission rates.
    /// </summary>
    public float BoostLerp => boostLerp;

    // -------------------------------------------------------------------------
    // 📢 Events
    // -------------------------------------------------------------------------

    /// <summary>
    /// Fired when a jump is denied due to insufficient energy.
    /// Allows HUD/audio to communicate the failure to the player.
    /// Parameter: true = grounded jump denied, false = air jump denied.
    /// </summary>
    public event System.Action<bool> OnJumpDenied;

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

    [Tooltip("Aim sensitivity in degrees per second at full stick deflection. " +
             "Controls how fast the pitch angle changes with stick input. Recommended: 60–120.")]
    [Range(10f, 300f)]
    [SerializeField] private float strafePitchSensitivity = 90f;

    [Tooltip("Time (seconds) to blend strafe movement in when entering strafe mode.")]
    [Min(0.05f)]
    [SerializeField] private float strafeModeBlendSeconds = 0.2f;

    [Tooltip("Counter-force (m/s²) per m/s of excess lateral speed in strafe mode. " +
             "Caps lateral velocity at strafeTopSpeed without a hard clamp. " +
             "Independent of the removed global soft cap — this only acts on the " +
             "lateral axis during strafe movement. Recommended: 20–60.")]
    [Range(0f, 120f)]
    [SerializeField] private float strafeLateralCapStrength = 40f;

    private float _strafeModeBlend; // 0..1, blends strafe movement authority in/out
    private float _strafePitchAccum; // accumulated FPS-style pitch angle (degrees)

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
    // Acquired via GetComponent<IHoverInputProvider>() in Awake.
    // Attach PlayerHoverInput (or any AIHoverInput implementation) to this same
    // GameObject. No inspector wiring needed — swapping the component changes
    // who drives the vehicle without touching any other script.

    // -------------------------------------------------------------------------
    // 🧭 Debug
    // -------------------------------------------------------------------------
    [Header("🧭 Debug")]
    [SerializeField] private bool drawDebug = false;

    [Tooltip("Optional global debug toggle. When assigned, overrides drawDebug.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.enableDebugGizmos : drawDebug;

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

        input = GetComponent<IHoverInputProvider>();
        if (input == null)
        {
            Debug.LogError(
                $"[Propulsion] '{name}': No IHoverInputProvider found on this GameObject. " +
                $"Attach PlayerHoverInput or an AI implementation. Vehicle will not respond to input.",
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

    // Cached once per FixedUpdate — read by multiple methods below.
    // Avoids redundant rb.linearVelocity reads and InverseTransformDirection calls.
    private Vector3 _cachedLocalVel;

    private void FixedUpdate()
    {
        // Cache once per tick — read by multiple methods below.
        bool  grounded      = foundation.IsHoverGrounded;
        float effectiveTopSpeed    = topSpeed        * Mathf.Lerp(1f, boostSpeedMultiplier,  boostLerp);
        float effectiveForwardAccel = maxForwardAccel * Mathf.Lerp(1f, boostAccelMultiplier, boostLerp);
        _cachedLocalVel = transform.InverseTransformDirection(rb.linearVelocity);

        ApplyBoostBlend();
        ApplyDriftBlend();
        ApplyStrafeModeBlend();
        ApplyDrive(grounded, effectiveTopSpeed, effectiveForwardAccel);
        ApplyStrafe(grounded, effectiveTopSpeed, effectiveForwardAccel);
        ApplyTurning(grounded);
        ApplyDrag();
        ApplyOverSpeedBleed(effectiveTopSpeed);
        ApplyStrafePitch();
        HandleJump(grounded);
        HandleDodge(grounded);
        ApplyDodgeForce();
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
        // Drive mode: forward OR backward throttle enables continuous boost.
        // Strafe mode: only forward-dominant stick enables continuous boost —
        // lateral/back directions route to dodge burst instead.
        // "Forward-dominant" means forward exceeds lateral magnitude, preventing
        // stick bleed from a sideways push from triggering continuous boost.
        bool inStrafe    = _strafeModeBlend > 0f;
        bool hasThrottle = inStrafe
            ? (input.ThrottleInput >= 0.15f && input.ThrottleInput > Mathf.Abs(input.StrafeX))
            : Mathf.Abs(input.ThrottleInput) >= 0.15f;
        bool wantsBoost  = enableBoost && input.Boost && hasThrottle;
        bool energyGranted = wantsBoost &&
                             energy.TryConsume(boostEnergyPerSecond * Time.fixedDeltaTime);

        float target = energyGranted ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, boostBlendSeconds);
        boostLerp    = Mathf.MoveTowards(boostLerp, target, step);
    }

    // -------------------------------------------------------------------------
    // 💨 Dodge — strafe-mode burst on boost press
    // -------------------------------------------------------------------------
    /// <summary>
    /// In strafe mode, pressing boost without forward throttle triggers a dodge:
    /// a short sustained force window in the stick direction (left, right, or back).
    ///
    /// Force is front-loaded and tapers to zero over dodgeDuration — reads as a
    /// jet burst rather than an instant velocity snap. Same pattern as Foundation's
    /// unstick lift. ForceMode.Acceleration — mass-independent.
    ///
    /// HandleDodge detects the trigger and caches direction + timer.
    /// ApplyDodgeForce runs every FixedUpdate to apply the tapering force.
    /// </summary>
    private void HandleDodge(bool grounded)
    {
        if (!enableBoost || _strafeModeBlend <= 0f)
        {
            boostHeldLastFrame = input.Boost;
            return;
        }

        // Cooldown tick
        if (dodgeCooldownTimer > 0f)
            dodgeCooldownTimer = Mathf.Max(0f, dodgeCooldownTimer - Time.fixedDeltaTime);

        bool boostPressed = input.Boost && !boostHeldLastFrame;
        boostHeldLastFrame = input.Boost;

        if (!boostPressed || dodgeCooldownTimer > 0f)
            return;

        // Only dodge when stick is NOT in forward-dominant territory.
        // Mirrors the check in ApplyBoostBlend — forward-dominant means
        // continuous boost, everything else means dodge.
        if (input.ThrottleInput >= 0.15f && input.ThrottleInput > Mathf.Abs(input.StrafeX))
            return;

        // Build dodge direction from stick input — lateral + backward only.
        float dodgeLat = input.StrafeX;
        float dodgeFwd = Mathf.Min(input.ThrottleInput, 0f); // clamp out positive
        Vector3 localDir = new Vector3(dodgeLat, 0f, dodgeFwd);

        if (localDir.sqrMagnitude < 0.01f)
            return;

        if (!energy.TryConsume(dodgeEnergyCost))
            return;

        dodgeForceDir   = (transform.right * localDir.x + transform.forward * localDir.z).normalized;
        dodgeForceTimer = dodgeDuration;
        dodgeCooldownTimer = dodgeCooldown;

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, dodgeForceDir * 3f, Color.blue, 0.5f);
    }

    // -------------------------------------------------------------------------
    // 💨 Sustained Dodge Force
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies the dodge force over a short window each FixedUpdate.
    /// dodgeForceTimer is set by HandleDodge when the trigger condition is met.
    /// Force is front-loaded — strongest on the first frame, tapering to zero.
    /// Reads as a jet burst rather than an instant teleport.
    /// ForceMode.Acceleration keeps the behavior mass-independent.
    /// </summary>
    private void ApplyDodgeForce()
    {
        if (dodgeForceTimer <= 0f)
            return;

        float progress  = dodgeForceTimer / Mathf.Max(0.01f, dodgeDuration);
        float magnitude = dodgeForce * progress;

        rb.AddForce(dodgeForceDir * magnitude, ForceMode.Acceleration);

        dodgeForceTimer = Mathf.Max(0f, dodgeForceTimer - Time.fixedDeltaTime);
    }

    // -------------------------------------------------------------------------
    // 🌀 Drift blend
    // -------------------------------------------------------------------------
    /// <summary>
    /// Drift engages when:
    ///   1. Drift button is held.
    ///   2. Turn input exceeds driftTurnThreshold.
    ///   3. Craft is grounded.
    ///   4. Forward speed >= minDriftSpeed at the moment of initiation (entry gate only).
    ///
    /// Once initiated, speed is not re-checked — the drift sustains until button release
    /// or turn input drops below threshold. Scrubbing speed through a corner does not
    /// eject the player from drift state mid-arc.
    ///
    /// driftLerp drives all drift-state values: lateralDamp, yawAccel, chassis bank.
    /// </summary>
    private void ApplyDriftBlend()
    {
        bool baseCondition = input.Drift
                          && Mathf.Abs(input.TurnInput) >= driftTurnThreshold
                          && foundation.IsHoverGrounded;

        if (_isDrifting)
        {
            // Already drifting — sustain without re-checking speed.
            _isDrifting = baseCondition;
        }
        else
        {
            // Not yet drifting — require minimum forward speed to initiate.
            _isDrifting = baseCondition && _cachedLocalVel.z >= minDriftSpeed;
        }

        float target = _isDrifting ? 1f : 0f;
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
    /// Charge resets regardless of energy outcome — intentional design.
    /// You cannot hold a charged jump if you didn't have the energy to execute it;
    /// the release motion happened and the charge is spent whether the jump fires or not.
    /// Lockout timer starts on successful fire only.
    /// </summary>
    private void FireGroundedJump()
    {
        float chargeT = Mathf.Clamp01(jumpChargeTimer / jumpMaxChargeTime);
        float impulse = Mathf.Lerp(jumpImpulseMin, jumpImpulseMax, chargeT);

        // Reset charge unconditionally — the player released the button.
        // If energy denies the jump, the charge is still gone: no energy = no jump,
        // even if you held the button for a full charge.
        jumpChargeTimer = 0f;

        if (!energy.TryConsume(jumpGroundedEnergyCost))
        {
            OnJumpDenied?.Invoke(true);
            return;
        }

        // ForceMode.VelocityChange: adds m/s directly, ignoring mass.
        // Guarantees identical jump height across all vehicle Rigidbody masses.
        rb.AddForce(Vector3.up * impulse, ForceMode.VelocityChange);
        jumpLockoutTimer = jumpGroundedLockout;

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, Vector3.up * impulse * 0.5f, Color.cyan, 0.5f);
    }

    /// <summary>
    /// Fires the air jump. Fixed impulse, no charge.
    /// Token is NOT consumed on energy denial — player retains it for retry.
    /// </summary>
    private void FireAirJump()
    {
        if (!energy.TryConsume(jumpAirEnergyCost))
        {
            OnJumpDenied?.Invoke(false);
            return;
        }

        airJumpAvailable = false;

        // ForceMode.VelocityChange: adds m/s directly, ignoring mass.
        // Guarantees identical air jump height across all vehicle Rigidbody masses.
        rb.AddForce(Vector3.up * airJumpImpulse, ForceMode.VelocityChange);

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, Vector3.up * airJumpImpulse * 0.5f, Color.magenta, 0.5f);
    }

    // -------------------------------------------------------------------------
    // Drive — unified throttle force
    // -------------------------------------------------------------------------
    /// <summary>
    /// Throttle-exclusive force model — drive and forward drag are mutually exclusive:
    ///   • Throttle held: drive force applied, forward drag suppressed (see ApplyDrag).
    ///   • Throttle released: drive suppressed, forward drag applied.
    ///   Never both simultaneously — eliminates the opposing-force oscillation that
    ///   causes jitter at any speed between zero and top speed.
    ///
    /// Top speed enforcement: drive force is suppressed when forward speed already
    ///   meets or exceeds effectiveTopSpeed. Drag is no longer needed to hold the
    ///   ceiling — the drive simply stops pushing.
    ///
    /// Airborne: throttle suppressed without boost. Boost re-enables drive scaled
    ///   by boost multipliers. Exit velocity carries cleanly as inertia.
    /// </summary>
    private void ApplyDrive(bool grounded, float effectiveTopSpeed, float effectiveForwardAccel)
    {
        bool boosting = boostLerp > 0f;

        if (!grounded && !boosting)
            return;

        float throttle   = Mathf.Clamp(input.ThrottleInput, -1f, 1f);
        float currentFwd = _cachedLocalVel.z;

        // No throttle input — forward drag handles deceleration, nothing to do here.
        if (Mathf.Abs(throttle) < 0.001f)
            return;

        // Blend forward cap and accel toward strafe values in strafe mode.
        // Strafe top speed is boost-scaled the same way ApplyStrafe scales lateral top speed,
        // so boost still has an effect in strafe mode — just relative to the strafe ceiling.
        float strafeEffectiveTopSpeed = strafeTopSpeed * (effectiveTopSpeed / topSpeed);
        float blendedTopSpeed  = Mathf.Lerp(effectiveTopSpeed,     strafeEffectiveTopSpeed, _strafeModeBlend);
        float blendedFwdAccel  = Mathf.Lerp(effectiveForwardAccel, strafeAccel,             _strafeModeBlend);

        float rawAccel = 0f;
        if (grounded)
        {
            if (throttle >= 0f)
            {
                // Suppress drive if already at or above top speed forward.
                if (currentFwd < blendedTopSpeed)
                    rawAccel = throttle * blendedFwdAccel;
            }
            else
            {
                // Suppress reverse drive if already at or below reverse top speed.
                // reverseTopSpeed is independent of strafe mode — reverse and strafe
                // share the same speed budget by design, so no additional blending here.
                if (currentFwd > -reverseTopSpeed)
                    rawAccel = throttle * maxReverseAccel;
            }
        }
        else
        {
            // Airborne with boost — only forward, capped at top speed.
            if (currentFwd < blendedTopSpeed)
                rawAccel = Mathf.Max(throttle, 0f) * blendedFwdAccel;
        }

        if (Mathf.Abs(rawAccel) < 0.001f)
            return;

        rb.AddForce(transform.forward * rawAccel, ForceMode.Acceleration);

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, transform.forward * rawAccel, Color.yellow);
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

        float localYawRate  = transform.InverseTransformDirection(rb.angularVelocity).y;
        float dampingTorque = yawDamping > 0f ? -localYawRate * yawDamping : 0f;

        rb.AddRelativeTorque(Vector3.up * ((desiredYawAcc + dampingTorque) * inertiaY), ForceMode.Force);
    }

    // -------------------------------------------------------------------------
    // Drag — grounded only, both axes
    // -------------------------------------------------------------------------
    /// <summary>
    /// Lateral drag: always applies when grounded — shapes heading tracking and
    ///   strafe feel regardless of throttle state.
    ///
    /// Forward drag: mutually exclusive with drive. Only applies when throttle
    ///   is at or near zero — when the player is coasting or braking.
    ///   When throttle is held, drive handles the force and forward drag is
    ///   suppressed entirely. This eliminates the opposing-force oscillation
    ///   that causes jitter at any speed between zero and top speed.
    ///
    /// Drift state reduces both independently via driftLateralDamp and
    /// driftForwardDamp.
    /// </summary>
    private void ApplyDrag()
    {
        if (!foundation.IsHoverGrounded)
            return;

        // Lateral drag — always active grounded, independent of throttle.
        float effectiveLateralDamp = Mathf.Lerp(lateralDamp, driftLateralDamp, driftLerp);
        if (effectiveLateralDamp > 0f)
            rb.AddForce(transform.right * (-_cachedLocalVel.x * effectiveLateralDamp), ForceMode.Acceleration);

        // Forward drag — fades in as throttle approaches zero.
        // Full drag at throttle == 0, zero drag at throttle >= 0.15.
        // Prevents the binary snap between "full drag" and "zero drag" that
        // feels twitchy on worn sticks or light inputs.
        // Exception: always apply full forward drag during drift. The drive-drag
        // mutual exclusion assumed heading == velocity, which breaks during drift.
        // Drive force along the yawing heading with no forward drag to resist
        // destabilizes the vehicle. Drift's own driftForwardDamp handles the
        // reduced damping feel.
        float throttleMag = Mathf.Abs(input.ThrottleInput);
        float dragWeight  = driftLerp > 0f
            ? 1f
            : 1f - Mathf.Clamp01(throttleMag / 0.15f);
        if (dragWeight > 0f)
        {
            float effectiveForwardDamp = Mathf.Lerp(forwardDamp, driftForwardDamp, driftLerp);
            if (effectiveForwardDamp > 0f)
                rb.AddForce(transform.forward * (-_cachedLocalVel.z * effectiveForwardDamp * dragWeight), ForceMode.Acceleration);
        }
    }

    // -------------------------------------------------------------------------
    // Over-speed bleed — velocity-aligned deceleration
    // -------------------------------------------------------------------------
    /// <summary>
    /// When speed exceeds effectiveTopSpeed (e.g. boost fading while throttle is
    /// held), the drive-drag mutual exclusion leaves no force to decelerate.
    /// Drive is suppressed (above cap) and forward drag is suppressed (throttle held).
    ///
    /// This method applies a proportional counter-force along the actual world
    /// velocity direction — NOT along transform.forward. This is critical because
    /// during drift, heading diverges from velocity. A heading-aligned force would
    /// push sideways or vertically, destabilizing the vehicle.
    ///
    /// Velocity-aligned means it always decelerates in the direction the vehicle
    /// is actually moving, regardless of heading orientation. Safe during drift,
    /// safe on slopes, safe at any heading angle.
    /// </summary>
    private void ApplyOverSpeedBleed(float effectiveTopSpeed)
    {
        // Suppressed during drift — drift has its own damping and the lateral
        // velocity component inflates total magnitude, causing false triggers.
        if (driftLerp > 0f)
            return;

        // Use forward-axis speed only — total magnitude includes lateral which
        // is irrelevant to the forward top-speed cap.
        float forwardSpeed = _cachedLocalVel.z;

        if (forwardSpeed > effectiveTopSpeed)
        {
            float excess = forwardSpeed - effectiveTopSpeed;
            rb.AddForce(-transform.forward * excess * forwardDamp, ForceMode.Acceleration);
        }
        else if (forwardSpeed < -reverseTopSpeed)
        {
            // Mirror bleed for reverse — catches dodge burst overshoot and any
            // other impulse that pushes past reverseTopSpeed backward.
            float excess = -reverseTopSpeed - forwardSpeed;
            rb.AddForce(transform.forward * excess * forwardDamp, ForceMode.Acceleration);
        }
    }

    // -------------------------------------------------------------------------
    // Chassis bank — visual only, runs in Update
    // -------------------------------------------------------------------------
    /// <summary>
    /// Two additive bank contributions:
    ///   Passive bank — proportional to turn input magnitude, always active.
    ///                  Gives a subtle carving look on any hard turn.
    ///   Drift bank   — proportional to driftLerp, only during drift state.
    ///                  Exaggerated lean that reads clearly at speed.
    /// Both use the same turn sign so they always lean in the same direction.
    /// </summary>
    private void ApplyChassisBank()
    {
        if (meshRoot == null)
            return;

        float turn     = Mathf.Clamp(input.TurnInput, -1f, 1f);
        float turnSign = Mathf.Sign(turn);
        float turnMag  = Mathf.Abs(turn);

        float passiveAngle = -turnSign * passiveBankAngle * turnMag;
        float driftAngle   = -turnSign * maxBankAngle * driftLerp;
        float targetAngle  = passiveAngle + driftAngle;

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
    /// ApplyDrive blends its top speed and accel caps toward strafe values as
    /// this increases, so forward drive weakens proportionally on strafe entry.
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
    /// Left Stick X drives lateral movement only while strafe mode is active.
    /// Left Stick Y (ThrottleInput) handles forward/back in both modes via ApplyDrive.
    ///
    /// Lateral force scales with _strafeModeBlend so entry/exit is smooth.
    /// In drive mode (blend == 0), lateral movement is fully suppressed —
    /// steering is yaw-only. Lateral damp in ApplyDrag handles residual slide.
    ///
    /// The per-axis speed cap only kicks in when a single local axis exceeds
    /// strafeTopSpeed — forward and lateral are capped independently so entry
    /// momentum doesn't strangle lateral acceleration.
    /// </summary>
    private void ApplyStrafe(bool grounded, float effectiveTopSpeed, float effectiveForwardAccel)
    {
        if (!grounded || _strafeModeBlend <= 0f)
            return;

        float stickX = Mathf.Clamp(input.StrafeX, -1f, 1f);

        if (Mathf.Abs(stickX) < 0.001f)
            return;

        // Lateral accel and top speed scale with boost the same way forward does.
        // Derives the boost ratio from the forward accel multiplier so strafe
        // acceleration scales identically with boost without a separate parameter.
        float lateralAccel             = strafeAccel * (effectiveForwardAccel / maxForwardAccel);
        float effectiveLateralTopSpeed = strafeTopSpeed * (effectiveTopSpeed / topSpeed);

        rb.AddForce(transform.right * (stickX * lateralAccel * _strafeModeBlend), ForceMode.Acceleration);

        // Per-axis lateral speed cap — only resists if lateral velocity alone
        // exceeds the boost-scaled lateral top speed.
        float localLateral = _cachedLocalVel.x;

        if (Mathf.Abs(localLateral) > effectiveLateralTopSpeed)
        {
            float excess = Mathf.Abs(localLateral) - effectiveLateralTopSpeed;
            rb.AddForce(-transform.right * (Mathf.Sign(localLateral) * excess * strafeLateralCapStrength),
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
    /// FPS-style: stick input accumulates as a continuous pitch angle (like mouse
    /// look). Releasing the stick holds the current pitch — no rubber-banding.
    /// The accumulated angle is clamped to [-strafePitchLimit, +strafePitchLimit].
    /// Resets to 0 on strafe exit so Foundation leveling can take over cleanly.
    ///
    /// Implementation: proportional torque drives toward the accumulated target;
    /// a separate damping term kills angular velocity on the pitch axis to
    /// prevent oscillation. Foundation's leveling torque is the opposing force.
    ///
    /// Tuning note: strafePitchTorque must stay LOW relative to Foundation's
    /// levelingTorqueStrength — they're competing forces. strafePitchDamping
    /// is the primary oscillation killer. Start with torque=3, damping=6.
    /// </summary>
    private void ApplyStrafePitch()
    {
        if (_strafeModeBlend <= 0f)
        {
            _strafePitchAccum = 0f;
            return;
        }

        float aimY = input.CameraLookY;

        // Small deadzone to prevent jitter at stick center
        if (Mathf.Abs(aimY) < 0.1f)
            aimY = 0f;

        // Accumulate stick input as delta — FPS-style.
        // Stick up (+1) = nose up = negative pitch convention.
        _strafePitchAccum -= aimY * strafePitchSensitivity * Time.fixedDeltaTime;
        _strafePitchAccum  = Mathf.Clamp(_strafePitchAccum, -strafePitchLimit, strafePitchLimit);

        float currentPitch = HoverMath.NormalizeAngle(transform.localEulerAngles.x);
        float pitchError   = _strafePitchAccum - currentPitch;

        // Proportional drive toward accumulated target pitch
        float driveTorque = pitchError * strafePitchTorque * _strafeModeBlend;

        // Damping: counter-torque opposing current pitch angular velocity
        float localPitchRate = transform.InverseTransformDirection(rb.angularVelocity).x;
        float dampTorque     = -localPitchRate * strafePitchDamping * _strafeModeBlend;

        rb.AddRelativeTorque(Vector3.right * (driveTorque + dampTorque), ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

}
