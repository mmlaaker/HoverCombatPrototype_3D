using UnityEngine;

/// <summary>
/// HoverController_Propulsion v1.0
///
/// Drives the chassis forward, back, sideways, and up. Owns turning, drag, drift,
/// boost, dodge, jump, and strafe-mode authority.
///
/// Key design choices baked in:
///
///   Drive and forward drag are mutually exclusive. Holding throttle applies drive
///   and suppresses forward drag. Releasing throttle applies drag and suppresses
///   drive. Never both at once. This is the only reliable cure for jitter caused
///   by opposing forces fighting at the same speed.
///
///   Top speed is enforced inside ApplyDrive. Once forward speed reaches the cap,
///   drive simply stops pushing. No drag wall is needed to hold the ceiling.
///
///   Drift modifies three physics values (lateralDamp, forwardDamp, yawAccel) and
///   one visual transform (meshRoot Z rotation). It does not introduce new forces.
///
///   Strafe mode blends in lateral authority and a free-aim pitch torque. Forward
///   top speed and accel blend toward strafe values as the blend rises.
///
///   Boost is continuous while throttle is held. In strafe mode, pressing boost
///   without forward throttle fires a tapering dodge burst instead.
///
/// Physics contract: never writes to rb.linearVelocity or rb.angularVelocity.
/// All motion is AddForce / AddTorque. Exception: jump uses VelocityChange so all
/// vehicles jump to the same height regardless of mass.
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
    [Tooltip("How hard the chassis accelerates forward at full throttle, before boost.")]
    [SerializeField] private float maxForwardAccel = 25f;

    [Tooltip("How hard the chassis accelerates in reverse at full throttle.")]
    [SerializeField] private float maxReverseAccel = 15f;

    [Tooltip("Reverse top speed. Independent of forward top speed. " +
             "Tune this alongside Max Reverse Accel and Strafe Top Speed; they should feel like a matched budget.")]
    [SerializeField] private float reverseTopSpeed = 20f;

    [Tooltip("Forward top speed before boost.")]
    [SerializeField] private float topSpeed = 40f;

    // -------------------------------------------------------------------------
    // 🔄 Turning
    // -------------------------------------------------------------------------
    [Header("🔄 Turning")]
    [Tooltip("How fast the chassis rotates at full turn input. Higher feels twitchier and more arcade.")]
    [SerializeField] private float yawAccel = 8f;

    [Tooltip("How firmly turning settles. Counter-torque proportional to current yaw rate. Higher kills wobble.")]
    [Range(0f, 20f)]
    [SerializeField] private float yawDamping = 6f;

    [Tooltip("How much steering authority is retained airborne. 0 disables air turning, 1 is full ground response.")]
    [Range(0f, 1f)]
    [SerializeField] private float airTurnMultiplier = 0.5f;

    // -------------------------------------------------------------------------
    // ⚡ Boost
    // -------------------------------------------------------------------------
    [Header("⚡ Boost")]
    [Tooltip("Master switch for boost.")]
    [SerializeField] private bool enableBoost = true;

    [Tooltip("How much harder the chassis accelerates while boosting.")]
    [Range(1f, 3f)]
    [SerializeField] private float boostAccelMultiplier = 1.75f;

    [Tooltip("How much higher the top speed climbs while boosting.")]
    [Range(1f, 3f)]
    [SerializeField] private float boostSpeedMultiplier = 1.5f;

    [Tooltip("How long it takes for boost to ramp in and ramp out. Longer feels smoother, shorter feels punchier.")]
    [Min(0.01f)]
    [SerializeField] private float boostBlendSeconds = 0.35f;

    [Tooltip("Energy drained per second of continuous boost. " +
             "Defaults assume a full 100-energy tank lasts about 5 seconds of pure boost.")]
    [Min(0f)]
    [SerializeField] private float boostEnergyPerSecond = 20f;

    private float boostLerp; // 0..1, managed by ApplyBoostBlend

    // -------------------------------------------------------------------------
    // 💨 Dodge (Strafe Mode)
    // -------------------------------------------------------------------------
    [Header("💨 Dodge (Strafe Mode)")]
    [Tooltip("Peak strength of the dodge burst. Front-loaded and tapers to zero. Mass independent.")]
    [Min(0f)]
    [SerializeField] private float dodgeForce = 120f;

    [Tooltip("How long the dodge burst lasts as it tapers to zero. Shorter is snappier, longer is more jet-like. Try 0.15 to 0.35.")]
    [Min(0.01f)]
    [SerializeField] private float dodgeDuration = 0.25f;

    [Tooltip("Flat energy cost per dodge.")]
    [Min(0f)]
    [SerializeField] private float dodgeEnergyCost = 20f;

    [Tooltip("Seconds between dodges. Prevents spam.")]
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
    [Tooltip("Master switch for jump.")]
    [SerializeField] private bool enableJump = true;

    [Tooltip("Jump strength on a quick tap (minimum charge). Mass independent so all vehicles reach the same height.")]
    [Min(0f)]
    [SerializeField] private float jumpImpulseMin = 4f;

    [Tooltip("Jump strength on a fully charged hold. Mass independent so all vehicles reach the same height.")]
    [Min(0f)]
    [SerializeField] private float jumpImpulseMax = 12f;

    [Tooltip("How long the player must hold to reach a full charge. Charge holds at full until release.")]
    [Min(0.05f)]
    [SerializeField] private float jumpMaxChargeTime = 2f;

    [Tooltip("Cooldown after landing before another jump is allowed.")]
    [Min(0f)]
    [SerializeField] private float jumpGroundedLockout = 0.2f;

    [Tooltip("Energy cost for a grounded jump. Flat cost regardless of charge level.")]
    [Min(0f)]
    [SerializeField] private float jumpGroundedEnergyCost = 25f;

    [Tooltip("Energy cost for an air jump.")]
    [Min(0f)]
    [SerializeField] private float jumpAirEnergyCost = 25f;

    [Tooltip("Air jump strength. Not charge-based. Mass independent so all vehicles reach the same height.")]
    [Min(0f)]
    [SerializeField] private float airJumpImpulse = 7f;

    // ── Jump runtime state ──────────────────────────────────────────────────

    /// <summary>How long the jump button has been held this press. Capped at jumpMaxChargeTime.</summary>
    private float jumpChargeTimer;

    /// <summary>True while the jump button was held last FixedUpdate. Used only for air jump edge detection.</summary>
    private bool jumpHeldLastFrame;

    /// <summary>Seconds remaining in the post-land lockout. Jump blocked while > 0.</summary>
    private float jumpLockoutTimer;

    /// <summary>
    /// True when the air jump token is available.
    /// Granted on airborne to grounded transition. Consumed on air jump fire.
    /// </summary>
    private bool airJumpAvailable;

    /// <summary>
    /// Whether the chassis was grounded last FixedUpdate.
    /// Initialized true in Awake to avoid a false transition grant on frame 0.
    /// </summary>
    private bool wasGroundedLastFrame;

    // -------------------------------------------------------------------------
    // 🧲 Drag
    // -------------------------------------------------------------------------
    [Header("🧲 Drag")]
    [Tooltip("Sideways resistance to lateral velocity. " +
             "Controls how tightly the chassis tracks its heading and how hard strafe accel must push to build sideways speed. " +
             "Higher needs higher Strafe Accel to compensate. 0 is fully slippery sideways.")]
    [Range(0f, 50f)]
    [SerializeField] private float lateralDamp = 2f;

    [Tooltip("Forward and reverse resistance to longitudinal velocity. " +
             "Controls coasting bleed-off when throttle is released. " +
             "Only applies near-zero throttle (drive and drag are mutually exclusive). " +
             "0 means the chassis coasts indefinitely.")]
    [Range(0f, 50f)]
    [SerializeField] private float forwardDamp = 2f;

    // -------------------------------------------------------------------------
    // 🌀 Drift
    // -------------------------------------------------------------------------
    [Header("🌀 Drift")]
    [Tooltip("Lateral resistance while fully drifting. " +
             "Lower than Lateral Damp so the chassis slides through the turn. 0 is fully free.")]
    [Range(0f, 50f)]
    [SerializeField] private float driftLateralDamp = 0f;

    [Tooltip("Forward resistance while fully drifting. " +
             "Typically close to Forward Damp; drift changes how the chassis slides sideways, not how it coasts forward. " +
             "Lower this slightly to carry more forward momentum through a drift.")]
    [Range(0f, 50f)]
    [SerializeField] private float driftForwardDamp = 2f;

    [Tooltip("Yaw rate multiplier while fully drifting. " +
             "Above 1 means the nose rotates faster than the momentum vector, creating the shouldering angle. Try 1.2 to 1.6.")]
    [Range(1f, 3f)]
    [SerializeField] private float driftYawMultiplier = 1.4f;

    [Tooltip("Minimum turn input required to engage drift. Prevents drift from triggering on gentle steering. Try 0.3 to 0.5.")]
    [Range(0f, 1f)]
    [SerializeField] private float driftTurnThreshold = 0.4f;

    [Tooltip("Minimum forward speed required to start a drift. " +
             "Once drifting, speed is no longer checked: you own the drift until the button releases. " +
             "Aligns naturally with Strafe Top Speed: outpacing strafe means you can drift.")]
    [Min(0f)]
    [SerializeField] private float minDriftSpeed = 20f;

    [Tooltip("How long it takes for drift to ramp in and out. Faster is snappier. Try 0.1 to 0.25.")]
    [Min(0.01f)]
    [SerializeField] private float driftBlendSeconds = 0.15f;

    [Tooltip("Mesh parent (HoverCar). Rotated on local Z for the chassis bank visual. " +
             "Assign the HoverCar object in the inspector, not the root or individual meshes.")]
    [SerializeField] private Transform meshRoot;

    [Tooltip("Maximum chassis lean at full drift. Try 15 to 25. " +
             "Exaggerate slightly: readability matters at speed.")]
    [Range(0f, 45f)]
    [SerializeField] private float maxBankAngle = 20f;

    [Tooltip("Subtle bank applied during normal turns (not drift). " +
             "Gives the chassis a constant carving look. Stacks with drift bank, so keep it low. Try 3 to 5.")]
    [Range(0f, 15f)]
    [SerializeField] private float passiveBankAngle = 4f;

    [Tooltip("How fast the chassis lean catches up to the target angle. Try 6 to 10.")]
    [Range(1f, 20f)]
    [SerializeField] private float bankLerpSpeed = 8f;

    private float driftLerp;   // 0..1, managed by ApplyDriftBlend
    private bool  _isDrifting; // entry-gate state: true once speed+turn initiated drift

    /// <summary>
    /// Current drift blend weight (0 = no drift, 1 = full drift).
    /// Read by HoverCameraController for shoulder shift magnitude.
    /// </summary>
    public float DriftLerp => driftLerp;

    /// <summary>
    /// Current boost blend weight (0 = no boost, 1 = full boost).
    /// Read by HoverVehicleVFX to modulate particle emission rates.
    /// </summary>
    public float BoostLerp => boostLerp;

    // -------------------------------------------------------------------------
    // 📢 Events
    // -------------------------------------------------------------------------

    /// <summary>
    /// Fired when a jump is denied due to insufficient energy.
    /// Allows HUD/audio to communicate the failure.
    /// Parameter: true = grounded jump denied, false = air jump denied.
    /// </summary>
    public event System.Action<bool> OnJumpDenied;

    /// <summary>
    /// Fired when a dodge burst is successfully triggered.
    /// Parameter: local-space direction of the dodge (e.g. x &gt; 0 = right, x &lt; 0 = left).
    /// Used by HoverVehicleVFX to fire side particle bursts.
    /// </summary>
    public event System.Action<Vector3> OnDodge;

    // -------------------------------------------------------------------------
    // 🎯 Strafe Mode
    // -------------------------------------------------------------------------
    [Header("🎯 Strafe Mode")]
    [Tooltip("Master switch for strafe / aim mode (Left Trigger).")]
    [SerializeField] private bool enableStrafe = true;

    [Tooltip("Maximum speed sustainable in strafe mode. " +
             "Entry speed above this bleeds off naturally via the soft cap. " +
             "Cannot be re-built above this threshold once in strafe.")]
    [Min(1f)]
    [SerializeField] private float strafeTopSpeed = 20f;

    [Tooltip("Acceleration for omni-directional strafe movement on the ground plane. " +
             "Lower than forward accel: strafe is maneuvering, not charging.")]
    [Min(0f)]
    [SerializeField] private float strafeAccel = 15f;

    [Tooltip("Maximum nose tilt (up or down) in strafe mode. " +
             "Applied via torque; Foundation's leveling fights it, creating natural resistance.")]
    [Range(5f, 45f)]
    [SerializeField] private float strafePitchLimit = 15f;

    [Tooltip("Strength of the torque that drives the nose toward the target pitch in strafe mode. " +
             "Keep LOW: Foundation's Leveling Torque Strength fights it. Above 5 will oscillate. Try 2 to 5.")]
    [Range(0.5f, 15f)]
    [SerializeField] private float strafePitchTorque = 3f;

    [Tooltip("Damping on pitch angular velocity in strafe mode. " +
             "Critical for preventing oscillation. Pair with Strafe Pitch Torque. Try 4 to 8.")]
    [Range(0f, 20f)]
    [SerializeField] private float strafePitchDamping = 6f;

    [Tooltip("Aim sensitivity in degrees per second at full stick deflection. " +
             "Controls how fast the pitch angle changes with stick input. Try 60 to 120.")]
    [Range(10f, 300f)]
    [SerializeField] private float strafePitchSensitivity = 90f;

    [Tooltip("How long it takes strafe mode to ramp in or out on trigger press / release.")]
    [Min(0.05f)]
    [SerializeField] private float strafeModeBlendSeconds = 0.2f;

    [Tooltip("Resistance per unit of excess lateral speed in strafe. " +
             "Caps lateral velocity at Strafe Top Speed without a hard clamp. " +
             "Only acts on the lateral axis. Try 20 to 60.")]
    [Range(0f, 120f)]
    [SerializeField] private float strafeLateralCapStrength = 40f;

    private float _strafeModeBlend; // 0..1, blends strafe movement authority in/out
    private float _strafePitchAccum; // accumulated FPS-style pitch angle (degrees)

    /// <summary>
    /// Maximum nose pitch (degrees) in strafe mode.
    /// Read by HoverController_Aim to keep weapon aim range consistent with vehicle pitch range.
    /// </summary>
    public float StrafePitchLimit => strafePitchLimit;

    /// <summary>
    /// Current strafe blend weight (0 = drive mode, 1 = full strafe).
    /// Read by HoverController_Aim to scale aim pitch in sync with strafe entry/exit.
    /// </summary>
    public float StrafeModeBlend => _strafeModeBlend;

    // -------------------------------------------------------------------------
    // 🕹 Input
    // -------------------------------------------------------------------------
    // Acquired via GetComponent<IHoverInputProvider>() in Awake.
    // Attach PlayerHoverInput (or any AI implementation) to this same GameObject.
    // Swapping the component swaps who drives the vehicle, no other wiring needed.

    // -------------------------------------------------------------------------
    // 🧭 Debug
    // -------------------------------------------------------------------------
    [Header("🧭 Debug")]
    [SerializeField] private bool drawDebug = false;

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug.")]
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

    // Cached once per FixedUpdate, read by multiple methods below.
    // Avoids redundant rb.linearVelocity reads and InverseTransformDirection calls.
    private Vector3 _cachedLocalVel;

    private void FixedUpdate()
    {
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
        // Chassis bank is visual only. Runs in Update for smooth interpolation
        // independent of the physics timestep.
        ApplyChassisBank();
    }

    // -------------------------------------------------------------------------
    // ⚡ Boost blend (energy-gated)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Boost advances boostLerp toward 1 only when:
    ///   1. enableBoost is true.
    ///   2. The player is holding boost input.
    ///   3. Energy can cover the cost this frame (TryConsume succeeds).
    ///
    /// If energy is depleted or EMP-frozen mid-boost, target drops to 0 and
    /// boostLerp fades out over boostBlendSeconds. The fade preserves smooth feel
    /// even on a hard energy cutoff.
    /// </summary>
    private void ApplyBoostBlend()
    {
        // Drive mode: forward OR backward throttle enables continuous boost.
        // Strafe mode: only forward-dominant stick enables continuous boost.
        // Lateral or backward routes to dodge burst instead. Forward-dominant
        // means forward exceeds lateral magnitude, preventing stick bleed from
        // a sideways push from triggering continuous boost.
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
    // 💨 Dodge (strafe-mode burst on boost press)
    // -------------------------------------------------------------------------
    /// <summary>
    /// In strafe mode, pressing boost without forward throttle triggers a dodge:
    /// a short tapering force in the stick direction (left, right, or back).
    ///
    /// Force is front-loaded and tapers to zero over dodgeDuration so it reads as
    /// a jet burst rather than an instant velocity snap. Same pattern as Foundation's
    /// unstick lift. Mass independent.
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

        if (dodgeCooldownTimer > 0f)
            dodgeCooldownTimer = Mathf.Max(0f, dodgeCooldownTimer - Time.fixedDeltaTime);

        bool boostPressed = input.Boost && !boostHeldLastFrame;
        boostHeldLastFrame = input.Boost;

        if (!boostPressed || dodgeCooldownTimer > 0f)
            return;

        // Only dodge when stick is NOT in forward-dominant territory.
        // Mirrors the check in ApplyBoostBlend: forward-dominant means continuous
        // boost, everything else means dodge.
        if (input.ThrottleInput >= 0.15f && input.ThrottleInput > Mathf.Abs(input.StrafeX))
            return;

        // Build dodge direction from stick input. Lateral and backward only.
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

        OnDodge?.Invoke(localDir);

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, dodgeForceDir * 3f, Color.blue, 0.5f);
    }

    // -------------------------------------------------------------------------
    // 💨 Sustained Dodge Force
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies the dodge force over a short window each FixedUpdate.
    /// Front-loaded: strongest on the first frame, tapering to zero.
    /// Reads as a jet burst rather than an instant teleport. Mass independent.
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
    ///   3. Chassis is grounded.
    ///   4. Forward speed >= minDriftSpeed at the moment of initiation (entry gate only).
    ///
    /// Once initiated, speed is not re-checked. The drift sustains until the button
    /// releases or turn input drops below threshold. Scrubbing speed through a corner
    /// does not eject the player from drift mid-arc.
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
            // Already drifting. Sustain without re-checking speed.
            _isDrifting = baseCondition;
        }
        else
        {
            // Not yet drifting. Require minimum forward speed to initiate.
            _isDrifting = baseCondition && _cachedLocalVel.z >= minDriftSpeed;
        }

        float target = _isDrifting ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, driftBlendSeconds);
        driftLerp    = Mathf.MoveTowards(driftLerp, target, step);
    }

    // -------------------------------------------------------------------------
    // 🦘 Jump (charge-based grounded + fixed air jump, both energy-gated)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Grounded jump:
    ///   Hold jump button: jumpChargeTimer accumulates, capped at jumpMaxChargeTime.
    ///   Release (jumpHeld false AND chargeTimer > 0): fire.
    ///   Release detection is stateless. No dependency on jumpHeldLastFrame.
    ///   Avoids the FixedUpdate / Update timing mismatch that breaks edge detection.
    ///
    /// Air jump:
    ///   Token granted on airborne to grounded transition. wasGroundedLastFrame is
    ///   initialized true in Awake to avoid a false grant on frame 0.
    ///   Fires on button press while airborne.
    ///   Token NOT consumed on energy denial: player can retry when reserves recover.
    ///
    /// Physics note:
    ///   VelocityChange adds m/s directly, ignoring Rigidbody mass. Guarantees
    ///   identical jump height across all vehicles regardless of mass. This is the
    ///   only direct velocity modification in Propulsion and is intentional.
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

        // ── Air jump token: grant on airborne to grounded transition only ────
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
                // Button released AND charge has built. Fire.
                // Stateless: no edge detection, no last-frame dependency.
                // Correct regardless of FixedUpdate / Update timing.
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
    /// Charge resets regardless of energy outcome: the release motion happened and
    /// the charge is spent whether the jump fired or not. No energy = no jump,
    /// even with a full charge held.
    /// Lockout timer starts on successful fire only.
    /// </summary>
    private void FireGroundedJump()
    {
        float chargeT = Mathf.Clamp01(jumpChargeTimer / jumpMaxChargeTime);
        float impulse = Mathf.Lerp(jumpImpulseMin, jumpImpulseMax, chargeT);

        // Reset charge unconditionally. The player released the button.
        jumpChargeTimer = 0f;

        if (!energy.TryConsume(jumpGroundedEnergyCost))
        {
            OnJumpDenied?.Invoke(true);
            return;
        }

        // VelocityChange: adds m/s directly, ignoring mass.
        rb.AddForce(Vector3.up * impulse, ForceMode.VelocityChange);
        jumpLockoutTimer = jumpGroundedLockout;

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, Vector3.up * impulse * 0.5f, Color.cyan, 0.5f);
    }

    /// <summary>
    /// Fires the air jump. Fixed impulse, no charge.
    /// Token is NOT consumed on energy denial: player retains it for retry.
    /// </summary>
    private void FireAirJump()
    {
        if (!energy.TryConsume(jumpAirEnergyCost))
        {
            OnJumpDenied?.Invoke(false);
            return;
        }

        airJumpAvailable = false;

        // VelocityChange: adds m/s directly, ignoring mass.
        rb.AddForce(Vector3.up * airJumpImpulse, ForceMode.VelocityChange);

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, Vector3.up * airJumpImpulse * 0.5f, Color.magenta, 0.5f);
    }

    // -------------------------------------------------------------------------
    // Drive (unified throttle force)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Throttle-exclusive force model. Drive and forward drag are mutually exclusive:
    ///   Throttle held: drive force applied, forward drag suppressed (see ApplyDrag).
    ///   Throttle released: drive suppressed, forward drag applied.
    ///   Never both at once. Eliminates the opposing-force oscillation that causes
    ///   jitter at any speed between zero and top speed.
    ///
    /// Top speed enforcement: drive is suppressed when forward speed already meets
    /// or exceeds effectiveTopSpeed. Drag is no longer needed to hold the ceiling;
    /// the drive simply stops pushing.
    ///
    /// Airborne: throttle suppressed without boost. Boost re-enables drive scaled by
    /// boost multipliers. Exit velocity carries cleanly as inertia.
    /// </summary>
    private void ApplyDrive(bool grounded, float effectiveTopSpeed, float effectiveForwardAccel)
    {
        bool boosting = boostLerp > 0f;

        if (!grounded && !boosting)
            return;

        float throttle   = Mathf.Clamp(input.ThrottleInput, -1f, 1f);
        float currentFwd = _cachedLocalVel.z;

        // No throttle input. Forward drag handles deceleration; nothing to do here.
        if (Mathf.Abs(throttle) < 0.001f)
            return;

        // Blend forward cap and accel toward strafe values in strafe mode.
        // Strafe top speed is boost-scaled the same way ApplyStrafe scales lateral
        // top speed, so boost still affects strafe mode (relative to the strafe ceiling).
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
                // reverseTopSpeed is independent of strafe mode by design (reverse
                // and strafe share the same speed budget), so no blending here.
                if (currentFwd > -reverseTopSpeed)
                    rawAccel = throttle * maxReverseAccel;
            }
        }
        else
        {
            // Airborne with boost. Forward only, capped at top speed.
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
    // Turning (yaw torque + counter-torque damping)
    // -------------------------------------------------------------------------
    private void ApplyTurning(bool grounded)
    {
        float turn      = Mathf.Clamp(input.TurnInput, -1f, 1f);
        float turnScale = grounded ? 1f : airTurnMultiplier;

        // Effective yaw multiplier inlined: lerp between 1 and driftYawMultiplier.
        float effectiveYawMult = Mathf.Lerp(1f, driftYawMultiplier, driftLerp);

        float inertiaY      = Mathf.Max(0.001f, rb.inertiaTensor.y);
        float desiredYawAcc = turn * yawAccel * turnScale * effectiveYawMult;

        float localYawRate  = transform.InverseTransformDirection(rb.angularVelocity).y;
        float dampingTorque = yawDamping > 0f ? -localYawRate * yawDamping : 0f;

        rb.AddRelativeTorque(Vector3.up * ((desiredYawAcc + dampingTorque) * inertiaY), ForceMode.Force);
    }

    // -------------------------------------------------------------------------
    // Drag (grounded only, both axes)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Lateral drag: always applies grounded. Shapes heading tracking and strafe
    /// feel regardless of throttle.
    ///
    /// Forward drag: mutually exclusive with drive. Only applies near-zero throttle
    /// (coasting or braking). When throttle is held, drive handles the force and
    /// forward drag is suppressed entirely. Eliminates the opposing-force
    /// oscillation that causes jitter at any speed between zero and top speed.
    ///
    /// Drift state reduces both independently via driftLateralDamp and driftForwardDamp.
    /// </summary>
    private void ApplyDrag()
    {
        if (!foundation.IsHoverGrounded)
            return;

        // Lateral drag: always active grounded, independent of throttle.
        float effectiveLateralDamp = Mathf.Lerp(lateralDamp, driftLateralDamp, driftLerp);
        if (effectiveLateralDamp > 0f)
            rb.AddForce(transform.right * (-_cachedLocalVel.x * effectiveLateralDamp), ForceMode.Acceleration);

        // Forward drag: fades in as throttle approaches zero.
        // Full drag at throttle == 0, zero drag at throttle >= 0.15.
        // Prevents the binary snap between full drag and zero drag that feels
        // twitchy on worn sticks or light inputs.
        // Exception: always apply full forward drag during drift. The drive-drag
        // mutual exclusion assumes heading == velocity, which breaks during drift.
        // Drive force along the yawing heading with no forward drag to resist
        // destabilizes the chassis. Drift's own driftForwardDamp handles the
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
    // Over-speed bleed (velocity-aligned deceleration)
    // -------------------------------------------------------------------------
    /// <summary>
    /// When speed exceeds effectiveTopSpeed (e.g. boost fading while throttle is
    /// held), the drive-drag mutual exclusion leaves no force to decelerate.
    /// Drive is suppressed (above cap) and forward drag is suppressed (throttle held).
    ///
    /// This applies a proportional counter-force along the actual world velocity
    /// direction. NOT along transform.forward. Critical because during drift,
    /// heading diverges from velocity. A heading-aligned force would push sideways
    /// or vertically, destabilizing the chassis.
    ///
    /// Velocity-aligned means it always decelerates in the direction the chassis is
    /// actually moving, regardless of heading orientation. Safe during drift, on
    /// slopes, at any heading angle.
    /// </summary>
    private void ApplyOverSpeedBleed(float effectiveTopSpeed)
    {
        // Suppressed during drift. Drift has its own damping and lateral velocity
        // inflates total magnitude, causing false triggers.
        if (driftLerp > 0f)
            return;

        // Use forward-axis speed only. Total magnitude includes lateral, which is
        // irrelevant to the forward top-speed cap.
        float forwardSpeed = _cachedLocalVel.z;

        if (forwardSpeed > effectiveTopSpeed)
        {
            float excess = forwardSpeed - effectiveTopSpeed;
            rb.AddForce(-transform.forward * excess * forwardDamp, ForceMode.Acceleration);
        }
        else if (forwardSpeed < -reverseTopSpeed)
        {
            // Mirror bleed for reverse. Catches dodge burst overshoot and any other
            // impulse that pushes past reverseTopSpeed backward.
            float excess = -reverseTopSpeed - forwardSpeed;
            rb.AddForce(transform.forward * excess * forwardDamp, ForceMode.Acceleration);
        }
    }

    // -------------------------------------------------------------------------
    // Chassis bank (visual only, runs in Update)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Two additive bank contributions:
    ///   Passive bank: proportional to turn input magnitude, always active.
    ///                 Subtle carving look on any hard turn.
    ///   Drift bank:   proportional to driftLerp, only during drift state.
    ///                 Exaggerated lean that reads at speed.
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
    /// Smoothly blends strafe authority in and out on trigger hold / release.
    /// _strafeModeBlend is the weight applied to strafe-specific forces.
    /// ApplyDrive blends its top speed and accel caps toward strafe values as this
    /// rises, so forward drive weakens proportionally on strafe entry.
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
    // 🎯 Omni-Directional Lateral Movement (strafe mode)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Left Stick X drives lateral movement only while strafe mode is active.
    /// Left Stick Y (ThrottleInput) handles forward / back in both modes via ApplyDrive.
    ///
    /// Lateral force scales with _strafeModeBlend so entry / exit is smooth.
    /// In drive mode (blend == 0), lateral movement is fully suppressed: steering
    /// is yaw-only. Lateral damp in ApplyDrag handles residual slide.
    ///
    /// The per-axis speed cap only kicks in when a single local axis exceeds
    /// strafeTopSpeed. Forward and lateral are capped independently so entry
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

        // Per-axis lateral speed cap. Only resists if lateral velocity alone
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
    // 🎯 Strafe Pitch (nose tilt up / down via right stick Y)
    // -------------------------------------------------------------------------
    /// <summary>
    /// In strafe mode, right stick Y (CameraLookY) tilts the nose up or down within
    /// strafePitchLimit. The vehicle body IS the turret. Weapons fire along forward.
    ///
    /// FPS-style: stick input accumulates as a continuous pitch angle (like mouse
    /// look). Releasing the stick holds the current pitch, no rubber-banding.
    /// The accumulated angle is clamped to [-strafePitchLimit, +strafePitchLimit].
    /// Resets to 0 on strafe exit so Foundation leveling can take over cleanly.
    ///
    /// Implementation: proportional torque drives toward the accumulated target;
    /// a separate damping term kills pitch angular velocity to prevent oscillation.
    /// Foundation's leveling torque is the opposing force.
    ///
    /// Tuning: strafePitchTorque must stay LOW relative to Foundation's
    /// levelingTorqueStrength. They're competing forces. strafePitchDamping is the
    /// primary oscillation killer. Start with torque=3, damping=6.
    /// </summary>
    private void ApplyStrafePitch()
    {
        if (_strafeModeBlend <= 0f)
        {
            _strafePitchAccum = 0f;
            return;
        }

        float aimY = input.CameraLookY;

        // Small deadzone to prevent jitter at stick center.
        if (Mathf.Abs(aimY) < 0.1f)
            aimY = 0f;

        // Accumulate stick input as delta. FPS-style.
        // Stick up (+1) = nose up = negative pitch convention.
        _strafePitchAccum -= aimY * strafePitchSensitivity * Time.fixedDeltaTime;
        _strafePitchAccum  = Mathf.Clamp(_strafePitchAccum, -strafePitchLimit, strafePitchLimit);

        float currentPitch = HoverMath.NormalizeAngle(transform.localEulerAngles.x);
        float pitchError   = _strafePitchAccum - currentPitch;

        // Proportional drive toward accumulated target pitch.
        float driveTorque = pitchError * strafePitchTorque * _strafeModeBlend;

        // Damping: counter-torque opposing current pitch angular velocity.
        float localPitchRate = transform.InverseTransformDirection(rb.angularVelocity).x;
        float dampTorque     = -localPitchRate * strafePitchDamping * _strafeModeBlend;

        rb.AddRelativeTorque(Vector3.right * (driveTorque + dampTorque), ForceMode.Acceleration);
    }

}
