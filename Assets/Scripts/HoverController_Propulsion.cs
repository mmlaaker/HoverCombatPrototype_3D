using UnityEngine;

/// <summary>
/// HoverController_Propulsion v1.3
///
/// v1.3: Downed lockout. Jump (grounded and air), all commanded torque (yaw input,
///       air control) and all commanded thrust (ApplyDrive, ApplyStrafe, boost
///       engagement) are suppressed while Foundation.IsDowned. Thrust is included
///       because drive is a FORCE: it escaped a torque-only lockout and kept the
///       downed chassis moving above flipRecoverySpeedThreshold, resetting the
///       arming clock and more than doubling recovery time under button-mashing.
///       Air control was gated on !IsHoverGrounded, which is not the same as
///       airborne -- on its flank the craft's sensor rays find nothing, so it read
///       as airborne while lying on the floor and handed back full roll authority
///       to lever itself upright. Yaw DAMPING is deliberately still applied while
///       downed; it is a stabilizer, not agency.
///
/// v1.2: Strafe forward dead band fixed (ApplyOverSpeedBleed now bleeds against the
///       same strafe-blended cap ApplyDrive clamps to, closing a band where nothing
///       acted on the chassis at all). Boost works in reverse: the gate always
///       accepted reverse throttle and drained energy for no effect, so reverse
///       accel and cap are now scaled by the same boost ratios as the forward path.
///
/// v1.1: Airborne air control. While airborne with drift held, left stick Y = pitch
///       (up = nose down, arcade car convention) and left stick X = roll; right
///       stick X stays yaw via the existing airborne turning path. Intent is
///       handed to Foundation via SetAirControl (no torque here, same pattern as
///       strafe pitch). Strafe-aim wins airborne: effective weight is scaled by
///       (1 - strafe blend). Reuses the Drift bool, which is free airborne.
///
/// Drives the chassis forward, back, sideways, and up. Owns turning, drag, drift,
/// boost, dodge, jump, and strafe-mode authority.
///
/// Tuning lives on a VehicleTuningProfile asset (profile.propulsion). Scene refs
/// (meshRoot) and runtime state (timers, lerps) stay on the component.
///
/// Key design choices baked in:
///
///   Drive and forward drag are mutually exclusive. Holding throttle applies drive
///   and suppresses forward drag. Releasing throttle applies drag and suppresses
///   drive. Never both at once. This is the only reliable cure for jitter caused
///   by opposing forces fighting at the same speed.
///
///   Top speed is enforced inside ApplyDrive. Full accel until the tick that
///   crosses the cap, which is clamped to land exactly on it (no overshoot
///   ripple). At the cap, drive simply stops pushing; no drag wall is needed
///   to hold the ceiling.
///
///   Drift modifies three physics values (lateralDamp, forwardDamp, yawAccel) and
///   one transform (meshRoot Z rotation). It does not introduce new forces. Note
///   that the meshRoot rotation is NOT purely visual -- meshRoot owns the mesh
///   colliders. See ApplyChassisBank for what that does and does not cost.
///
///   Strafe mode blends in lateral authority and a free-aim pitch target. The
///   pitch target is handed to Foundation (SetAimPitch), whose leveling torque is
///   the single attitude authority; Propulsion applies no pitch torque itself.
///   Forward top speed and accel blend toward strafe values as the blend rises.
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
    // 📦 Tuning Profile
    // -------------------------------------------------------------------------
    [Header("📦 Tuning")]
    [Tooltip("Vehicle tuning profile (shared SO). All numeric tuning lives here. Required.")]
    [SerializeField] private VehicleTuningProfile profile;

    /// <summary>Shorthand for profile.propulsion. Used at every read site below.</summary>
    private PropulsionTuning P => profile.propulsion;

    // -------------------------------------------------------------------------
    // ⚡ Boost runtime state
    // -------------------------------------------------------------------------
    private float boostLerp; // 0..1, managed by ApplyBoostBlend

    // -------------------------------------------------------------------------
    // 💨 Dodge runtime state
    // -------------------------------------------------------------------------
    private float   dodgeCooldownTimer;
    private float   dodgeForceTimer;   // counts down while dodge force is being applied
    private Vector3 dodgeForceDir;     // world-space direction cached at trigger time
    private bool    boostHeldLastFrame; // for rising-edge detection on boost button

    // -------------------------------------------------------------------------
    // 🦘 Jump runtime state
    // -------------------------------------------------------------------------

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
    // 🌀 Drift scene refs + runtime state
    // -------------------------------------------------------------------------
    [Header("🌀 Drift")]
    [Tooltip("Mesh parent (HoverCar). Rotated on local Z for the chassis bank visual. " +
             "Assign the HoverCar object in the inspector, not the root or individual meshes.")]
    [SerializeField] private Transform meshRoot;

    private float driftLerp;   // 0..1, managed by ApplyDriftBlend
    private bool  _isDrifting; // entry-gate state: true once speed+turn initiated drift

    // -------------------------------------------------------------------------
    // 🎯 Strafe runtime state
    // -------------------------------------------------------------------------
    private float _strafeModeBlend; // 0..1, blends strafe movement authority in/out
    private float _strafePitchAccum; // accumulated FPS-style pitch angle (degrees)

    // -------------------------------------------------------------------------
    // 🛩 Air control runtime state
    // -------------------------------------------------------------------------
    private float _airControlBlend;  // 0..1, managed by ApplyAirControl
    private float _airControlWeight; // effective authority after strafe suppression

    // -------------------------------------------------------------------------
    // Public read-only state
    // -------------------------------------------------------------------------

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

    /// <summary>
    /// Maximum nose pitch (degrees) in strafe mode.
    /// Currently has no callers. HoverController_Aim was expected to read this but
    /// does not need to: it derives aim from the vehicle's actual settled euler
    /// angles, which are already clamped by this limit upstream in ApplyStrafePitch.
    /// Kept as a read-only accessor for HUD/reticle work that wants the range.
    /// </summary>
    public float StrafePitchLimit => profile.propulsion.strafePitchLimit;

    /// <summary>
    /// Current strafe blend weight (0 = drive mode, 1 = full strafe).
    /// Read by HoverController_Aim to scale aim pitch in sync with strafe entry/exit.
    /// </summary>
    public float StrafeModeBlend => _strafeModeBlend;

    /// <summary>
    /// Current air-control authority (0 = none, 1 = full pitch/roll authority).
    /// Already suppressed by strafe mode. For future camera/VFX/HUD hooks.
    /// </summary>
    public float AirControlWeight => _airControlWeight;

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
    // 🧭 Debug
    // -------------------------------------------------------------------------
    [Header("🧭 Debug")]
    [SerializeField] private bool drawDebug = false;

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.enableDebugGizmos : drawDebug;

    // -------------------------------------------------------------------------
    // 🕹 Input
    // -------------------------------------------------------------------------
    // Acquired via GetComponent<IHoverInputProvider>() in Awake.
    // Attach PlayerHoverInput (or any AI implementation) to this same GameObject.
    // Swapping the component swaps who drives the vehicle, no other wiring needed.

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

        if (profile == null)
        {
            Debug.LogError(
                $"[Propulsion] '{name}': VehicleTuningProfile is not assigned. " +
                $"Assign one in the inspector. Vehicle disabled.",
                this
            );
            enabled = false;
            return;
        }

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

    private void OnEnable()
    {
        if (energy != null)
            energy.OnEmpFreezeApplied += HandleEmpFreeze;
    }

    private void OnDisable()
    {
        if (energy != null)
            energy.OnEmpFreezeApplied -= HandleEmpFreeze;
    }

    // Snap all input-driven transient state to a clean zero on freeze entry.
    // Snap (not fade) because the chassis is about to tumble — smooth blend has no payoff.
    private void HandleEmpFreeze(float _)
    {
        boostLerp          = 0f;
        driftLerp          = 0f;
        _strafeModeBlend   = 0f;
        _strafePitchAccum  = 0f;
        _isDrifting        = false;
        dodgeForceTimer    = 0f;
        jumpChargeTimer    = 0f;
        airJumpAvailable   = false;
        boostHeldLastFrame = false;
        jumpHeldLastFrame  = false;
        _airControlBlend   = 0f;
        _airControlWeight  = 0f;

        // Clear the aim pitch target held by Foundation. Foundation skips all
        // torques during the freeze, but a stale target must not snap the nose
        // when the freeze lifts. Same for the air-control intent.
        foundation.SetAimPitch(0f, 0f);
        foundation.SetAirControl(0f, 0f, 0f);

        if (meshRoot != null)
            meshRoot.localRotation = Quaternion.identity;
    }

    // Cached once per FixedUpdate, read by multiple methods below.
    // Avoids redundant rb.linearVelocity reads and InverseTransformDirection calls.
    private Vector3 _cachedLocalVel;

    private void FixedUpdate()
    {
        // EMP freeze: complete control lockout. Skip every input-driven and
        // damping force. HandleEmpFreeze (subscribed to OnEmpFreezeApplied) has
        // already zeroed transient state; nothing to maintain here. Keep
        // wasGroundedLastFrame pinned false so the air-jump token cannot
        // false-grant via an airborne→grounded transition during the freeze.
        if (energy.IsEmpFrozen)
        {
            wasGroundedLastFrame = false;
            return;
        }

        bool grounded   = foundation.IsHoverGrounded;
        _cachedLocalVel = transform.InverseTransformDirection(rb.linearVelocity);

        // Boost blend runs BEFORE the values derived from it, so everything in this tick
        // reads one consistent boostLerp.
        //
        // It used to run after, which split the tick against itself: the cap and accel
        // below were computed from the PREVIOUS tick's boostLerp, while ApplyDrive's
        // airborne gate (boosting = boostLerp > 0f) read the freshly updated one. For one
        // tick on boost entry the airborne branch therefore let drive through while
        // handing it unboosted numbers, and the mirror happened on release. Ten
        // milliseconds at the shipped 100Hz, so not a feel bug -- but it is exactly the
        // kind of thing that surfaces as an unexplained sample in a tuning measurement.
        ApplyBoostBlend();

        float effectiveTopSpeed     = P.topSpeed        * Mathf.Lerp(1f, P.boostSpeedMultiplier, boostLerp);
        float effectiveForwardAccel = P.maxForwardAccel * Mathf.Lerp(1f, P.boostAccelMultiplier, boostLerp);

        // Note ApplyBoostBlend reads _strafeModeBlend, which ApplyStrafeModeBlend updates
        // further down, so the strafe-mode boost gate still runs a tick behind. Left
        // alone deliberately: hoisting the strafe blend above boost would also move the
        // dodge trigger's view of it, which is a behaviour change rather than a
        // consistency fix. Revisit only with a reason.
        ApplyDriftBlend();
        ApplyStrafeModeBlend();
        ApplyDrive(grounded, effectiveTopSpeed, effectiveForwardAccel);
        ApplyStrafe(grounded, effectiveTopSpeed, effectiveForwardAccel);
        ApplyTurning(grounded);
        ApplyDrag();
        ApplyOverSpeedBleed(effectiveTopSpeed);
        ApplyStrafePitch();
        ApplyAirControl(grounded);
        HandleJump(grounded);
        HandleDodge(grounded);
        ApplyDodgeForce();
    }

    private void Update()
    {
        // EMP freeze: chassis bank is input-driven (turn input). HandleEmpFreeze
        // already snapped meshRoot to identity on freeze entry.
        if (energy.IsEmpFrozen)
            return;

        // Runs in Update for smooth interpolation independent of the physics
        // timestep. Not visual-only; see ApplyChassisBank.
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
        // Downed: refuse to engage at all, so the meter is not drained for thrust
        // that ApplyDrive is going to discard anyway. Same principle as the
        // boost-in-reverse fix: never charge the player for nothing.
        bool wantsBoost  = P.enableBoost && input.Boost && hasThrottle && !foundation.IsDowned;
        bool energyGranted = wantsBoost &&
                             energy.TryConsume(P.boostEnergyPerSecond * Time.fixedDeltaTime);

        float target = energyGranted ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, P.boostBlendSeconds);
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
        if (!P.enableBoost || _strafeModeBlend <= 0f)
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

        if (!energy.TryConsume(P.dodgeEnergyCost))
            return;

        dodgeForceDir   = (transform.right * localDir.x + transform.forward * localDir.z).normalized;
        dodgeForceTimer = P.dodgeDuration;
        dodgeCooldownTimer = P.dodgeCooldown;

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

        float progress  = dodgeForceTimer / Mathf.Max(0.01f, P.dodgeDuration);
        float magnitude = P.dodgeForce * progress;

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
                          && Mathf.Abs(input.TurnInput) >= P.driftTurnThreshold
                          && foundation.IsHoverGrounded;

        if (_isDrifting)
        {
            // Already drifting. Sustain without re-checking speed.
            _isDrifting = baseCondition;
        }
        else
        {
            // Not yet drifting. Require minimum forward speed to initiate.
            _isDrifting = baseCondition && _cachedLocalVel.z >= P.minDriftSpeed;
        }

        float target = _isDrifting ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, P.driftBlendSeconds);
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
        if (!P.enableJump)
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

        // ── Downed lockout ───────────────────────────────────────────────────
        // No jump of any kind while flipped and against the ground. Placed AFTER
        // the token grant and the lockout tick so both keep their bookkeeping,
        // and before either fire path so neither can trigger.
        //
        // The air jump is the load-bearing half: airJumpAvailable survives a
        // flip, and a downed craft is not hover-grounded, so the airborne branch
        // below would happily fire it and pop the player off the ground into
        // air-control range. Charge is cleared so a jump held down through the
        // whole recovery does not fire the instant the craft rights itself.
        if (foundation.IsDowned)
        {
            jumpChargeTimer   = 0f;
            jumpHeldLastFrame = jumpHeld;
            return;
        }

        // ── Grounded jump ────────────────────────────────────────────────────
        if (grounded && jumpLockoutTimer <= 0f)
        {
            if (jumpHeld)
            {
                jumpChargeTimer = Mathf.Min(
                    jumpChargeTimer + Time.fixedDeltaTime,
                    P.jumpMaxChargeTime
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
        float chargeT = Mathf.Clamp01(jumpChargeTimer / P.jumpMaxChargeTime);
        float impulse = Mathf.Lerp(P.jumpImpulseMin, P.jumpImpulseMax, chargeT);

        // Reset charge unconditionally. The player released the button.
        jumpChargeTimer = 0f;

        if (!energy.TryConsume(P.jumpGroundedEnergyCost))
        {
            OnJumpDenied?.Invoke(true);
            return;
        }

        // VelocityChange: adds m/s directly, ignoring mass.
        rb.AddForce(Vector3.up * impulse, ForceMode.VelocityChange);
        jumpLockoutTimer = P.jumpGroundedLockout;

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, Vector3.up * impulse * 0.5f, Color.cyan, 0.5f);
    }

    /// <summary>
    /// Fires the air jump. Fixed impulse, no charge.
    /// Token is NOT consumed on energy denial: player retains it for retry.
    /// </summary>
    private void FireAirJump()
    {
        if (!energy.TryConsume(P.jumpAirEnergyCost))
        {
            OnJumpDenied?.Invoke(false);
            return;
        }

        airJumpAvailable = false;

        // VelocityChange: adds m/s directly, ignoring mass.
        rb.AddForce(Vector3.up * P.airJumpImpulse, ForceMode.VelocityChange);

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, Vector3.up * P.airJumpImpulse * 0.5f, Color.magenta, 0.5f);
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
    /// or exceeds effectiveTopSpeed, and the crossing tick is clamped to land
    /// exactly on the cap (no overshoot ripple). Drag is not needed to hold the
    /// ceiling; the drive simply stops pushing.
    ///
    /// Airborne: throttle suppressed without boost. Boost re-enables drive scaled by
    /// boost multipliers. Exit velocity carries cleanly as inertia.
    /// </summary>
    private void ApplyDrive(bool grounded, float effectiveTopSpeed, float effectiveForwardAccel)
    {
        // Downed: thrusters do nothing while the craft is on its back. Drive is a
        // FORCE, not a torque, so the jump/torque lockout did not cover it and
        // throttle kept shoving the chassis around at whatever angle it was lying
        // at. That held it above flipRecoverySpeedThreshold, which resets the
        // arming clock, so mashing more than doubled the recovery: measured 1.70s
        // untouched (repeatable to 0.01s) versus 3.15-5.70s under panic input.
        // Note the cost only appears with throttle AND boost together, because
        // ApplyBoostBlend needs both, and 1.5x drive is what made it bite.
        if (foundation.IsDowned)
            return;

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
        float strafeEffectiveTopSpeed = P.strafeTopSpeed * (effectiveTopSpeed / P.topSpeed);
        float blendedTopSpeed  = Mathf.Lerp(effectiveTopSpeed,     strafeEffectiveTopSpeed, _strafeModeBlend);
        float blendedFwdAccel  = Mathf.Lerp(effectiveForwardAccel, P.strafeAccel,           _strafeModeBlend);

        // Exact-cap clamp: full accel until the tick that crosses the cap, which
        // is clamped to land exactly ON the cap. Without it, the hard gate
        // overshot by accel * dt (0.5 m/s at default tuning) and the weak
        // over-speed bleed took ~1s to pull it back before drive re-fired: a
        // permanent 0.5 m/s ripple at ~1Hz riding above the cap. Invisible in
        // motion but it flickers any speedometer HUD and pulses anything keyed
        // to "is accelerating" (audio pitch, thruster VFX). Feel-identical below
        // the cap; only the single crossing tick changes.
        float rawAccel = 0f;
        if (grounded)
        {
            if (throttle >= 0f)
            {
                // Suppress drive if already at or above top speed forward.
                if (currentFwd < blendedTopSpeed)
                    rawAccel = Mathf.Min(throttle * blendedFwdAccel,
                                         (blendedTopSpeed - currentFwd) / Time.fixedDeltaTime);
            }
            else
            {
                // Suppress reverse drive if already at or below reverse top speed.
                // reverseTopSpeed is independent of strafe mode by design (reverse
                // and strafe share the same speed budget), so no strafe blending here.
                //
                // Boost DOES apply, though. The boost gate in ApplyBoostBlend accepts
                // reverse throttle (|throttle| >= 0.15), so holding boost in reverse was
                // already draining energy every tick -- it just had no effect here,
                // because this branch read the raw tuning values while only the forward
                // branch used the boost-scaled ones. Paying for nothing is worse than
                // not being able to boost at all, so scale both by the same ratios the
                // forward path uses.
                float boostAccelRatio = effectiveForwardAccel / P.maxForwardAccel;
                float boostSpeedRatio = effectiveTopSpeed     / P.topSpeed;
                float reverseAccel    = P.maxReverseAccel * boostAccelRatio;
                float reverseCap      = P.reverseTopSpeed * boostSpeedRatio;

                if (currentFwd > -reverseCap)
                    rawAccel = Mathf.Max(throttle * reverseAccel,
                                         (-reverseCap - currentFwd) / Time.fixedDeltaTime);
            }
        }
        else
        {
            // Airborne with boost. Forward only, capped exactly at top speed.
            if (currentFwd < blendedTopSpeed)
                rawAccel = Mathf.Min(Mathf.Max(throttle, 0f) * blendedFwdAccel,
                                     (blendedTopSpeed - currentFwd) / Time.fixedDeltaTime);
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
        float turnScale = grounded ? 1f : P.airTurnMultiplier;

        // Downed: no commanded torque until the craft is upright. The yaw DAMPING
        // term below is deliberately left running -- it is a stabilizer, not player
        // agency, and letting a downed chassis keep spinning would only fight the
        // flip recovery's own speed gate and stretch the lockout unpredictably.
        if (foundation.IsDowned)
            turn = 0f;

        // Effective yaw multiplier inlined: lerp between 1 and driftYawMultiplier.
        float effectiveYawMult = Mathf.Lerp(1f, P.driftYawMultiplier, driftLerp);

        float inertiaY      = Mathf.Max(0.001f, rb.inertiaTensor.y);
        float desiredYawAcc = turn * P.yawAccel * turnScale * effectiveYawMult;

        float localYawRate  = transform.InverseTransformDirection(rb.angularVelocity).y;
        float dampingTorque = P.yawDamping > 0f ? -localYawRate * P.yawDamping : 0f;

        rb.AddRelativeTorque(Vector3.up * ((desiredYawAcc + dampingTorque) * inertiaY), ForceMode.Force);
    }

    // -------------------------------------------------------------------------
    // Drag (grounded only, both axes)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Lateral drag: always applies grounded. Shapes heading tracking and kills
    /// unwanted slide. The player's intended strafe velocity is excluded from
    /// damping so drag never fights strafe input; strafeTopSpeed (via the soft
    /// cap in ApplyStrafe) is the real lateral ceiling.
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
        // Damps only UNWANTED lateral velocity: the player's intended strafe
        // velocity (stick * strafeTopSpeed, boost-scaled to mirror ApplyStrafe's
        // effectiveLateralTopSpeed) is excluded. Damping raw velocity silently
        // capped strafe at strafeAccel / lateralDamp (25 m/s at default tuning),
        // below strafeTopSpeed (30), making that knob dead and the soft cap in
        // ApplyStrafe unreachable. With the intended term excluded, lateralDamp
        // purely kills unwanted slide and the soft cap owns the ceiling.
        // In drive mode intended is zero, so drive-mode behavior is unchanged.
        float effectiveLateralDamp = Mathf.Lerp(P.lateralDamp, P.driftLateralDamp, driftLerp);
        if (effectiveLateralDamp > 0f)
        {
            float boostRatio      = Mathf.Lerp(1f, P.boostSpeedMultiplier, boostLerp);
            float intendedLateral = Mathf.Clamp(input.StrafeX, -1f, 1f)
                                  * P.strafeTopSpeed * boostRatio * _strafeModeBlend;
            float unwantedLateral = _cachedLocalVel.x - intendedLateral;

            rb.AddForce(transform.right * (-unwantedLateral * effectiveLateralDamp), ForceMode.Acceleration);
        }

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
            float effectiveForwardDamp = Mathf.Lerp(P.forwardDamp, P.driftForwardDamp, driftLerp);
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

        // Match the cap ApplyDrive actually clamps against. Drive stops pushing at the
        // strafe-blended top speed, so bleeding against the unblended one left a band
        // (strafeTopSpeed..topSpeed) where drive was suppressed for being over cap, drag
        // was suppressed for held throttle, and the bleed had not engaged yet -- nothing
        // acted on the chassis and it coasted there indefinitely. Same expression as
        // ApplyDrive so the two can never disagree again.
        float strafeEffectiveTopSpeed = P.strafeTopSpeed * (effectiveTopSpeed / P.topSpeed);
        float blendedTopSpeed = Mathf.Lerp(effectiveTopSpeed, strafeEffectiveTopSpeed, _strafeModeBlend);

        // Use forward-axis speed only. Total magnitude includes lateral, which is
        // irrelevant to the forward top-speed cap.
        float forwardSpeed = _cachedLocalVel.z;

        if (forwardSpeed > blendedTopSpeed)
        {
            float excess = forwardSpeed - blendedTopSpeed;
            rb.AddForce(-transform.forward * excess * P.forwardDamp, ForceMode.Acceleration);
        }
        else
        {
            // Mirror bleed for reverse. Catches dodge burst overshoot and any other
            // impulse that pushes past the reverse cap backward.
            //
            // Boost-scaled to match the cap ApplyDrive clamps reverse against, for the
            // same reason as the forward axis above: bleeding at the unboosted cap while
            // drive pushes to the boosted one puts the two in direct opposition.
            float reverseCap = P.reverseTopSpeed * (effectiveTopSpeed / P.topSpeed);

            if (forwardSpeed < -reverseCap)
            {
                float excess = -reverseCap - forwardSpeed;
                rb.AddForce(transform.forward * excess * P.forwardDamp, ForceMode.Acceleration);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Chassis bank (runs in Update)
    // -------------------------------------------------------------------------
    /// <summary>
    /// NOT visual-only, despite what this was called for a long time. meshRoot is
    /// 3D/HoverCar, which owns all five mesh colliders, so banking rotates the
    /// collision hull relative to a Rigidbody and a set of hover rays that do not
    /// move with it. Measured consequences at the full -17 degree bank:
    ///
    ///   inertia magnitudes  unchanged at (3293.6, 3756.1, 1091.0)
    ///   inertia BASIS       rotates the full 17 degrees
    ///   centre of mass      shifts 2.5 cm laterally
    ///   ApplyTurning        a unit yaw command leaks -3.91% into pitch
    ///                       (about 5.5 deg/s of nose drift at full drift yaw)
    ///
    /// That was judged not worth decoupling: for scale, the hull's own principal
    /// axes are 0.81 degrees off its local axes, so a unit yaw command ALREADY
    /// leaks -3.46% into roll at zero bank, and decoupling meshRoot would not
    /// touch that. Both are an order of magnitude below anything that destabilizes
    /// the craft. Revisit if the bank budget grows a lot.
    ///
    /// The one consequence that WAS severe is fixed at its source rather than
    /// here: banking used to swing Rim_FR / Rim_RR into their own hover rays past
    /// ~15.5 degrees, which flipped the craft instantly. ApplyHoverForces now
    /// skips self-hits. See the comment there before changing bank angles.
    ///
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

        float passiveAngle = -turnSign * P.passiveBankAngle * turnMag;
        float driftAngle   = -turnSign * P.maxBankAngle * driftLerp;
        float targetAngle  = passiveAngle + driftAngle;

        Quaternion targetRot = Quaternion.Euler(0f, 0f, targetAngle);
        meshRoot.localRotation = Quaternion.Lerp(
            meshRoot.localRotation,
            targetRot,
            P.bankLerpSpeed * Time.deltaTime
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
        if (!P.enableStrafe)
        {
            _strafeModeBlend = 0f;
            return;
        }

        float target = input.StrafeHeld ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, P.strafeModeBlendSeconds);
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
    /// Lateral speed model mirrors the forward axis: drive is gated at the
    /// boost-scaled cap (exact-cap clamp), and a gentle over-speed bleed handles
    /// anything above it. Forward and lateral are capped independently so entry
    /// momentum doesn't strangle lateral acceleration. Dodge bursts fired at the
    /// cap intentionally exceed it and glide back down via the bleed: additive
    /// mobility, paid for in energy, decaying on its own.
    /// </summary>
    private void ApplyStrafe(bool grounded, float effectiveTopSpeed, float effectiveForwardAccel)
    {
        // Downed: same reasoning as ApplyDrive. Lateral thrust is a force too.
        if (foundation.IsDowned)
            return;

        if (!grounded || _strafeModeBlend <= 0f)
            return;

        float stickX = Mathf.Clamp(input.StrafeX, -1f, 1f);

        if (Mathf.Abs(stickX) < 0.001f)
            return;

        // Lateral accel and top speed scale with boost the same way forward does.
        // Derives the boost ratio from the forward accel multiplier so strafe
        // acceleration scales identically with boost without a separate parameter.
        float lateralAccel             = P.strafeAccel * (effectiveForwardAccel / P.maxForwardAccel);
        float effectiveLateralTopSpeed = P.strafeTopSpeed * (effectiveTopSpeed / P.topSpeed);

        // Lateral drive: gated at the cap with the same exact-cap clamp as
        // ApplyDrive (the crossing tick lands exactly on the cap). Drive no
        // longer pushes against the soft cap, which lets the cap below be tuned
        // purely as a gentle over-speed bleed instead of a wall strong enough
        // to out-muscle drive.
        float stickSign          = Mathf.Sign(stickX);
        float lateralTowardStick = _cachedLocalVel.x * stickSign;

        if (lateralTowardStick < effectiveLateralTopSpeed)
        {
            float accelMag = Mathf.Min(Mathf.Abs(stickX) * lateralAccel * _strafeModeBlend,
                                       (effectiveLateralTopSpeed - lateralTowardStick) / Time.fixedDeltaTime);
            rb.AddForce(transform.right * (stickSign * accelMag), ForceMode.Acceleration);
        }

        // Lateral over-speed bleed. Anything above the boost-scaled cap (dodge
        // bursts, strafe-entry momentum, boost fade) tapers back down at
        // excess * strafeLateralCapStrength. Deliberately gentle: a dodge fired
        // at the cap must read as an ADDITIVE surge that glides back to the cap
        // (~1s at default tuning), not get crushed. The old value (40) implied
        // a 25ms decay constant and ate the burst in a tenth of a second, which
        // silently gated dodge effectiveness at the strafe ceiling.
        float localLateral = _cachedLocalVel.x;

        if (Mathf.Abs(localLateral) > effectiveLateralTopSpeed)
        {
            float excess = Mathf.Abs(localLateral) - effectiveLateralTopSpeed;
            rb.AddForce(-transform.right * (Mathf.Sign(localLateral) * excess * P.strafeLateralCapStrength),
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
    /// Implementation: this method only accumulates the target angle and hands it
    /// to Foundation via SetAimPitch. Foundation's leveling torque is the single
    /// attitude authority and drives toward the target itself, aim strength on
    /// the pitch axis only (aimPitchTrackingStrength / pitchRollDamping own the
    /// response feel). The previous design applied a pitch torque here that
    /// fought leveling; two competing force systems are a jitter source (see
    /// Architecture Principles) and coupled the tuning of both modules.
    /// </summary>
    private void ApplyStrafePitch()
    {
        if (_strafeModeBlend <= 0f)
        {
            _strafePitchAccum = 0f;
            foundation.SetAimPitch(0f, 0f);
            return;
        }

        float aimY = input.CameraLookY;

        // Small deadzone to prevent jitter at stick center.
        if (Mathf.Abs(aimY) < 0.1f)
            aimY = 0f;

        // Accumulate stick input as delta. FPS-style.
        // Stick up (+1) = nose up = negative pitch convention (Unity euler X).
        _strafePitchAccum -= aimY * P.strafePitchSensitivity * Time.fixedDeltaTime;
        _strafePitchAccum  = Mathf.Clamp(_strafePitchAccum, -P.strafePitchLimit, P.strafePitchLimit);

        // Weight = strafe blend, so aim authority ramps in/out with strafe mode.
        foundation.SetAimPitch(_strafePitchAccum, _strafeModeBlend);
    }

    // -------------------------------------------------------------------------
    // 🛩 Air Control (drift-held pitch/roll while airborne)
    // -------------------------------------------------------------------------
    /// <summary>
    /// While airborne and holding Drift, left stick Y = pitch (up = nose down,
    /// arcade car convention) and left stick X = roll (right = roll right).
    /// Right stick X stays pure yaw via ApplyTurning's airborne path, so this
    /// completes 3-axis air control without touching turning.
    ///
    /// Intent gating only: the Drift bool is free airborne (ApplyDriftBlend
    /// requires grounded), so no interface changes and no situational gating.
    /// Hops never engage it because drift is a deliberate held input, and a
    /// neutral stick produces zero rotation via the deadzone.
    ///
    /// Pattern mirrors ApplyStrafePitch: this method computes intent and hands
    /// it to Foundation via SetAirControl; Foundation is the single attitude
    /// authority and applies the torque. Strafe-aim wins airborne: effective
    /// weight is scaled by (1 - _strafeModeBlend) so partial blends crossfade
    /// rather than pop.
    /// </summary>
    private void ApplyAirControl(bool grounded)
    {
        // !foundation.IsDowned: "not hover-grounded" is NOT the same as "airborne".
        // A craft on its flank finds no ground with its sensor rays, so it read as
        // airborne while lying on the floor and handed the player full roll
        // authority to lever itself upright against the ground -- faster than the
        // recovery it was supposed to be waiting out. A flip is meant to cost time.
        bool active = P.enableAirControl && input.Drift && !grounded && !foundation.IsDowned;

        float target = active ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, P.airControlBlendSeconds);
        _airControlBlend = Mathf.MoveTowards(_airControlBlend, target, step);

        _airControlWeight = _airControlBlend * (1f - _strafeModeBlend);

        if (_airControlWeight <= 0f)
        {
            foundation.SetAirControl(0f, 0f, 0f);
            return;
        }

        float pitch = input.ThrottleInput;
        float roll  = input.StrafeX;

        // Center deadzone, same 0.1 idiom as ApplyStrafePitch. Load-bearing:
        // the input layer's 0.05 deadzone still passes stick drift that would
        // otherwise produce ~20 deg/s of phantom roll at default tuning.
        if (Mathf.Abs(pitch) < 0.1f) pitch = 0f;
        if (Mathf.Abs(roll)  < 0.1f) roll  = 0f;

        foundation.SetAirControl(pitch, roll, _airControlWeight);
    }

}
