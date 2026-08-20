using UnityEngine;

/// <summary>
/// HoverController_Propulsion v2.2
///
/// v2.2: The drive-mode lane change. TODO 0.36, two testers asking unprompted for left
///       stick X to nudge the craft sideways while driving.
///
///       WHAT IT IS, in the owner's words 2026-08-20: "definitively not a steering
///       mechanism -- a lane change, or a way to make a small adjustment to your line or
///       line up a shot a little better, without being twitchy." Every decision below
///       falls out of that sentence, so read it before changing any of them.
///
///       driveLateralPush is expressed as the sideways speed the craft SETTLES at, and it
///       is applied against the existing lateral drag rather than alongside a new cap.
///       Push and drag balance at exactly the tuned number whatever lateralDamp is, so a
///       later change to the drag cannot silently rescale the knob, and releasing the
///       stick lets the drag put the craft back on its line with nothing to re-centre it.
///
///       SCALED BY FORWARD SPEED, which is the owner's hard constraint and not a polish
///       detail: a fixed sideways speed would be a lean at 80 and a craft crabbing under
///       its own power at a standstill, which is strafing without the trigger. Scaling
///       makes it a constant ANGLE off the line -- 5.7 degrees at the shipped 8 -- so it
///       is the same move at every speed and exactly nothing when stopped. Measured at
///       rest with full stick for 3s: 0.00 m/s and 3cm of drift.
///
///       Measured at 80 m/s: 2.96m sideways in the first second, 9.47m by two, settling
///       at 8.00 m/s. Forward speed held at 80 throughout, so the manoeuvre is free.
///
///       THE KNOB SETS DISTANCE, NOT TIMING, and the two cannot be separated with it.
///       Measured at 4 / 8 / 16, the craft is 64% of the way to its own final speed at
///       one second in ALL THREE cases -- the shape is invariant and only the scale moves.
///       Turning it up therefore does make the first half-second respond harder (0.96 /
///       1.84 / 3.48 m/s at 0.25s) but cannot buy a SMALL lean that arrives QUICKLY. That
///       pairing would need a dedicated response term; lateralDamp is NOT it and the owner
///       has ruled it out, since too much feel is built on that number.
///
///       Air, strafe and drift are all exempt. The airborne one is the item's named trap
///       and is structural rather than a flag: ApplyDrag returns early at zero support, so
///       no lateral force can exist there at all. It also makes the handoff exact, since
///       air-control roll fades IN on (1 - HoverSupport) while this fades OUT on the same
///       number -- a grounded BOOL would switch at a threshold and leave a seam mid-band.
///
/// v2.1: Steering fades with speed. TODO 0.32, two independent testers asking for less
///       direct control the faster they go. There was no speed term in ApplyTurning at
///       all: yaw rate measured 106.7 deg/s at 20 m/s and 106.1 at 40, so a 180 took
///       1.69s at any speed you liked and turn radius simply scaled with velocity.
///
///       WHAT THE MEASUREMENT ACTUALLY SHOWED, and it reframed the fix. Full lock entered
///       at 80 put the craft up to 74 degrees off its own nose and bled 80 m/s down to 45
///       in two seconds. Turning at speed ALREADY cost enormously -- but it was paid after
///       the input, in speed nobody chose to lose, while the stick authority never
///       changed. "Too tight" is that gap. (This is not TuningLog's 0.10 slide, which was
///       steering plus full brake above top speed; that decision stands.)
///
///       yawSpeedFade scales yawAccel and NOT yawDamping, so the nose tops out slower
///       while the turn stays just as crisp to initiate. Shipped shape: full authority to
///       half of top speed, 0.5 at top speed. Measured at 80: yaw 106.8 -> 53.4 deg/s,
///       slip angle 74 -> 53 degrees, and speed carried through the corner 44.6 -> 70.3
///       m/s at the two-second mark. Low speed is untouched to the decimal.
///
///       Read against ACTUAL horizontal speed over the UNBOOSTED topSpeed, which is two
///       deliberate departures from accelCurve. Not the forward axis, because a slip angle
///       collapses it mid-corner and would relax the fade inside the very manoeuvre this
///       governs. Not the boosted cap, because a corner at 100 is harder than one at 80,
///       and dividing by the boosted cap hands authority back exactly when the craft is
///       fastest.
///
///       boostYawMultiplier extends the fade across the band boost adds, because the curve
///       is authored over 0..1 and everything above topSpeed was otherwise FLAT: measured
///       53.3 deg/s at 104 m/s against 53.4 at 80, so the extra 24 m/s cost nothing. Now
///       45.3 at 104. Driven by SPEED, not by boostLerp, so boosting out of a slow corner
///       is free and releasing at 95 does not hand the steering straight back.
///
///       THREE EXEMPTIONS, all verified rather than asserted. Drift, because it only
///       exists above minDriftSpeed and an unexempted fade bites hardest inside it --
///       identical to the failure accelCurve was measured committing in v2.0. Strafe,
///       because right stick X is AIM yaw there. Airborne, already scaled by
///       airTurnMultiplier. Drift re-measured identical to two decimals; strafe confirmed
///       by a matched A/B, 96.6 deg/s in drive against 105.7 in strafe at the same 50 m/s.
///
/// v2.0: Acceleration gets a shape. TODO 0.33, from a playtester who said top speed
///       arrived far too quickly. The reason there was nothing to tune is that there was
///       no curve at all: ApplyDrive applied throttle * blendedFwdAccel as a CONSTANT
///       until the cap cut it off, and forward drag is mutually exclusive with throttle,
///       so nothing opposed it on the way up either. Measured dead straight in all three
///       modes: 0.919s to 80 in drive, 0.828s to 100 boosted, 1.130s to 53 in strafe.
///
///       accelCurve scales that accel by the craft's FRACTION of whichever cap is in
///       force. Normalising against the cap rather than against m/s is what lets one
///       curve cover every mode, and it also closed an asymmetry nobody had noticed:
///       boost multiplies accel by 1.5 but the ceiling by only 1.25, so boost reached its
///       HIGHER top speed in LESS time than the craft reached its ordinary one.
///
///       Shipped shape, measured: 2.35-2.43s to 80 in drive, 2.03s to 100 boosted, 2.87s
///       to 53 in strafe. All three stretched by about 2.5x, so the relationship BETWEEN
///       the modes is unchanged. THE LAUNCH IS UNCHANGED TOO -- 10 m/s arrives at 0.119s
///       before and 0.114-0.119s after -- because the curve is flat at full thrust for
///       the first 15% of the ceiling. What got longer is the tail: 70 to 80 m/s alone
///       now takes about 0.7s of the 2.4.
///
///       DRIFT IS EXEMPT, and the exemption is not optional. A drift's cap FALLS by
///       design, so the fraction pins near 1 and the entire slide runs at the floor
///       multiplier: measured 53.5 m/s against 77.3 m/s at 1.5s into a held drift, which
///       is a held slide quietly converted into a brake. Faded out by driftLerp, and
///       verified to restore the pre-change numbers to within noise. CLAUDE.md fenced
///       drift off from this item in advance and the fence turned out to be load-bearing.
///
///       REVERSE IS EXEMPT TOO, structurally rather than by a flag: the reverse branch
///       builds its accel from maxReverseAccel and never touches blendedFwdAccel.
///       maxReverseAccel doubles as the brake, so shaping it would have softened stopping
///       as a side effect of an acceleration decision. Braking measured at 67.17 m/s^2
///       after the change against a tuning value of 67, which is the proof rather than
///       the claim.
///
/// v1.9: Drift stops being the fastest thing in the game, and starts costing something.
///       One defect and one feature, and the defect is the reason the feature looked
///       impossible to tune.
///
///       DEFECT: both places that enforce top speed compared the FORWARD AXIS against the
///       cap. While heading equals velocity that IS the speed, but a drift separates them
///       by design, and the cap silently became a multiplier: total = cap / cos(angle).
///       Measured on flat ground at full throttle and full lock, a settled drift held
///       128.4 m/s against a topSpeed of 80, in a 337m arc, with drive still at full accel
///       because the forward axis sat at 73 and never reached the gate. Both ApplyDrive and
///       ApplyOverSpeedBleed now compare TOTAL horizontal speed while drifting, and the
///       bleed's force is velocity-aligned there. Fixing only one of the two is not enough:
///       the bleed alone pulled 27 m/s^2 against a drive that never switched off (107 m/s),
///       and the drive gate alone would let a drift coast at its entry speed. Non-drift
///       paths are byte-for-byte the same decision as before and were regression-checked.
///
///       FEATURE: a drift now ENDS when it bleeds out, instead of plateauing at the floor
///       with the bank still on while throttle covers the difference. driftSustainedTopSpeed
///       sits below minDriftSpeed by design, so the old behaviour parked the player in a
///       state they could never have started. _driftSpent latches the exit until the button
///       is released, without which the craft re-enters the moment throttle carries it back
///       over minDriftSpeed and chatters against its own floor. New event: OnDriftSpent.
///
///       FEATURE (TODO M.7a): sustained drifts lose their speed ceiling over time, via
///       DriftSpeedCap and the _driftHeldTime clock. Implemented as a falling CAP rather
///       than as extra drag, because a damping coefficient reaches an equilibrium and holds
///       it forever -- raising it only lowers the plateau, which is a smaller drift rather
///       than a drift that costs anything. Three new tunables: driftSustainSeconds,
///       driftBleedSeconds, driftSustainedTopSpeed.
///
///       Also note driftLateralDamp went 0.3 -> 4 and driftYawMultiplier 1.5 -> 2.5. The
///       old values were not wrong for the craft they were tuned on; they were tuned
///       against the runaway, where every corner was hundreds of metres wide regardless.
///
/// v1.8: Drift becomes a held slide instead of a spin. Two additions and one reframe.
///       maxDriftAngle caps the gap between heading and velocity by fading yaw authority
///       as the slide widens, because drift previously had no equilibrium at all: lateral
///       damping is cut so velocity holds its line, yaw runs at driftYawMultiplier, and
///       nothing restored the angle, so the nose came round for as long as the stick was
///       held. Only the widening direction is limited, so an overcooked entry can always
///       be steered out of. driftHopImpulse adds the entry kick the transition was missing
///       and is deliberately tiny: drift only sustains while grounded, which ends 2.5m up,
///       so a hop past that would cancel the drift it just started. What drift is FOR is
///       now the angle itself, not speed -- weapons fire along chassis forward, so a held
///       slide is the only way to aim off the line of travel at full speed, bought with
///       acceleration rather than the third of top speed strafe mode costs. New event:
///       OnDriftHop.
///
/// v1.7: Air control gated on Foundation.HasAirControlClearance. Closes the case where
///       drifting into a tap jump handed over full attitude authority: the left stick is
///       throttle grounded and pitch airborne, so not letting go of forward commanded a
///       hard nose-down and flipped the craft. Authority now has a height floor; airtime
///       still decides what can be finished above it.
///
/// v1.6: Drag and air control moved onto Foundation.HoverSupport. Both were gated on
///       IsHoverGrounded, which stays true for 2.5m above ride height where the springs
///       have already stopped pushing, so ground drag braked the chassis in mid-air and
///       air control was unavailable for the whole of a tap jump. Both now scale with
///       support (drag by it, air control by its inverse) so they crossfade with
///       Foundation's leveling torque rather than switching. ApplyAirControl no longer
///       takes grounded. Drive, jump charge and drift entry keep IsHoverGrounded: losing
///       throttle on a bump crest would be a worse bug than the one being fixed.
///
/// v1.5: Reverse cap is strafe-blended. New BlendedReverseTopSpeed joins the two
///       existing shared cap helpers, and both reverse read sites (ApplyDrive and
///       ApplyOverSpeedBleed) now call it, because a cap that two methods rebuild
///       separately is exactly what produced the strafe forward dead band and the
///       reverse axis inherited that failure mode the moment its cap stopped being
///       constant. reverseTopSpeed is now the DRIVE-mode ceiling only; strafe mode
///       converges on strafeTopSpeed, so forward, reverse and lateral share one
///       number at full blend. This overturns the previous "reverse is independent
///       of strafe mode by design" note: pinning reverseTopSpeed to strafeTopSpeed
///       did buy omnidirectional symmetry, but it also meant drive-mode reverse
///       could not be raised without breaking that same symmetry. maxReverseAccel
///       is deliberately NOT blended, since it doubles as the brake and blending it
///       would soften braking inside strafe mode as a side effect of a speed
///       decision. ApplyDrag also stopped rebuilding the strafe ceiling by hand and
///       now calls StrafeTopSpeedScaled; the two were algebraically identical, so
///       that part is behaviour-neutral.
///
/// v1.4: Tuning readout. (See CLAUDE.md; this docstring never carried its entry.)
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
///   Top speed is enforced inside ApplyDrive. The tick that crosses the cap is
///   clamped to land exactly on it (no overshoot ripple). At the cap, drive
///   simply stops pushing; no drag wall is needed to hold the ceiling.
///
///   The approach to that cap is SHAPED, and the shape is the only thing that
///   opposes drive on the way up. Forward drag is mutually exclusive with
///   throttle, so nothing else can be doing the work: if the ramp feels wrong,
///   accelCurve is where it is wrong. See AccelCurveMultiplier.
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

    // Seconds of drift held, the input to the sustained-drift speed bleed. Counts UP
    // while drifting and DOWN at the same rate while not, rather than resetting on
    // release: a hard reset would make tapping out and back in a free way to clear the
    // penalty, which is the whole cost of a long drift. Symmetric decay means a long
    // drift has to be paid for with roughly the time it lasted.
    private float _driftHeldTime;

    // Latched true when a drift bleeds out under a still-held button, cleared the moment
    // the button is released. Without it the craft re-enters drift as soon as throttle
    // carries it back over minDriftSpeed and chatters against its own floor.
    private bool _driftSpent;

    // When the entry hop last fired. Timestamp rather than a countdown, per the project
    // convention: a countdown needs somewhere to be decremented and a timestamp just
    // goes stale. NegativeInfinity so the first drift of a session is never blocked.
    private float _lastDriftHopTime = float.NegativeInfinity;

    /// <summary>
    /// How long the entry hop stays disarmed after firing.
    ///
    /// This replaced a HoverSupport &gt; 0.9 test that was doing the same job by proxy
    /// ("wait until the springs are carrying the craft again", which is roughly the hop's
    /// own airtime). The proxy had a measured failure: it also suppressed the hop whenever
    /// the craft happened to be crossing a bump, because support is 0 there for reasons
    /// that have nothing to do with re-firing.
    ///
    /// 0.4s is set against the hop's own flight, not picked round: 10 m/s against 39.24
    /// m/s^2 of grounded gravity apexes at 0.25s, and the springs catch the descent before
    /// it completes. It is also short enough that entering a drift, exiting, and entering
    /// the next corner still earns a second hop, which the old test allowed and a longer
    /// cooldown would take away.
    ///
    /// A const rather than a tuning field on purpose: this guards a mechanism (the entry
    /// gate is a threshold on turn input, so a wobbling stick would otherwise re-fire the
    /// hop every crossing) rather than expressing a feel. Promote it to
    /// VehicleTuningProfile if it ever needs to differ per vehicle.
    /// </summary>
    private const float DriftHopRearmSeconds = 0.4f;

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
    /// Current strafe blend weight (0 = drive mode, 1 = full strafe).
    /// Read by HoverController_Aim to scale aim pitch in sync with strafe entry/exit.
    /// </summary>
    public float StrafeModeBlend => _strafeModeBlend;

    /// <summary>
    /// Current air-control authority (0 = none, 1 = full pitch/roll authority).
    /// Already suppressed by strafe mode. For future camera/VFX/HUD hooks.
    /// </summary>
    public float AirControlWeight => _airControlWeight;

    /// <summary>
    /// The aim pitch the player has ASKED for, in degrees, clamped to strafePitchLimit
    /// and NOT weighted by strafe blend. Instrumentation only.
    ///
    /// This is the command, not the achieved angle. The chassis is a torque servo and
    /// lags it, so the two are supposed to differ; the gap between this and the nose's
    /// measured elevation is exactly what says whether the servo is tracking or whether
    /// something else is moving the nose. Do not use it to size a force: reading the
    /// command instead of the achieved attitude is the mistake `UnaimRotation` documents.
    /// </summary>
    public float CommandedAimPitch => _strafePitchAccum;

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

    /// <summary>
    /// Fired once when a drift starts and the entry hop kicks. For VFX and audio:
    /// the hop is the moment of commitment and is the obvious hook for tyre smoke,
    /// a thruster puff or a chirp. Not fired on ordinary drift exit, deliberately —
    /// letting go of the button is the player's own doing and needs no announcing.
    /// </summary>
    public event System.Action OnDriftHop;

    /// <summary>
    /// Fired once when a drift BLEEDS OUT: the speed ramp has run to completion and the
    /// drift ends itself while the button is still held. This is the bookend to
    /// OnDriftHop, and unlike an ordinary exit it does want announcing, because the
    /// craft is taking control back rather than the player handing it over.
    ///
    /// The chassis bank blending out already signals it, but that reads as the drift
    /// merely weakening. Hook this for the moment it ENDED.
    /// </summary>
    public event System.Action OnDriftSpent;

    // -------------------------------------------------------------------------
    // 🧭 Debug
    // -------------------------------------------------------------------------
    [Header("🧭 Debug")]
    [SerializeField] private bool drawDebug = false;

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.IsEnabled(HoverDebugCategory.Movement) : drawDebug;

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
        _driftHeldTime     = 0f;
        _driftSpent        = false;
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

        // Same reason, and load-bearing here because the freeze returns from
        // FixedUpdate above HandleJump: without this the squat target would be the
        // last one written before the freeze, held for its whole duration, and
        // waiting to pull the craft down the moment the springs come back.
        foundation.SetJumpCharge(0f);

        if (meshRoot != null)
            meshRoot.localRotation = Quaternion.identity;
    }

    // Cached once per FixedUpdate, read by multiple methods below.
    // Avoids redundant rb.linearVelocity reads and InverseTransformDirection calls.
    private Vector3 _cachedLocalVel;

    // -------------------------------------------------------------------------
    // Speed caps (shared so they cannot disagree)
    // -------------------------------------------------------------------------

    /// <summary>
    /// The boost-scaled strafe ceiling. Lateral drive and the lateral bleed both cap against this.
    /// </summary>
    private float StrafeTopSpeedScaled(float effectiveTopSpeed)
        => P.strafeTopSpeed * (effectiveTopSpeed / P.topSpeed);

    /// <summary>
    /// The axis drive and reverse thrust act along, with the commanded AIM pitch
    /// rotated back out, weighted by strafe blend.
    ///
    /// Drive thrust used raw transform.forward. In drive mode that is correct and
    /// this returns it unchanged. In strafe mode it is not, because the chassis is
    /// pitched to AIM rather than to travel, so aiming up commanded a climb and
    /// aiming down commanded a dive, at full drive authority.
    ///
    /// What that cost, from the live values on 2026-08-16:
    ///
    ///   grounded gravity          9.81 * (1 + extraGravityMultiplier 3) = 39.24 m/s^2
    ///   forward thrust in strafe  strafeAccel 47      * sin(36) = 27.6 m/s^2 vertical
    ///   REVERSE thrust in strafe  maxReverseAccel 67  * sin(36) = 39.4 m/s^2 vertical
    ///
    /// Reverse is the binding case, and it is binding because maxReverseAccel is
    /// deliberately NOT strafe-blended (it doubles as the brake). 67 * sin(t) reaches
    /// 39.24 at t = 35.85 degrees, so at strafePitchLimit 36 a nose-down reverse is
    /// already at neutral buoyancy and anything past it flies. That is the value at
    /// which the owner reported functionality breaking, to within 0.15 degrees.
    ///
    /// Below that threshold it does not fly, it OSCILLATES, which is the reported
    /// symptom. The spring clamps to zero above hoverHeight so it cannot pull down;
    /// the craft rises, the spring bottoms out, gravity returns it, the spring
    /// catches it. liftStrength 16 and liftDamping 2.2 give a damping ratio of
    /// 2.2 / (2 * sqrt(16)) = 0.275, underdamped, period ~1.6s, decaying over
    /// roughly a second. "Gentle rubber banding up and down for a bit."
    ///
    /// Removing the COMMANDED aim pitch rather than projecting onto the horizontal
    /// is deliberate: terrain-induced pitch still contributes, so strafing up a ramp
    /// still thrusts along the ramp. Only the part the player aimed with is removed.
    ///
    /// Uses the commanded value rather than the measured chassis attitude because
    /// commanded is what the player asked for; the measured angle also carries the
    /// terrain pitch this is meant to preserve, and reading it back would cancel
    /// that too.
    ///
    /// Known and accepted, same shape as the ApplyBoostBlend note: _strafePitchAccum
    /// is written by ApplyStrafePitch, which runs LATER in the same FixedUpdate, so
    /// this reads last tick's value. At 100Hz and strafePitchSensitivity 150 that is
    /// at most 1.5 degrees of stale aim. Hoisting ApplyStrafePitch would also change
    /// what every other contributor sees, which is a behaviour change rather than a
    /// consistency fix.
    /// </summary>
    private Vector3 ComputeDriveAxis()
    {
        if (_strafeModeBlend <= 0f)
            return transform.forward;

        // _strafePitchAccum is in Unity euler X convention (negative is nose up),
        // so undoing it is a rotation of -accum about the chassis right axis.
        float aimPitch = _strafePitchAccum * _strafeModeBlend;

        return Quaternion.AngleAxis(-aimPitch, transform.right) * transform.forward;
    }

    /// <summary>
    /// This tick's drive axis, written once in FixedUpdate. Read it; do not recompute
    /// it. See ComputeDriveAxis.
    /// </summary>
    private Vector3 _driveAxis;

    /// <summary>
    /// The forward cap drive actually clamps against: boost-scaled top speed, blended toward the
    /// boost-scaled strafe ceiling by strafe mode.
    ///
    /// ApplyDrive and ApplyOverSpeedBleed MUST use the same expression, and this exists so they
    /// cannot drift apart again. When they disagreed there was a band (strafeTopSpeed..topSpeed)
    /// where drive was suppressed for being over cap, forward drag was suppressed for held
    /// throttle, and the bleed had not engaged yet: nothing acted on the chassis and it coasted
    /// there indefinitely. The debug readout reports that state as NO FORCE, which is what makes
    /// a band like it visible instead of feeling like intended glide.
    /// </summary>
    private float BlendedTopSpeed(float effectiveTopSpeed)
        => Mathf.Lerp(effectiveTopSpeed, StrafeTopSpeedScaled(effectiveTopSpeed), _strafeModeBlend);

    /// <summary>
    /// The reverse cap drive actually clamps against: boost-scaled reverse top speed, blended
    /// toward the boost-scaled strafe ceiling by strafe mode.
    ///
    /// Mirrors BlendedTopSpeed on the forward axis and exists for the same reason: ApplyDrive
    /// and ApplyOverSpeedBleed must share ONE expression or they open a band where neither acts
    /// on the chassis.
    ///
    /// This overturns the previous "reverseTopSpeed is independent of strafe mode by design"
    /// decision, which had reverseTopSpeed serving two jobs at once. Reverse and strafe were
    /// pinned to the same number so that strafe mode reads as a consistent omnidirectional
    /// speed, but that also meant drive-mode reverse could not be raised without raising the
    /// strafe ceiling and breaking exactly the symmetry the pin existed to protect. Blending
    /// gives both: reverseTopSpeed is now the DRIVE-mode cap only, and strafe mode converges
    /// on strafeTopSpeed, so forward, reverse and lateral all land on one value at full blend.
    /// </summary>
    private float BlendedReverseTopSpeed(float effectiveTopSpeed)
        => Mathf.Lerp(P.reverseTopSpeed, P.strafeTopSpeed, _strafeModeBlend)
         * (effectiveTopSpeed / P.topSpeed);

    // -------------------------------------------------------------------------
    // 🧭 Debug readout state
    // -------------------------------------------------------------------------
    // Written during FixedUpdate, read by OnDrawGizmos. Plain fields rather than
    // recomputation in the gizmo, for the same reason Foundation caches
    // _effectiveHoverHeight: OnDrawGizmos runs on the editor's schedule, sometimes
    // several times a frame and sometimes not at all, and must never re-derive
    // physics state or mutate anything.
    //
    // Kept as separate flags rather than one enum deliberately. Drive and forward
    // drag are supposed to be MUTUALLY EXCLUSIVE (see ApplyDrag), so seeing both
    // lit at once is a violation of the architecture rule that opposing forces
    // cause jitter, and a single enum would hide exactly that case.
    private bool  _dbgDrive, _dbgReverse, _dbgDrag, _dbgBleed;
    private float _dbgDriveAccel;
    private float _dbgFwdCap, _dbgLatCap;

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

        // Cleared here so each tick's readout reflects only this tick. The caps are
        // recorded after ApplyStrafeModeBlend below, since both depend on it.
        _dbgDrive = _dbgReverse = _dbgDrag = _dbgBleed = false;
        _dbgDriveAccel = 0f;

        // Note ApplyBoostBlend reads _strafeModeBlend, which ApplyStrafeModeBlend updates
        // further down, so the strafe-mode boost gate still runs a tick behind. Left
        // alone deliberately: hoisting the strafe blend above boost would also move the
        // dodge trigger's view of it, which is a behaviour change rather than a
        // consistency fix. Revisit only with a reason.
        ApplyDriftBlend();
        ApplyStrafeModeBlend();

        // Recorded after the strafe blend, before anything reads them. These are the
        // numbers the player is actually driving against, and neither is visible in
        // the inspector because both are derived from boost and strafe state.
        _dbgFwdCap = BlendedTopSpeed(effectiveTopSpeed);
        _dbgLatCap = StrafeTopSpeedScaled(effectiveTopSpeed);

        // Solved once, after the strafe blend it depends on and before the two methods
        // that read it. ApplyDrive and ApplyDrag MUST see the same axis: drag is the
        // force that opposes drive, and a damping force on a different axis than the
        // thrust it opposes is not damping. Caching makes that structural instead of
        // relying on nothing between them happening to change the inputs.
        _driveAxis = ComputeDriveAxis();

        ApplyDrive(grounded, effectiveTopSpeed, effectiveForwardAccel);
        ApplyStrafe(grounded, effectiveTopSpeed, effectiveForwardAccel);
        ApplyTurning(grounded);
        ApplyDrag(effectiveTopSpeed);
        ApplyOverSpeedBleed(effectiveTopSpeed);
        ApplyStrafePitch();
        ApplyAirControl();
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
        bool wasDrifting = _isDrifting;

        // Releasing the button re-arms drift. See _driftSpent below.
        if (!input.Drift)
            _driftSpent = false;

        bool baseCondition = input.Drift
                          && Mathf.Abs(input.TurnInput) >= P.driftTurnThreshold
                          && foundation.IsHoverGrounded;

        if (_isDrifting)
        {
            // Already drifting. Sustain without re-checking speed: scrubbing speed through
            // a corner must not eject the player mid-arc.
            //
            // EXCEPT once the bleed ramp has run to completion, which is the manoeuvre's
            // designed end rather than a situational speed dip. Without this the drift had
            // no terminal state at all: it plateaued at driftSustainedTopSpeed and held the
            // bank forever while throttle covered the floor, leaving the player parked in a
            // state they could never have STARTED, since the floor 45 sits deliberately
            // below minDriftSpeed 53. Ending it hands normal grip and steering back and
            // makes the cost legible.
            //
            // Clock-based rather than speed-based on purpose: driftSustainSeconds +
            // driftBleedSeconds is deterministic and identical every time, where waiting on
            // an actual speed reading would fire early downhill and never fire on a climb.
            bool spent = _driftHeldTime >= P.driftSustainSeconds + P.driftBleedSeconds;
            if (spent)
            {
                _driftSpent = true;
                OnDriftSpent?.Invoke();
            }
            _isDrifting = baseCondition && !spent;
        }
        else
        {
            // Not yet drifting. Require minimum forward speed to initiate, and require the
            // button to have been RELEASED since the last drift bled out — otherwise the
            // craft re-enters the moment throttle carries it back over minDriftSpeed, which
            // takes well under a second, and the drift chatters on and off against its own
            // floor. _driftHeldTime is deliberately NOT reset here: it decays in real time,
            // so drifting again too soon buys a shorter drift. That is what stops release-
            // and-re-press being a free way to dodge the bleed.
            _isDrifting = baseCondition
                       && !_driftSpent
                       && _cachedLocalVel.z >= P.minDriftSpeed;
        }

        // Entry hop. Fires on the rising edge, rate-limited by DriftHopRearmSeconds.
        //
        // TWO THINGS HAPPEN HERE AND THEY ARE GATED DIFFERENTLY ON PURPOSE.
        //
        // The EVENT marks a player action: the drift started. It fires on every
        // rate-limited entry, unconditionally. It used to sit behind the same
        // HoverSupport > 0.9 test as the impulse, and measured 2026-08-17 that dropped
        // roughly one entry in three on real terrain (support at three sampled entries:
        // 0.99, 0.23, 1.00, with 20% of cornering frames below the threshold). The
        // owner's report was that the drift hop is unreliable "when I'm already turning",
        // and this was why: not a weak hop, an absent one. Anything built on this event
        // inherits that, so a cue hung here would have failed in exactly the same third
        // of cases and looked like a broken cue.
        //
        // The IMPULSE is scaled by support rather than gated on it. A 10 m/s
        // VelocityChange applied while the springs are not carrying the craft is a jump,
        // not a hop, so it has to fade out. Scaling rather than switching follows what
        // HoverSupport is for: Foundation crossfades fall gravity, air control, leveling
        // and drag on it precisely so no feature has a cliff, and a boolean here would
        // just move the cliff rather than remove it.
        //
        // No energy cost. This is punctuation, not a mobility tool, and charging for
        // entering a drift would tax the manoeuvre for having a sound.
        if (_isDrifting && !wasDrifting
            && Time.time - _lastDriftHopTime >= DriftHopRearmSeconds)
        {
            _lastDriftHopTime = Time.time;

            float hop = P.driftHopImpulse * foundation.HoverSupport;
            if (hop > 0.001f)
                rb.AddForce(Vector3.up * hop, ForceMode.VelocityChange);

            OnDriftHop?.Invoke();

            if (ShouldDrawDebug)
                Debug.DrawRay(transform.position, Vector3.up * 2f, Color.green, 0.4f);
        }

        float target = _isDrifting ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, P.driftBlendSeconds);
        driftLerp    = Mathf.MoveTowards(driftLerp, target, step);

        // Held-drift clock. Drives DriftSpeedCap below. Counts down when not drifting so
        // the penalty cannot be cleared by releasing and re-pressing.
        _driftHeldTime = Mathf.Max(0f, _driftHeldTime
                                     + (_isDrifting ? Time.fixedDeltaTime : -Time.fixedDeltaTime));
    }

    /// <summary>
    /// The speed ceiling a drift is allowed to hold, which decays the longer it is held.
    /// This is TODO M.7a: "a sustained drift should bleed speed over time, even on full
    /// throttle". Returns the normal cap unchanged for the first driftSustainSeconds, then
    /// ramps to driftSustainedTopSpeed across driftBleedSeconds.
    ///
    /// Implemented as a moving CAP rather than as extra drag, deliberately. driftForwardDamp
    /// is a coefficient, so throttle and drag reach an equilibrium and hold it forever;
    /// raising it only lowers the plateau, which is a smaller drift, not a drift that costs
    /// something. A falling cap is the only form that keeps taking speed on full throttle,
    /// and it composes with the two places the cap is already enforced instead of adding a
    /// third force to balance against them.
    ///
    /// Never RAISES the cap: if driftSustainedTopSpeed is somehow above the current ceiling
    /// (a low boost-blended cap, say), the ceiling wins.
    /// </summary>
    private float DriftSpeedCap(float blendedTopSpeed)
    {
        float overrun = _driftHeldTime - P.driftSustainSeconds;
        if (overrun <= 0f)
            return blendedTopSpeed;

        float k     = Mathf.Clamp01(overrun / Mathf.Max(0.01f, P.driftBleedSeconds));
        float floor = Mathf.Min(P.driftSustainedTopSpeed, blendedTopSpeed);
        return Mathf.Lerp(blendedTopSpeed, floor, k);
    }

    /// <summary>
    /// Signed angle between where the chassis POINTS and where it is MOVING, in degrees.
    /// Positive means velocity is off to the right of the nose.
    ///
    /// This is the drift metric, and it is what the manoeuvre actually produces: weapons
    /// fire along chassis forward, so this gap is the only way to aim off the line of
    /// travel at full speed. Zero below walking pace, because Atan2 on a near-zero
    /// vector reports a confident angle that means nothing.
    /// </summary>
    private float ShoulderAngle =>
        _cachedLocalVel.sqrMagnitude > 1f
            ? Mathf.Atan2(_cachedLocalVel.x, _cachedLocalVel.z) * Mathf.Rad2Deg
            : 0f;

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
        {
            foundation.SetJumpCharge(0f);
            return;
        }

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
            foundation.SetJumpCharge(0f);
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

        // Hand the charge to Foundation so the springs can show it. Reported ONLY for
        // a charge that could actually fire from where the craft is standing, which is
        // the same condition the accumulate branch above uses:
        //
        //   Airborne is excluded even though the timer survives going off a ledge with
        //   the button held. There is nothing to compress against, and a squat target
        //   set mid-air would be waiting to yank the craft down the instant it lands.
        //
        //   The lockout is excluded because a craft that just jumped is not charging
        //   anything, and squatting through the landing of the jump you already made
        //   reads as the craft failing to recover rather than as a charge building.
        //
        // A tap jump reports whatever fraction one tick of hold produced, which rounds
        // to nothing: the squat is a hold made visible, and a tap has no hold to show.
        bool charging = grounded && jumpLockoutTimer <= 0f && P.jumpMaxChargeTime > 0f;
        foundation.SetJumpCharge(charging ? jumpChargeTimer / P.jumpMaxChargeTime : 0f);
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

        // Cost ramps on the SAME chargeT that sets the impulse, so what you pay and
        // what you get can never disagree. A flat price made a tap jump -- used
        // constantly just to clear things on the ground -- cost exactly as much as a
        // 20m launch that buys a whole trick window.
        float cost = Mathf.Lerp(P.jumpGroundedEnergyCost, P.jumpGroundedChargedEnergyCost, chargeT);

        // Reset charge unconditionally. The player released the button.
        jumpChargeTimer = 0f;

        if (!energy.TryConsume(cost))
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

        // Fires along the CRAFT'S OWN UP, not world up, so the air jump is a
        // directional juke rather than only a height extender: tilt toward a wall
        // and it shoves you off it, which is what the owner asked for and what
        // makes this ability worth keeping (TODO 5.9). The grounded jump keeps
        // world up deliberately -- levelling already aligns the craft to the
        // surface, so local and world up agree there, and a craft on a slope
        // should still jump UP rather than off down the hill.
        //
        // Clamped so it can never push DOWNWARD. Tricks mean inversion, and the
        // air jump's job during a trick is to buy hang time; a version that fired
        // the craft groundward whenever it was upside down would be at its most
        // harmful in exactly the situation it exists for. Zeroing the vertical
        // component rather than blending toward world up keeps the full lateral
        // direction at any attitude, so lying on one flank still gives a clean
        // horizontal shove off a wall. Continuous through 90 degrees; the only
        // discontinuity is at EXACTLY inverted, where there is no lateral
        // direction left to keep and world up is the sane fallback.
        Vector3 jumpDir = transform.up;

        if (jumpDir.y < 0f)
        {
            jumpDir.y = 0f;
            jumpDir = jumpDir.sqrMagnitude > 1e-4f ? jumpDir.normalized : Vector3.up;
        }

        // Cancel some of the velocity FIGHTING the jump before adding to it. The
        // impulse adds m/s rather than setting them, so falling at 30 and jumping
        // for 20 leaves the craft still falling at 10 and the button reads as dead,
        // while the identical press at the apex delivers the full 20. How strong
        // this ability feels was therefore a function of WHEN it was pressed.
        //
        // The complaint arrived immediately after M.6 more than doubled
        // extraFallGravity (13 -> 30). Nothing was wrong with the air jump before
        // that; it simply was never re-tuned against a craft that now falls twice
        // as fast, which is the general hazard of judging one tuning change in
        // isolation from its neighbours.
        //
        // Only the OPPOSING component is touched, projected on jumpDir rather than
        // on world up so it stays correct for the tilted and wall-shove cases the
        // clamp above exists to serve. Rise you already had is never removed, so
        // the apex stack that airJumpImpulse's tooltip describes still works.
        float opposing = Vector3.Dot(rb.linearVelocity, jumpDir);

        if (opposing < 0f && P.airJumpFallCancel > 0f)
            rb.AddForce(jumpDir * (-opposing * P.airJumpFallCancel), ForceMode.VelocityChange);

        // VelocityChange: adds m/s directly, ignoring mass.
        rb.AddForce(jumpDir * P.airJumpImpulse, ForceMode.VelocityChange);

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, jumpDir * P.airJumpImpulse * 0.5f, Color.magenta, 0.5f);
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
        float blendedTopSpeed  = BlendedTopSpeed(effectiveTopSpeed);
        float blendedFwdAccel  = Mathf.Lerp(effectiveForwardAccel, P.strafeAccel, _strafeModeBlend);

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
                // Which speed the cap binds on depends on whether heading equals velocity.
                //
                // Drive mode: the forward axis IS the speed, so this is unchanged.
                //
                // Drift: they diverge, and gating on the forward axis alone leaves drive
                // running at FULL accel far above the cap. Measured 2026-08-13: a settled
                // drift held 107 m/s against a topSpeed of 80 with drive still at 87,
                // because the forward axis sat at 73.2 and never reached the gate. Fixing
                // ApplyOverSpeedBleed alone was not enough for exactly this reason — the
                // bleed was pulling 27 m/s^2 against a drive that never switched off.
                // Lerped by driftLerp so entry and exit cannot pop.
                float capReference = currentFwd;
                float effectiveCap = blendedTopSpeed;
                if (driftLerp > 0f)
                {
                    Vector3 driftHorizVel = rb.linearVelocity;
                    driftHorizVel.y = 0f;
                    capReference = Mathf.Lerp(currentFwd, driftHorizVel.magnitude, driftLerp);
                    // Sustained drifts lose their ceiling over time. See DriftSpeedCap.
                    effectiveCap = Mathf.Lerp(blendedTopSpeed, DriftSpeedCap(blendedTopSpeed), driftLerp);
                }

                // Suppress drive if already at or above top speed.
                if (capReference < effectiveCap)
                {
                    // The ramp shape. Fed the SAME pair the cap is enforced on, so a drift
                    // is measured against the drift cap rather than against a forward axis
                    // that has stopped describing its speed -- and then faded straight back
                    // out by driftLerp, because drift is exempt. See AccelCurveMultiplier.
                    float shapedAccel = blendedFwdAccel * AccelCurveMultiplier(capReference, effectiveCap, driftLerp);

                    rawAccel = Mathf.Min(throttle * shapedAccel,
                                         (effectiveCap - capReference) / Time.fixedDeltaTime);
                }
            }
            else
            {
                // Suppress reverse drive if already at or below reverse top speed.
                // The cap is strafe-blended (see BlendedReverseTopSpeed): reverseTopSpeed
                // is the DRIVE-mode ceiling and strafe mode converges on strafeTopSpeed,
                // so all three axes share one value at full strafe blend.
                //
                // Accel is deliberately NOT strafe-blended, unlike the forward axis.
                // maxReverseAccel doubles as the brake, so blending it toward strafeAccel
                // would soften braking inside strafe mode specifically, which is a
                // side effect of a speed decision rather than a chosen one. The cost is
                // that reverse reaches the shared cap faster than forward or lateral do.
                //
                // Boost DOES apply. The boost gate in ApplyBoostBlend accepts reverse
                // throttle (|throttle| >= 0.15), so holding boost in reverse was already
                // draining energy every tick -- it just had no effect here, because this
                // branch read the raw tuning values while only the forward branch used
                // the boost-scaled ones. Paying for nothing is worse than not being able
                // to boost at all, so scale both by the same ratios the forward path uses.
                float boostAccelRatio = effectiveForwardAccel / P.maxForwardAccel;
                float reverseAccel    = P.maxReverseAccel * boostAccelRatio;
                float reverseCap      = BlendedReverseTopSpeed(effectiveTopSpeed);

                if (currentFwd > -reverseCap)
                    rawAccel = Mathf.Max(throttle * reverseAccel,
                                         (-reverseCap - currentFwd) / Time.fixedDeltaTime);
            }
        }
        else
        {
            // Airborne with boost. Forward only, capped exactly at top speed. Shaped by the
            // same curve on purpose: an airborne boost that ignored the ramp would be the
            // fastest way to reach top speed in the game, which is not a thing anyone asked
            // for and would quietly become the optimal opening move.
            if (currentFwd < blendedTopSpeed)
            {
                float shapedAccel = blendedFwdAccel * AccelCurveMultiplier(currentFwd, blendedTopSpeed, 0f);

                rawAccel = Mathf.Min(Mathf.Max(throttle, 0f) * shapedAccel,
                                     (blendedTopSpeed - currentFwd) / Time.fixedDeltaTime);
            }
        }

        if (Mathf.Abs(rawAccel) < 0.001f)
            return;

        _dbgDrive      = rawAccel > 0f;
        _dbgReverse    = rawAccel < 0f;
        _dbgDriveAccel = rawAccel;

        // Not transform.forward: in strafe mode the nose is an AIM direction, not a
        // travel direction. See ComputeDriveAxis.
        rb.AddForce(_driveAxis * rawAccel, ForceMode.Acceleration);

        if (ShouldDrawDebug)
            Debug.DrawRay(transform.position, _driveAxis * rawAccel, Color.yellow);
    }

    /// <summary>
    /// THE ONE PLACE the acceleration ramp is shaped. Returns the fraction of full thrust
    /// available at the craft's current fraction of whichever cap is in force.
    ///
    /// Read against the CAP rather than against metres per second, and that is the whole
    /// reason one curve covers every mode. Boost raises accel by 1.5 and the ceiling by
    /// only 1.25, so before this existed boost reached its HIGHER top speed in LESS time
    /// than the craft reached its ordinary one -- 0.83s against 0.92s, both measured. A
    /// curve read off absolute speed would have preserved that. Read off the fraction,
    /// boost, strafe and drift each get the same shape stretched over their own ceiling.
    ///
    /// Deliberately NOT applied to reverse. maxReverseAccel doubles as the brake, so
    /// shaping it would soften stopping as a side effect of an acceleration decision.
    /// The exclusion is structural rather than a flag: the reverse branch builds its accel
    /// from maxReverseAccel directly and never touches blendedFwdAccel.
    ///
    /// DRIFT IS EXEMPT, faded out by driftBlend, and that is not a special case bolted on
    /// -- it is the fence CLAUDE.md puts around drift for exactly this item. A drift's cap
    /// FALLS by design (DriftSpeedCap), so the fraction sits pinned near 1 and the craft
    /// gets the floor multiplier for the whole slide. Measured 1.5s into a held drift:
    /// 53.5 m/s shaped against 77.3 m/s unshaped. A drift already costs acceleration --
    /// that is what v1.9 decided it is FOR -- so shaping it charges for the same thing
    /// twice and quietly turns a held slide into a brake.
    ///
    /// No straight-line exploit hides in that exemption: drift needs turn input past
    /// driftTurnThreshold and minDriftSpeed 53 to start at all, so it cannot be used to
    /// skip the ramp from a standstill.
    ///
    /// The 0.01 floor here is a safety net, not the tuning. It stops a curve dragged to
    /// zero from making top speed unreachable, which would present as the craft capping
    /// below the number in the inspector for no visible reason. The floor that is MEANT
    /// to be tuned is the right-hand end of the curve itself.
    /// </summary>
    private float AccelCurveMultiplier(float speed, float cap, float driftBlend)
    {
        AnimationCurve curve = P.accelCurve;
        if (curve == null || curve.length == 0)
            return 1f;

        // A negative fraction means the craft is still travelling backwards under forward
        // throttle. Clamp01 hands it full authority there, which is what reversing your
        // direction of travel should feel like.
        float fraction = cap <= 0.01f ? 1f : Mathf.Clamp01(speed / cap);
        float shaped   = Mathf.Max(0.01f, curve.Evaluate(fraction));

        return Mathf.Lerp(shaped, 1f, driftBlend);
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

        // Drift angle ceiling. Without it a drift has no equilibrium: lateral damping is
        // reduced so velocity holds its line, yaw runs at driftYawMultiplier, and nothing
        // stops the nose coming round for as long as the stick is held. That is a spin,
        // not a slide, and it is why the manoeuvre could not be held and aimed from.
        //
        // Authority fades as the slide approaches maxDriftAngle, so it settles at an angle
        // instead of running away. Only the WIDENING direction is limited: yawing back
        // toward the line of travel keeps full authority at any angle, or an overcooked
        // entry would strand the craft sideways with no way out. Sign convention: yawing
        // right moves velocity leftward in the body frame, so turn and shoulder having
        // opposite signs is what "widening" means.
        if (driftLerp > 0f && P.maxDriftAngle > 0f)
        {
            float shoulder = ShoulderAngle;
            bool  widening = turn * shoulder < 0f;

            if (widening)
            {
                float budget = 1f - Mathf.Clamp01(Mathf.Abs(shoulder) / P.maxDriftAngle);
                effectiveYawMult *= Mathf.Lerp(1f, budget, driftLerp);
            }
        }

        float inertiaY      = Mathf.Max(0.001f, rb.inertiaTensor.y);
        float desiredYawAcc = turn * P.yawAccel * turnScale * effectiveYawMult * YawFadeMultiplier(grounded);

        float localYawRate  = transform.InverseTransformDirection(rb.angularVelocity).y;
        float dampingTorque = P.yawDamping > 0f ? -localYawRate * P.yawDamping : 0f;

        rb.AddRelativeTorque(Vector3.up * ((desiredYawAcc + dampingTorque) * inertiaY), ForceMode.Force);
    }

    /// <summary>
    /// THE ONE PLACE steering authority is faded with speed. Returns the fraction of
    /// yawAccel available at the craft's current speed.
    ///
    /// Scales yawAccel and NOT yawDamping, and the difference is the whole feel. Settled
    /// yaw rate is yawAccel/yawDamping and the response time is 1/yawDamping, so scaling
    /// only the first lowers the rate the nose tops out at while leaving the turn just as
    /// crisp to initiate. Scaling both would make it mushy instead of heavy, which is a
    /// different complaint from the one two testers actually reported.
    ///
    /// READ AGAINST ACTUAL SPEED, and deliberately NOT normalised the way accelCurve is.
    /// accelCurve divides by whichever cap is in force, so boost gets the same shape over
    /// its own ceiling -- correct there, because that curve is about approaching a
    /// ceiling. Turning is not: a corner at 100 m/s under boost is harder than one at 80,
    /// not equally hard, so dividing by the boosted cap would hand back authority exactly
    /// when the craft is fastest. Normalised against the UNBOOSTED topSpeed and clamped,
    /// so boosting past it holds the curve's right-hand value.
    ///
    /// Horizontal SPEED, not the forward axis. Hard cornering opens a slip angle of up to
    /// 70 degrees (measured), which collapses the forward axis while the craft is still
    /// travelling just as fast. Reading the forward axis would relax the fade in the
    /// middle of the very manoeuvre it exists to govern.
    ///
    /// THREE EXEMPTIONS, and the craft keeps today's full authority in all of them:
    ///   Drift   -- drift only exists above minDriftSpeed, so an unexempted fade bites
    ///              hardest inside it. That is the same failure accelCurve was measured
    ///              committing, and drift is judged good and explicitly fenced off.
    ///   Strafe  -- right stick X is AIM yaw there, not steering. Fading it degrades
    ///              aiming, which is a separate deferred item.
    ///   Air     -- already scaled by airTurnMultiplier, and airborne yaw belongs to
    ///              tricks and air control rather than to driving.
    /// Blended by MAX rather than by multiplying the two: either one being engaged should
    /// hand back full authority on its own, and multiplying would leave a half-faded band
    /// during a strafing drift where neither exemption is complete.
    /// </summary>
    private float YawFadeMultiplier(bool grounded)
    {
        AnimationCurve curve = P.yawSpeedFade;
        if (!grounded || curve == null || curve.length == 0)
            return 1f;

        Vector3 horizVel = rb.linearVelocity;
        horizVel.y = 0f;

        float fraction = horizVel.magnitude / Mathf.Max(0.01f, P.topSpeed);
        float faded    = Mathf.Max(0.01f, curve.Evaluate(Mathf.Clamp01(fraction)));

        // Past topSpeed the curve has nothing left to say -- it is authored over 0..1 --
        // so without this the entire band boost opens up is FLAT. Measured before it
        // existed: 53.3 deg/s at 104 m/s against 53.4 at 80, i.e. the extra 24 m/s cost
        // nothing at all. Keep the fade falling across exactly the band boost adds.
        //
        // Driven by SPEED, not by boostLerp, and that is deliberate. Gating on the button
        // would tax boosting out of a corner at half top speed, which is when boost is
        // supposed to feel good, and would hand the steering back the instant the button
        // released while the craft was still doing 95. Everything else about this fade is
        // a pure function of speed and this stays one, which is also what makes authority
        // return by itself as a corner scrubs speed off.
        if (fraction > 1f)
        {
            float band = Mathf.Max(0.01f, P.boostSpeedMultiplier - 1f);
            float over = Mathf.Clamp01((fraction - 1f) / band);

            faded *= Mathf.Lerp(1f, P.boostYawMultiplier, over);
        }

        return Mathf.Lerp(faded, 1f, Mathf.Max(driftLerp, _strafeModeBlend));
    }

    /// <summary>
    /// THE ONE PLACE the drive-mode lane change is decided. Returns the sideways speed the
    /// craft should settle at, in m/s, signed right-positive.
    ///
    /// SCALED BY FORWARD SPEED, and that is the whole design rather than a refinement. A
    /// fixed sideways speed would be a gentle lean at 80 and a craft crabbing sideways
    /// under its own power at a standstill -- which is strafing without the trigger, the
    /// one thing the owner ruled this must never become. Scaling by speed makes it a
    /// constant ANGLE off the line of travel instead: the same move at every speed, and
    /// exactly nothing when stopped, with no threshold to tune or to feel.
    ///
    /// Forward speed is clamped at zero, so there is no lane change in reverse. Lane
    /// changing backwards is not a manoeuvre, and letting it through would flip which way
    /// the stick moves the craft.
    ///
    /// THREE EXEMPTIONS, matching yawSpeedFade:
    ///   Air     -- fades out with HoverSupport, via the damping coefficient this is
    ///              multiplied by. That is the SAME number air-control roll fades IN on
    ///              (_airControlWeight scales by 1 - HoverSupport), so the two are exactly
    ///              complementary and can never both own the stick. A grounded BOOL would
    ///              switch at a threshold and leave a seam mid-band, which is the defect
    ///              ApplyDrag already fixed once by scaling on support instead of gating.
    ///   Strafe  -- faded by _strafeModeBlend, because that axis is the strafe axis and the
    ///              two would otherwise stack into more lateral speed than either grants.
    ///   Drift   -- faded by driftLerp. A sideways shove during a held slide fights a
    ///              judged-good behaviour, and not exempting drift has now been the defect
    ///              twice in this subsystem (accelCurve, then yawSpeedFade).
    /// </summary>
    private float DriveLateralTarget()
    {
        if (P.driveLateralPush <= 0f)
            return 0f;

        float stick = Mathf.Clamp(input.StrafeX, -1f, 1f);
        if (stick == 0f)
            return 0f;

        float speedFraction = Mathf.Clamp01(Mathf.Max(0f, _cachedLocalVel.z) / Mathf.Max(0.01f, P.topSpeed));

        return stick * P.driveLateralPush * speedFraction
             * (1f - _strafeModeBlend)
             * (1f - driftLerp);
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
    private void ApplyDrag(float effectiveTopSpeed)
    {
        // Scaled by support rather than gated on IsHoverGrounded. Gating applied full
        // ground drag across the whole sensor band, so a tap jump was braked in mid-air
        // for most of its arc and arrived slower than it left. Both damp terms below are
        // scaled, so drag fades out with the springs instead of switching off 2.5m late.
        float support = foundation.HoverSupport;

        if (support <= 0f)
            return;

        // Lateral drag: always active grounded, independent of throttle.
        // Damps only UNWANTED lateral velocity: the player's intended strafe
        // velocity (stick * the shared StrafeTopSpeedScaled, which is exactly what
        // ApplyStrafe caps against) is excluded. This used to rebuild that ceiling
        // by hand from boostSpeedMultiplier; the two expressions were algebraically
        // identical, but a hand-rolled copy of a cap is the precise shape of the
        // defect that produced the strafe forward dead band, so it now calls the
        // helper instead. Damping raw velocity silently
        // capped strafe at strafeAccel / lateralDamp (25 m/s at default tuning),
        // below strafeTopSpeed (30), making that knob dead and the soft cap in
        // ApplyStrafe unreachable. With the intended term excluded, lateralDamp
        // purely kills unwanted slide and the soft cap owns the ceiling.
        // In drive mode intended is zero, so drive-mode behavior is unchanged.
        float effectiveLateralDamp = Mathf.Lerp(P.lateralDamp, P.driftLateralDamp, driftLerp) * support;
        if (effectiveLateralDamp > 0f)
        {
            float intendedLateral = Mathf.Clamp(input.StrafeX, -1f, 1f)
                                  * StrafeTopSpeedScaled(effectiveTopSpeed) * _strafeModeBlend;
            float unwantedLateral = _cachedLocalVel.x - intendedLateral;

            rb.AddForce(transform.right * (-unwantedLateral * effectiveLateralDamp), ForceMode.Acceleration);

            // The drive-mode lane change (TODO 0.36). Deliberately applied HERE, against the
            // damping term directly above it, because the two are one mechanism: the push is
            // expressed as the sideways speed it should SETTLE at, and multiplying by the same
            // damping coefficient is what makes that true. At equilibrium the two forces cancel
            // at exactly DriveLateralTarget, whatever lateralDamp happens to be, so the knob
            // cannot be silently rescaled by a later change to the drag.
            //
            // The consequence worth knowing: lateralDamp sets how QUICKLY the lean arrives
            // (time constant 1/lateralDamp, about a second at the shipped 1) while
            // driveLateralPush sets how FAR it goes. Splitting a single feel across two knobs
            // is a real cost, accepted because the alternative is excluding this from the drag
            // and rebuilding a second cap -- and a hand-rolled copy of a cap is exactly what
            // produced the strafe forward dead band.
            float lateralTarget = DriveLateralTarget();
            if (lateralTarget != 0f)
                rb.AddForce(transform.right * (lateralTarget * effectiveLateralDamp), ForceMode.Acceleration);
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
            float effectiveForwardDamp = Mathf.Lerp(P.forwardDamp, P.driftForwardDamp, driftLerp) * support;
            if (effectiveForwardDamp > 0f)
            {
                _dbgDrag = true;

                // Shares this tick's _driveAxis with ApplyDrive, and must. Drag is the
                // force that opposes drive, so measuring or applying it on a different
                // axis reintroduces the aim-pitch vertical component at the exact
                // moment drive stops resisting it: full drag arrives at throttle 0, so
                // the oscillation would simply move from the press to the release.
                //
                // Velocity is projected onto the same axis rather than read from
                // _cachedLocalVel.z, because a damping force whose magnitude and
                // direction disagree is not damping.
                float alongAxis = Vector3.Dot(rb.linearVelocity, _driveAxis);

                rb.AddForce(_driveAxis * (-alongAxis * effectiveForwardDamp * dragWeight), ForceMode.Acceleration);
            }
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
    /// TWO paths, because the right cap depends on whether heading equals velocity.
    ///
    ///   Not drifting: heading IS velocity, so a forward-axis read and a heading-aligned
    ///   force are exactly equivalent to the velocity-aligned version and cheaper. This
    ///   is the confirmed-good path; do not disturb it.
    ///
    ///   Drifting: heading and velocity diverge by design, so the cap is applied to total
    ///   horizontal speed and the force is aligned to VELOCITY. A heading-aligned force at
    ///   a 50 degree slide would push sideways and destabilize the chassis.
    ///
    /// Note for anyone reading an older copy of this file: this header used to claim the
    /// whole method was velocity-aligned, which the code had not been true of for a long
    /// time. It was accurate when written, then the method was narrowed to the forward
    /// axis and the header was not updated. Harmless while drift was suppressed here,
    /// since the two are identical when heading equals velocity, but it made the drift
    /// speed runaway much harder to find.
    /// </summary>
    private void ApplyOverSpeedBleed(float effectiveTopSpeed)
    {
        // Match the cap ApplyDrive actually clamps against. Drive stops pushing at the
        // strafe-blended top speed, so bleeding against the unblended one left a band
        // (strafeTopSpeed..topSpeed) where drive was suppressed for being over cap, drag
        // was suppressed for held throttle, and the bleed had not engaged yet -- nothing
        // acted on the chassis and it coasted there indefinitely. Same expression as
        // ApplyDrive so the two can never disagree again.
        float blendedTopSpeed = BlendedTopSpeed(effectiveTopSpeed);

        // ---- Drift: cap TOTAL horizontal speed, not the forward axis ----
        //
        // A forward-axis cap is only a speed cap while heading equals velocity. During a
        // drift they diverge by design, and the cap silently becomes a MULTIPLIER:
        // total = cap / cos(driftAngle). Measured 2026-08-13 on flat ground, holding full
        // throttle and full lock: the craft settled at 128.4 m/s against a topSpeed of 80,
        // because it sat at a 51.7 degree slide and 80 / cos(51.7) = 129. Drift was the
        // fastest thing in the game by 60%, and the resulting 337m arc was the "not enough
        // path curvature" complaint in TODO M.7.
        //
        // This used to be suppressed outright during drift, on the stated grounds that
        // "lateral velocity inflates total magnitude, causing false triggers". That reason
        // had already been designed out: the forward-axis read below exists precisely so
        // lateral cannot inflate anything. Deleting the suppression alone therefore fixes
        // nothing, because the forward axis is pinned AT the cap, never above it. The
        // comparison itself has to change, which is what this branch does.
        //
        // Velocity-aligned on purpose, and this is what the method's header has always
        // claimed it did: a heading-aligned force during a 50 degree slide pushes sideways
        // and destabilizes the chassis. Scaled by driftLerp so entry and exit cannot pop.
        if (driftLerp > 0f)
        {
            Vector3 horizVel = rb.linearVelocity;
            horizVel.y = 0f;
            float horizSpeed = horizVel.magnitude;

            // Sustained drifts lose their ceiling over time. See DriftSpeedCap. This is the
            // half of the bleed that actively TAKES speed; the ApplyDrive half only stops
            // adding it, which on its own would let a drift coast at its entry speed.
            float driftCap = Mathf.Lerp(blendedTopSpeed, DriftSpeedCap(blendedTopSpeed), driftLerp);

            if (horizSpeed > driftCap)
            {
                float driftExcess = horizSpeed - driftCap;
                _dbgBleed = true;
                rb.AddForce(-horizVel.normalized * (driftExcess * P.forwardDamp * driftLerp),
                            ForceMode.Acceleration);
            }
            return;
        }

        // Use forward-axis speed only. Total magnitude includes lateral, which is
        // irrelevant to the forward top-speed cap.
        float forwardSpeed = _cachedLocalVel.z;

        if (forwardSpeed > blendedTopSpeed)
        {
            float excess = forwardSpeed - blendedTopSpeed;
            _dbgBleed = true;
            rb.AddForce(-transform.forward * excess * P.forwardDamp, ForceMode.Acceleration);
        }
        else
        {
            // Mirror bleed for reverse. Catches dodge burst overshoot and any other
            // impulse that pushes past the reverse cap backward.
            //
            // Shares BlendedReverseTopSpeed with ApplyDrive, for the same reason the
            // forward axis shares BlendedTopSpeed: bleeding against a different cap than
            // drive clamps to is what opened the strafe forward dead band, and the reverse
            // axis gained the identical failure mode the moment its cap became
            // strafe-dependent. One expression, two callers, cannot disagree.
            float reverseCap = BlendedReverseTopSpeed(effectiveTopSpeed);

            if (forwardSpeed < -reverseCap)
            {
                float excess = -reverseCap - forwardSpeed;
                _dbgBleed = true;
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
        float effectiveLateralTopSpeed = StrafeTopSpeedScaled(effectiveTopSpeed);

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
    private void ApplyAirControl()
    {
        // !foundation.IsDowned: "not hover-grounded" is NOT the same as "airborne".
        // A craft on its flank finds no ground with its sensor rays, so it read as
        // airborne while lying on the floor and handed the player full roll
        // authority to lever itself upright against the ground -- faster than the
        // recovery it was supposed to be waiting out. A flip is meant to cost time.
        // HoverSupport, not !grounded. "The rays cannot see ground" was 2.5m later than
        // "the springs have stopped holding me up", so air control was unavailable for
        // the whole of a tap jump and most of any small hop. Support also SCALES the
        // final weight, which is what keeps this compatible with leveling: Foundation
        // fades levelingTorqueStrength out on the same curve, so authority hands over
        // with no window where both act on the same axis.
        // HasAirControlClearance is the floor, and it is why drifting into a hop is safe.
        // Drift and air control share a button, and the left stick is throttle on the
        // ground but pitch in the air, so holding drift through a small jump used to
        // reinterpret "still driving forward" as "full nose-down" the instant the craft
        // left the ground. That planted the nose, flipped the craft and charged the
        // player a full recovery, for the crime of not letting go of the stick. A height
        // floor separates the two cases outright: a hop on flat ground never clears it,
        // a charged jump or a ledge always does.
        //
        // HoverSupport < 1f is redundant against that floor at any sane tuning and is
        // kept anyway, because it is what guarantees leveling torque and air control can
        // never both act on the same axis even if the two thresholds are ever tuned to
        // cross. Cheap insurance against a jitter class this project has paid for before.
        bool active = P.enableAirControl
                   && input.Drift
                   && foundation.HasAirControlClearance
                   && foundation.HoverSupport < 1f
                   && !foundation.IsDowned;

        float target = active ? 1f : 0f;
        float step   = Time.fixedDeltaTime / Mathf.Max(0.01f, P.airControlBlendSeconds);
        _airControlBlend = Mathf.MoveTowards(_airControlBlend, target, step);

        _airControlWeight = _airControlBlend
                          * (1f - _strafeModeBlend)
                          * (1f - foundation.HoverSupport);

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

#if UNITY_EDITOR
    // -------------------------------------------------------------------------
    // 🎨 Tuning Readout
    // -------------------------------------------------------------------------
    /// <summary>
    /// Live movement state in the Scene view during play. This module owns most of the
    /// tunables in the profile and drew nothing, so every value below was previously
    /// invisible while driving.
    ///
    /// Reads only. All state is captured during FixedUpdate (see the debug fields);
    /// OnDrawGizmos runs on the editor's schedule and must not re-derive or mutate.
    ///
    /// What each line is for:
    ///
    ///   SPEED    forward and lateral against the caps ACTUALLY in force. Neither cap is
    ///            visible in the inspector: both move with boost, and the forward one also
    ///            blends toward the strafe ceiling. Tuning against topSpeed alone is how
    ///            you end up surprised by strafe mode.
    ///
    ///   FORCE    which longitudinal force acted this tick. The two states that matter:
    ///            NO FORCE (red) means drive, drag and bleed all declined -- the chassis is
    ///            coasting with nothing acting on it, which is the dead-band failure the
    ///            strafe caps produced once already. DRIVE+DRAG (red) means the mutual
    ///            exclusion in ApplyDrag has broken, which is a jitter source by the
    ///            project's own architecture rules.
    ///
    ///   BLENDS   the four authority weights. All are private, all gate large behaviour
    ///            changes, and none was observable before.
    ///
    ///   DRIFT    every entry gate with its live value, naming the one that is blocking.
    ///            Copied from Foundation's flip-recovery gizmo, which is the best
    ///            diagnostic in the project for exactly this reason: a state that will not
    ///            engage should say WHY, not leave you guessing which of four conditions
    ///            failed.
    ///
    ///   SHOULDER the signed angle between heading and velocity. This is THE drift metric
    ///            and nothing displayed it. It is an equilibrium rather than a setting:
    ///            driftYawMultiplier opens it, driftLateralDamp closes it, and maxDriftAngle
    ///            caps it by fading yaw authority as the slide widens. Watch this number
    ///            when tuning how a drift should feel, and read the live values off the
    ///            profile rather than from here.
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!ShouldDrawDebug || !Application.isPlaying) return;
        if (profile == null || rb == null || input == null) return;

        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        float   fwd      = localVel.z;
        float   lat      = localVel.x;

        // Signed heading-vs-velocity angle. Guarded: at rest the direction is undefined
        // and Atan2(0,0) would report a meaningless 0 that looks like a real reading.
        bool  moving   = localVel.sqrMagnitude > 1f;
        float shoulder = moving ? Mathf.Atan2(lat, fwd) * Mathf.Rad2Deg : 0f;

        // --- Force state ---
        //
        // Drive and forward drag overlapping is NOT automatically a fault, and an earlier
        // version of this readout flagged it red as an "opposing forces" violation. That was
        // wrong, and measured wrong: the overlap fires constantly in normal driving. Two
        // documented cases produce it deliberately, verified in play mode --
        //
        //   throttle fade  ApplyDrag's weight ramps to zero across 0..0.15 throttle rather
        //                  than snapping, so any throttle inside that band drives AND drags.
        //                  Measured at throttle 0.08: DRIVE=True DRAG=True, driftLerp 0.
        //   drift          ApplyDrag forces full drag weight during drift regardless of
        //                  throttle, because the drive/drag exclusion assumes heading equals
        //                  velocity and drift breaks that. Measured at full throttle + drift:
        //                  DRIVE=True DRAG=True, driftLerp 0.27.
        //
        // So the overlap is only suspicious OUTSIDE both cases. Naming which one is active
        // keeps the readout informative instead of crying wolf, and a red DRIVE+DRAG now
        // means something genuinely unexplained.
        bool noForce  = !_dbgDrive && !_dbgReverse && !_dbgDrag && !_dbgBleed;
        bool overlap  = (_dbgDrive || _dbgReverse) && _dbgDrag;
        bool inFade   = Mathf.Abs(input.ThrottleInput) < 0.15f;
        bool drifting = driftLerp > 0f;
        bool unexplainedOverlap = overlap && !inFade && !drifting;

        string forceText =
              unexplainedOverlap ? "DRIVE+DRAG  UNEXPLAINED (opposing forces, jitter source)"
            : overlap && drifting ? $"DRIVE+DRAG  (drift, expected)   accel {_dbgDriveAccel:F1}"
            : overlap             ? $"DRIVE+DRAG  (throttle fade, expected)   accel {_dbgDriveAccel:F1}"
            : _dbgDrive           ? $"DRIVE    accel {_dbgDriveAccel:F1}"
            : _dbgReverse         ? $"REVERSE  accel {_dbgDriveAccel:F1}"
            : _dbgDrag            ? "DRAG"
            : _dbgBleed           ? "BLEED    (over cap)"
            :                       "NO FORCE (coasting, nothing acting)";

        Color forceColor = (unexplainedOverlap || noForce) ? Color.red
                         : _dbgBleed                       ? new Color(1f, 0.6f, 0f)
                         : overlap                          ? new Color(0.6f, 0.9f, 0.4f)
                         :                                    Color.green;

        // --- Speed bar: forward speed against the live cap ---
        Vector3 barStart = transform.position + Vector3.up * 4.6f;
        float   capFrac  = _dbgFwdCap > 0.01f ? Mathf.Clamp01(fwd / _dbgFwdCap) : 0f;

        Gizmos.color = new Color(0.25f, 0.25f, 0.25f);
        Gizmos.DrawLine(barStart, barStart + transform.right * 3f);
        Gizmos.color = fwd > _dbgFwdCap + 0.01f ? Color.red : Color.green;
        Gizmos.DrawLine(barStart, barStart + transform.right * (capFrac * 3f));

        // --- Heading (white) vs velocity (cyan). The gap between them IS the drift angle. ---
        if (moving)
        {
            Gizmos.color = Color.white;
            Gizmos.DrawRay(transform.position, transform.forward * 4f);
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(transform.position, rb.linearVelocity.normalized * 4f);
        }

        // --- Drift gates, in the order ApplyDriftBlend evaluates them ---
        float turnMag = Mathf.Abs(input.TurnInput);
        string driftText =
              _isDrifting                          ? $"DRIFTING  lerp {driftLerp:F2}"
            : !input.Drift                         ? "idle: drift not held"
            : turnMag < P.driftTurnThreshold        ? $"BLOCKED: turn {turnMag:F2} < {P.driftTurnThreshold:F2}"
            : !foundation.IsHoverGrounded           ? "BLOCKED: airborne"
            : fwd < P.minDriftSpeed                 ? $"BLOCKED: speed {fwd:F1} < {P.minDriftSpeed:F1}"
            :                                         "ready";

        UnityEditor.Handles.color = forceColor;
        UnityEditor.Handles.Label(
            barStart + Vector3.up * 0.35f,
            $"SPEED     fwd {fwd,6:F1} / {_dbgFwdCap:F1}     lat {lat,6:F1} / {_dbgLatCap:F1}\n" +
            $"FORCE     {forceText}\n" +
            $"BLENDS    boost {boostLerp:F2}  drift {driftLerp:F2}  strafe {_strafeModeBlend:F2}  air {_airControlWeight:F2}\n" +
            $"DRIFT     {driftText}\n" +
            $"SHOULDER  {shoulder,6:F1} deg" + (moving ? "" : "  (stationary)")
        );
    }
#endif
}
