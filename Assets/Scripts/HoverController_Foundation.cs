using UnityEngine;

/// <summary>
/// HoverController_Foundation v1.9
///
/// v1.9: Hover rays read the SMOOTH surface normal, not the flat triangle normal.
///       A raycast reports the normal of whichever TRIANGLE it hit, while the terrain
///       the player sees is smooth-shaded from interpolated vertex normals, so the
///       physics was reading rounded ground as a series of tilted plates and levelling
///       torque re-levelled the chassis to each plate as it crossed. Measured on the
///       mountain terrain: ground-normal steps of 2.36 deg on average (worst 17.45)
///       arriving 5.9 times a second, with chassis pitch+roll at 36.44 deg/s in the
///       60ms after a crossing against 6.38 deg/s otherwise, nearly 6x. Reported by
///       the owner as "micro-bumpy" on rounded slopes before anything was measured,
///       and visible in BOTH camera modes, which is what ruled out the camera.
///       ResolveSurfaceNormal interpolates the vertex normals via the hit's
///       barycentric coordinate; probing 240m of terrain, face normals produced 7
///       discontinuities above one degree and smooth normals produced ZERO. The
///       resolved normal feeds the spring axis, the damping projection, the gravity
///       feedforward and AverageGroundNormal, which must all agree or they push along
///       slightly different axes. Preferred over filtering the normal over time
///       because a time filter cannot tell a triangle edge from a real ramp edge and
///       would make genuine terrain arrive late; nothing here is smoothed, only read
///       correctly. Per-mesh cache of triangles and normals is REQUIRED rather than an
///       optimisation, since Mesh.triangles and Mesh.normals allocate on every access.
///       Falls back to the flat normal for primitive colliders, convex meshes,
///       non-readable meshes and degenerate creases, so it is always safe to leave on.
///       New scene toggle: useSmoothGroundNormals (on).
///       NOTE the environment scale was NOT the cause and was briefly suspected:
///       uniform scaling leaves every angle unchanged, so doubling the world altered
///       how OFTEN facets arrive and not how large each one is.
///
/// v1.8: Flip recovery arms on its own angle. flipRecoveryArmAngle (70) is split out of
///       flipRecoveryAngleThreshold (80), which now owns only the downed lockout. The
///       shared value sat ABOVE the hover-supported resting equilibrium this file
///       already documented at ~78 deg, so a craft that settled into the gap was too
///       tipped to drive out and not tipped enough to be rescued, and never armed.
///       Caught on a screenshot reading "tilt 80deg (need 80) ... authorized False"
///       with unstick firing uselessly, which it must, since unstick pushes UP and the
///       problem is rotational. Splitting rather than lowering the shared number is
///       load-bearing: UpdateDownedState has no speed gate, so one value low enough to
///       rescue a stuck craft would also strip control the moment the chassis brushed a
///       steep bank at speed. Arming still needs a full flipRecoveryDelay under
///       flipRecoverySpeedThreshold, so it only ever catches a craft at rest. Verified:
///       parked at 75 deg, downed at 0.50s, upright by 2.66s. Gizmo now prints tilt to
///       one decimal and names both angles, because F0 rounding printed a satisfied
///       gate while the code disagreed.
///
/// v1.7: The clearance gate is predictive. It now compares the BALLISTIC PEAK from
///       current rise velocity against airControlMinClearance rather than the craft's
///       present clearance, because the thing it gates lasts a whole flight and
///       present-tense was the approximation. Cost of the approximation was 0.225s of
///       dead air at the start of every charged jump, which is precisely where a flip
///       needs to read as already having momentum. Same jumps are gated (clearing the
///       threshold from ground level takes 25.06 m/s of rise against a tap jump's 20),
///       the decision just happens at takeoff. Peak height is conserved under ballistic
///       motion so the prediction cannot drift or flicker during the climb, and it
///       decays to plain measured clearance on the way down, closing once at the
///       threshold. Both properties matter because airControlBlendSeconds is now
///       effectively instant and a flickering gate would chop the torque.
///
/// v1.6: Air control clearance floor. A dedicated downward probe reports how much room
///       is below the craft, and Propulsion gates air control on it. Reason: drift and
///       air control share a button, and the left stick is throttle on the ground but
///       PITCH in the air, so holding drift through a hop reinterpreted "still driving
///       forward" as full nose-down the instant the craft left the ground, planting it
///       and costing a full flip recovery. A height floor separates a hop from a real
///       jump outright. It needs its own ray because the hover sensors top out at
///       sensorRange - hoverHeight = 2.5m of measurable clearance while the tap jump
///       alone apexes above 5m, so the threshold is simply outside what they can see.
///       Measures clearance BELOW rather than height gained, so a hop off a ledge arms
///       and the same hop on flat ground does not, with no special case for either.
///       New tunable: airControlMinClearance (8m).
///
/// v1.5: HoverSupport replaces IsHoverGrounded as the gate for anything that should
///       behave differently in the air. IsHoverGrounded answers "can the rays see
///       ground", which is 2.5m later than "are the springs holding me up": above
///       hoverHeight compression goes negative and the spring clamps to zero, so the
///       craft is already in free fall while the rays still report hits. Four systems
///       read that band as grounded -- fall gravity stayed off, drag kept braking
///       mid-air, leveling pinned the chassis flat, air control was locked out -- and
///       a tap jump apexes 2.87m against a 2.5m band, so two thirds of it happened
///       inside the lie. That is the whole difference between a tap jump reading as
///       weak and floaty and a charged jump (apex 20.4m) reading as good. HoverSupport
///       is continuous rather than a second boolean on purpose: a boolean just moves
///       the cliff, and handing air control authority while leveling is still at full
///       strength puts two attitude authorities on the same axis. Fall gravity and air
///       control now scale by (1 - support) while leveling and drag scale by support,
///       so the handover is a crossfade with no overlap. Drive, jump charge and drift
///       entry deliberately keep the generous IsHoverGrounded signal. New tunable:
///       supportMargin (0.75m).
///
/// v1.4: Hover rays skip self-hits, flip recovery disarms on attitude only at a
///       new lower release angle, and IsDowned is added. ApplyHoverForces uses
///       RaycastNonAlloc and rejects hits whose attachedRigidbody is this craft:
///       groundLayers is Everything and the chassis colliders sit on the vehicle
///       layer, so past ~15.5 deg of bank the craft shot its own rims and read
///       0.04m instead of 6.83m. That was the drift flip. Flip-recovery disarm
///       dropped its "|| IsHoverGrounded" term, which was revoking righting
///       authority mid-rotation, and righting now releases at
///       flipRecoveryReleaseAngle rather than the arm threshold. IsDowned drives
///       Propulsion's control lockout. Do not modify without explicit justification.
///
/// v1.3: Gravity feedforward. Each hover point feeds forward its share of weight
///       along the surface normal, so the spring corrects error only instead of
///       also holding the craft up; hoverHeight is now the literal resting height
///       on flat ground and on slopes. Adds asymmetric extraFallGravity (airborne
///       and descending only). Slope lift compensation removed as redundant.
///       Ceiling duck added: ComputeEffectiveHoverHeight clamps ride height so low
///       geometry squats the craft instead of pinning it.
///
/// v1.2: Air control torque. Propulsion pushes airborne pitch/roll intent via
///       SetAirControl(pitch, roll, weight) while drift is held airborne;
///       per-axis torque authority (roll runs hotter than pitch), and
///       pitch/roll damping crossfades toward airControlDamping by the weight
///       so sustained rotation is actually reachable. Still the single
///       attitude authority: Propulsion applies no attitude torque itself.
///
/// v1.1: Hard landing spring give. On an airborne-to-grounded edge above a speed
///       threshold, spring forces are suppressed for a short front-loaded window
///       so the chassis slams onto its collider before popping back to ride
///       height. Fires OnHardLanding(severity). Feel only; no damage, no lockout.
///
/// Holds the chassis at hover height via per-point spring-dampers, levels it to
/// the ground normal, damps tilt, and runs two recovery paths:
///
///   Upright unstick: chassis touching ground but not flipped. A short tapering
///                    upward push frees it. Horizontal speed is intentionally
///                    NOT gated (unstick must work after any landing velocity);
///                    vertical speed IS gated so dynamic belly scrapes don't
///                    receive pulse trains while the hover springs are working.
///
///   Flip recovery:   chassis touching ground, flipped, and slow. Righting
///                    torque rotates it back upright. Longer delay so a flip
///                    reads as a real consequence.
///
/// Both paths can be suspended via SetRecoveryEnabled(bool) for EMP, scripted
/// events, or ability hooks.
///
/// Tuning lives on a VehicleTuningProfile asset (profile.foundation). Scene refs
/// (hoverPoints, groundLayers) and runtime state stay on the component.
///
/// Physics contract: never writes to rb.angularVelocity, rb.rotation, or
/// rb.linearVelocity. All motion is AddForce / AddTorque.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController_Foundation : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 📦 Tuning Profile
    // -------------------------------------------------------------------------
    [Header("📦 Tuning")]
    [Tooltip("Vehicle tuning profile (shared SO). All numeric tuning lives here. Required.")]
    [SerializeField] private VehicleTuningProfile profile;

    /// <summary>Shorthand for profile.foundation. Used at every read site below.</summary>
    private FoundationTuning F => profile.foundation;

    // -------------------------------------------------------------------------
    // 🧩 Hover Points
    // -------------------------------------------------------------------------
    [Header("🧩 Hover Points")]
    [Tooltip("Empty transforms positioned at the bottom of the chassis (one per corner is typical). " +
             "Each casts a ray straight down to find the ground. Assign by hand from the hover point children.")]
    [SerializeField] private Transform[] hoverPoints;

    // -------------------------------------------------------------------------
    // 🌍 Ground Interaction
    // -------------------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Layers treated as ground for hover detection.")]
    [SerializeField] private LayerMask groundLayers = ~0;

    [Tooltip("Read the SMOOTH shaded surface normal instead of the flat triangle normal.\n\n" +
             "A raycast returns the normal of the triangle it hit, so smooth-shaded terrain is " +
             "read by the physics as a series of tilted plates and the craft re-levels to each " +
             "one as it crosses it. Measured on the mountain terrain that is 5.9 attitude nudges " +
             "per second averaging 2.4 degrees, felt as a persistent micro-bumpiness on rounded " +
             "slopes. Reading the interpolated normal removed every discontinuity over 240m of " +
             "probing.\n\n" +
             "Falls back to the flat normal automatically wherever a smooth one is unavailable " +
             "(primitive colliders, convex meshes, meshes without Read/Write enabled), so it is " +
             "always safe to leave on. Turn it OFF to A/B the feel, or if terrain meshes are ever " +
             "made non-readable to save memory.")]
    [SerializeField] private bool useSmoothGroundNormals = true;

    [Tooltip("Draws hover rays in the scene view.")]
    [SerializeField] private bool drawDebugRays = true;

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug Rays. " +
             "Create via Assets > Create > Hover > Debug Settings.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.IsEnabled(HoverDebugCategory.Recovery) : drawDebugRays;

    // -------------------------------------------------------------------------
    // 📡 Collision Tracking
    // -------------------------------------------------------------------------
    /// <summary>
    /// Tracks ground-layer collision contact. OnCollisionStay fires every physics
    /// step while contact persists, refreshing a timestamp. Contact state is a
    /// timestamp, not a boolean, per the timestamps-over-booleans principle:
    /// an OnCollisionExit-cleared bool goes false when exiting ONE of two
    /// simultaneously touching ground colliders, and needs the exit callback to
    /// fire at all (not guaranteed on collider destruction). The timestamp simply
    /// goes stale. floorContactNormal is only read behind IsContactingFloor, so
    /// staleness there is harmless.
    /// Both flip recovery and unstick read these values; neither writes them.
    /// </summary>
    private void OnCollisionStay(Collision collision)
    {
        if ((groundLayers.value & (1 << collision.gameObject.layer)) == 0)
            return;

        lastGroundContactTime = Time.time;

        int count = collision.GetContacts(_contactBuffer);
        if (count == 0)
            return;

        // Two separate signals, and they must stay separate.
        //
        //   lastGroundContactTime / IsContactingGround: touching ANYTHING. Flip
        //   recovery and the downed lockout need this, because a craft flipped
        //   against a wall is still very much down and must still recover.
        //
        //   lastFloorContactTime / floorContactNormal: touching something
        //   FLAT ENOUGH TO BE STUCK ON. Only unstick uses this. It used to use
        //   the all-contact average, so a wall's horizontal normal became the
        //   unstick push direction and the craft shoved itself off any wall it
        //   touched, once every unstickRecoveryDelay. Hover cannot lift a craft
        //   off a wall, so there is nothing there to unstick from.
        float minNormalY = Mathf.Cos(F.unstickMaxSurfaceAngle * Mathf.Deg2Rad);

        Vector3 floorSum   = Vector3.zero;
        int     floorCount = 0;

        for (int i = 0; i < count; i++)
        {
            Vector3 n = _contactBuffer[i].normal;
            if (n.y < minNormalY)
                continue;

            floorSum += n;
            floorCount++;
        }

        // Averaged across floor contacts only, so touching a wall and the floor
        // at once still yields a clean upward push instead of a diagonal one.
        if (floorCount > 0)
        {
            lastFloorContactTime = Time.time;
            floorContactNormal   = (floorSum / floorCount).normalized;
        }
    }

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private Rigidbody rb;
    private HoverController_Energy energy;

    // Pre-allocated buffer for OnCollisionStay contact reads. Avoids per-call
    // ContactPoint[] allocation on every physics tick.
    private readonly System.Collections.Generic.List<ContactPoint> _contactBuffer = new(8);

    // Pre-allocated buffer for the hover raycasts. RaycastNonAlloc is used
    // instead of Raycast so self-hits can be skipped in favour of the next
    // hit along the ray (see ApplyHoverForces). Sized well above the number of
    // colliders a single 9.5m sensor ray can plausibly cross.
    private readonly RaycastHit[] _hoverHitBuffer = new RaycastHit[16];

    private float   unstickTimer;            // counts up while upright and contacting ground
    private float   flipTimer;               // counts up while flipped, slow, and contacting ground
    private float   lastGroundContactTime = float.NegativeInfinity; // any contact, refreshed by OnCollisionStay
    private float   lastFloorContactTime  = float.NegativeInfinity; // floor-like contact only (see OnCollisionStay)
    private Vector3 floorContactNormal;

    /// <summary>
    /// True while ground contact is fresh. OnCollisionStay refreshes the
    /// timestamp every physics step during contact; two fixed steps of grace
    /// covers callback ordering within a step.
    /// </summary>
    private bool IsContactingGround => Time.time - lastGroundContactTime <= Time.fixedDeltaTime * 2f;

    /// <summary>
    /// True while contact with a surface flat enough to be stuck on is fresh.
    /// Strictly narrower than IsContactingGround, which counts walls. Only the
    /// unstick path reads this; recovery deliberately uses the wider signal.
    /// </summary>
    private bool IsContactingFloor => Time.time - lastFloorContactTime <= Time.fixedDeltaTime * 2f;

    // Aim pitch target, written by Propulsion via SetAimPitch each FixedUpdate
    // while strafe mode is active. Degrees in Unity euler-X convention
    // (negative = nose up). Weight is the strafe blend (0 = no target).
    private float aimPitchDegrees;
    private float aimPitchWeight;

    // Air control intent, written by Propulsion via SetAirControl each
    // FixedUpdate while airborne with drift held. Inputs are raw stick -1..1;
    // weight is the air-control blend already suppressed by strafe precedence
    // (0 = inactive).
    private float airControlPitchInput;
    private float airControlRollInput;
    private float airControlWeight;
    private float   unstickFiredFlashTimer;  // drives the fired-impulse gizmo
    private bool    firedFlashWasFlip;       // which path set the flash (label only)
    private bool    rightingAuthorized;      // true after flip timer threshold; cleared when craft rights
    private bool    recoveryEnabled = true;
    private float   unstickForceTimer;       // counts down while sustained unstick lift is being applied
    private Vector3 unstickForceDir;         // direction cached at trigger time, held for duration
    private float   _effectiveHoverHeight;   // ride height the springs target this tick (lowest of the two below)
    private float   _rideHeightConstraint;   // ride height the CEILING allows: the involuntary half
    private float   _jumpChargeFraction;     // written by Propulsion via SetJumpCharge each FixedUpdate
    private float   _squatBlend;             // 0..1, follows the charge down, rate-limited on the way up

    /// <summary>
    /// Cached triangle and vertex-normal arrays per collision mesh, for
    /// ResolveSurfaceNormal. Static so several vehicles share one copy of the
    /// terrain; the arrays are read-only after the first fill.
    /// </summary>
    private struct MeshNormalData
    {
        public int[]     triangles;
        public Vector3[] normals;
    }

    private static readonly System.Collections.Generic.Dictionary<Mesh, MeshNormalData>
        _meshNormalCache = new System.Collections.Generic.Dictionary<Mesh, MeshNormalData>();
    private float   hardLandingTimer;        // counts down while hard-landing spring suppression is active
    private float   hardLandingSeverity;     // 0..1, cached at detection so the taper scales with impact

    /// <summary>True when at least one hover point has a ground hit this frame.</summary>
    public bool IsHoverGrounded { get; private set; }

    /// <summary>
    /// How much of the chassis weight the springs are actually carrying, 0..1. The
    /// continuous form of "grounded", and the correct gate for anything that should
    /// behave differently in the air.
    ///
    /// IsHoverGrounded answers "can the rays see ground", which is NOT the same
    /// question and is wrong by 2.5m at default tuning. The springs only push while
    /// compressed, so anywhere above hoverHeight the craft is in free fall with the
    /// rays still reporting hits. Four systems read that as grounded and behaved
    /// accordingly: fall gravity stayed off, drag kept scrubbing speed mid-air,
    /// leveling pinned the chassis flat, and air control was locked out. A tap jump
    /// at jumpImpulseMin 15 apexes 2.87m up against a 2.5m band, so roughly two
    /// thirds of it happened inside that, which is the whole reason a tap jump read
    /// as weak and floaty while a charged jump (apex 20.4m, clear immediately) read
    /// as good. One threshold, two impulses either side of it.
    ///
    /// Two terms, both load-bearing:
    ///
    ///   Height  fades 1 to 0 across supportMargin above the effective ride height.
    ///           Keyed to _effectiveHoverHeight, not F.hoverHeight, so a ducking
    ///           craft is judged against the height it is actually targeting.
    ///
    ///   Count   the fraction of hover points with a hit. Hanging one corner over a
    ///           ledge genuinely is three of four springs working, and this makes
    ///           that transition continuous instead of a snap at the last corner.
    ///
    /// Deliberately NOT used for drive, jump charge or drift entry. Those want the
    /// generous signal: losing throttle on every bump crest would be much worse than
    /// the problem this fixes.
    /// </summary>
    public float HoverSupport { get; private set; }

    /// <summary>
    /// Clear ground below the craft, in metres ABOVE the ride height being targeted.
    /// PositiveInfinity when the probe finds nothing.
    ///
    /// SATURATES at airControlMinClearance by design. The probe is only ever asked a
    /// yes/no question, so its range stops exactly where the answer stops changing and
    /// a MISS is the pass condition. Anything past the threshold does not need
    /// measuring, and a longer ray would cost more to tell us something nobody reads.
    /// </summary>
    public float AirControlClearance { get; private set; }

    /// <summary>
    /// True when there is enough room below to be granted air control. Propulsion's
    /// gate; see UpdateAirControlClearance for why this is a separate probe.
    /// </summary>
    public bool HasAirControlClearance { get; private set; }

    /// <summary>Average ground normal this frame (Vector3.up when airborne).</summary>
    public Vector3 AverageGroundNormal { get; private set; }

    /// <summary>
    /// True while the craft is flipped AND resting against the ground: the state
    /// flip recovery exists to undo. Propulsion reads this to lock out the jump
    /// and all commanded torque, so a flip costs the player the full recovery
    /// time instead of being cancellable.
    ///
    /// Both terms are load-bearing:
    ///
    ///   Tilt alone is not enough. Air control is gated on this, and a barrel
    ///   roll passes through the flip threshold every single time, so a
    ///   tilt-only test would cut the player's authority in the middle of the
    ///   air tricks the system exists for.
    ///
    ///   Contact alone is not enough either. IsHoverGrounded cannot stand in for
    ///   it: on its flank the craft's sensor rays point sideways and find
    ///   nothing, so it reads as airborne while physically lying on the floor.
    ///   That is exactly the hole that let a downed player hold drift and lever
    ///   themselves upright against the ground with attitude thrusters, at
    ///   measured 4.79 rad/s, beating the recovery outright.
    ///
    /// See UpdateDownedState for the exact condition and why it also latches.
    /// </summary>
    public bool IsDowned { get; private set; }

    // -------------------------------------------------------------------------
    // 📢 Events
    // -------------------------------------------------------------------------

    /// <summary>
    /// Fired once when the vehicle lands harder than hardLandingMinSpeed.
    /// Parameter is severity 0..1 mapping min..max impact speed. Pure feel hook
    /// today (VFX, camera punch); gameplay consequences can subscribe later.
    /// </summary>
    public event System.Action<float> OnHardLanding;

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// <summary>
    /// Sets the aim pitch target for the leveling torque. Called by Propulsion
    /// every FixedUpdate while strafe mode is active (weight = strafe blend),
    /// and with (0, 0) on strafe exit or EMP freeze.
    ///
    /// Degrees use Unity's euler-X convention: negative = nose up.
    /// While weight > 0, leveling drives toward the ground normal rotated by
    /// this pitch, at aimPitchTrackingStrength on the pitch axis only. This
    /// makes leveling the single torque authority over attitude; Propulsion
    /// applies no pitch torque of its own (competing forces cause jitter).
    /// </summary>
    public void SetAimPitch(float degrees, float weight)
    {
        aimPitchDegrees = degrees;
        aimPitchWeight  = Mathf.Clamp01(weight);
    }

    /// <summary>
    /// Sets how far into a grounded jump charge the craft is, 0..1. Called by
    /// Propulsion every FixedUpdate, and with 0 whenever no grounded charge is
    /// building: airborne, locked out, downed, or simply not holding.
    ///
    /// Propulsion owns the charge; Foundation owns the springs. This is the whole
    /// interface between them, and it is deliberately one number with no verbs in
    /// it. Propulsion cannot lower the craft, only report a fraction, so there is
    /// no path by which the jump could reach into the ride height and change the
    /// launch it was tuned for.
    ///
    /// EXPRESSES THE CHARGE, NEVER STORES IT. See ComputeEffectiveHoverHeight.
    /// </summary>
    public void SetJumpCharge(float chargeFraction)
    {
        _jumpChargeFraction = Mathf.Clamp01(chargeFraction);
    }

    /// <summary>
    /// Floor on the hover ray's normal projection, so a hull tilted far from the
    /// surface cannot manufacture lift. See the projection in ApplyHoverForces.
    ///
    /// Set to hoverHeight / sensorRange because that is the steepest tilt the sensors
    /// can actually serve: holding hoverHeight perpendicular to the surface at tilt t
    /// needs hoverHeight / cos(t) of ray, so at 7 and 9.5 the rays run out at
    /// acos(7 / 9.5) = 42.5 degrees. Compensating past the point the ray can reach
    /// would promise a ride height the sensors cannot see.
    ///
    /// CONSEQUENCE FOR TUNING: strafePitchLimit must stay meaningfully below 42.5.
    /// Past it the rays miss, the craft reads as airborne and sinks anyway, which is
    /// the original defect in a worse form. Buying more by raising sensorRange is not
    /// free: it also defines GROUNDED, and widening it makes hard landings LESS
    /// likely, which collides with TODO 0.21.
    /// </summary>
    private float MinGapProjection =>
        F.sensorRange > 0.001f ? Mathf.Clamp01(F.hoverHeight / F.sensorRange) : 1f;

    /// <summary>
    /// The rotation that takes the chassis back to square on the surface, so the hover
    /// sensors can be placed and pointed as though the craft were not aiming. Identity
    /// whenever the craft is not aiming, so drive mode is untouched by construction.
    ///
    /// MEASURED FROM THE HULL, NEVER FROM THE COMMANDED AIM ANGLE. The chassis is a
    /// torque servo and lags the stick, so sizing anything here from the command means
    /// correcting a tilt the craft has not reached: that is what put roughly 19 m/s^2
    /// of phantom lift under a fast stick sweep. Reading the achieved attitude has no
    /// such failure mode, because there is nothing being predicted.
    ///
    /// The reference is AverageGroundNormal rather than world up, and that is not a
    /// choice made here: ApplyLevelingTorque already defines the craft's attitude
    /// target as AverageGroundNormal rotated by the aim angle. So "square on the
    /// surface" is the existing definition of unaimed, and reading the deviation from
    /// it recovers the aim the servo has actually achieved, plus whatever error it is
    /// still carrying. Both should come out, which is why this is not clamped to
    /// strafePitchLimit.
    ///
    /// PITCH ONLY, about the chassis right axis. Roll must survive untouched: bank is
    /// real, the drift flip lived in the roll axis, and the wall-hover case in TODO
    /// 5.10 depends on the sensors rotating with a rolled craft.
    ///
    /// Scaled by aimPitchWeight, which is the strafe blend, so this fades in and out
    /// with strafe mode instead of switching.
    /// </summary>
    private Quaternion UnaimRotation()
    {
        if (aimPitchWeight <= 0.001f)
            return Quaternion.identity;

        Vector3 axis = transform.right;

        // Both flattened into the plane the pitch axis turns in, so roll and yaw
        // cannot leak into the angle.
        Vector3 hullUp   = Vector3.ProjectOnPlane(transform.up,     axis);
        Vector3 squareUp = Vector3.ProjectOnPlane(AverageGroundNormal, axis);

        if (hullUp.sqrMagnitude < 1e-6f || squareUp.sqrMagnitude < 1e-6f)
            return Quaternion.identity;

        float pitchDeviation = Vector3.SignedAngle(hullUp, squareUp, axis);

        return Quaternion.AngleAxis(pitchDeviation * aimPitchWeight, axis);
    }

    /// <summary>
    /// Sets the airborne pitch/roll control intent. Called by Propulsion every
    /// FixedUpdate (weight = air-control blend, strafe-suppressed), and with
    /// (0, 0, 0) when inactive or on EMP freeze.
    ///
    /// Sign conventions (applied here, not by the caller):
    ///   pitchInput +1 (stick up)    = nose down (+local X), arcade car standard.
    ///   rollInput  +1 (stick right) = roll right (-local Z), matching the
    ///   chassis-bank convention in Propulsion.
    /// </summary>
    public void SetAirControl(float pitchInput, float rollInput, float weight)
    {
        airControlPitchInput = Mathf.Clamp(pitchInput, -1f, 1f);
        airControlRollInput  = Mathf.Clamp(rollInput,  -1f, 1f);
        airControlWeight     = Mathf.Clamp01(weight);
    }

    /// <summary>
    /// Enables or disables both flip recovery and ground unstick.
    /// Use for EMP effects, scripted events, or ability interactions.
    /// When disabled, timers are reset so recovery doesn't fire immediately on re-enable.
    /// </summary>
    public void SetRecoveryEnabled(bool enabled)
    {
        recoveryEnabled = enabled;
        if (!enabled)
        {
            unstickTimer       = 0f;
            flipTimer          = 0f;
            rightingAuthorized = false;
        }
    }

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        rb     = GetComponent<Rigidbody>();
        energy = GetComponent<HoverController_Energy>();

        if (profile == null)
        {
            Debug.LogError(
                $"[Foundation] '{name}': VehicleTuningProfile is not assigned. " +
                $"Assign one in the inspector. Hover disabled.",
                this
            );
            enabled = false;
            return;
        }

        ValidateHoverPoints();
    }

    private void ValidateHoverPoints()
    {
        if (hoverPoints == null || hoverPoints.Length == 0)
        {
            Debug.LogError(
                $"[Foundation] '{name}': hoverPoints array is empty. " +
                $"Assign HoverPoint transforms in the Inspector. Hover disabled.",
                this
            );
            enabled = false;
            return;
        }

        for (int i = 0; i < hoverPoints.Length; i++)
        {
            if (hoverPoints[i] == null)
            {
                Debug.LogError(
                    $"[Foundation] '{name}': hoverPoints[{i}] is null. " +
                    $"Fix the missing reference in the Inspector. Hover disabled.",
                    this
                );
                enabled = false;
                return;
            }
        }
    }

    private void FixedUpdate()
    {
        // EMP freeze: all Foundation forces off. Default Unity gravity still pulls
        // the chassis down; existing angular velocity persists with no leveling or
        // damping, so the vehicle tumbles for the freeze duration.
        if (energy != null && energy.IsEmpFrozen)
        {
            IsHoverGrounded        = false;
            AverageGroundNormal    = Vector3.up;
            HoverSupport           = 0f;
            AirControlClearance    = 0f;
            HasAirControlClearance = false;  // no attitude authority during a freeze
            IsDowned               = false;  // EMP already removes all control
            return;
        }

        ApplyHoverForces();
        UpdateAirControlClearance();   // after ApplyHoverForces: reads _effectiveHoverHeight
        ApplyExtraGravity();
        ApplyLevelingTorque();
        ApplyAirControlTorque();
        ApplyPitchRollDamping();
        HandleRecovery();
        UpdateDownedState();
        ApplyUnstickForce();
    }

    private void Update()
    {
        // Tick the debug flash timer in Update so it runs at game framerate.
        // OnDrawGizmos runs on the editor's schedule (unpredictable, sometimes
        // multiple times per frame, sometimes suppressed) and must not mutate state.
        if (unstickFiredFlashTimer > 0f)
            unstickFiredFlashTimer = Mathf.Max(0f, unstickFiredFlashTimer - Time.deltaTime);
    }

    // -------------------------------------------------------------------------
    // 🧠 Per-Point Spring-Damper Lift
    // -------------------------------------------------------------------------
    /// <summary>
    /// Raycast that never returns this craft's own geometry, taking the nearest
    /// surviving hit. RaycastNonAlloc does not sort, hence the explicit min.
    /// attachedRigidbody resolves every chassis collider to this Rigidbody, so
    /// one comparison covers the whole hierarchy.
    ///
    /// Self-skipping is not optional here. groundLayers defaults to Everything
    /// and the chassis colliders sit on the vehicle's own layer, so a sensor ray
    /// can and does hit this craft: once ApplyChassisBank rolls meshRoot past
    /// ~15.5 degrees it swings Rim_FR / Rim_RR straight into their own hover
    /// rays. The ray then returned ~0.04m instead of ~6.8m, compression read as
    /// nearly the whole hoverHeight, and one flank's spring jumped from 12.5 to
    /// 121 m/s^2 -- about 6x the craft's weight, on one side, pushed along the
    /// RIM's surface normal instead of the ground's. That was the drift flip.
    /// </summary>
    private bool RaycastIgnoringSelf(Vector3 origin, Vector3 direction, float range, out RaycastHit nearest)
    {
        int count = Physics.RaycastNonAlloc(origin, direction, _hoverHitBuffer, range,
                                            groundLayers, QueryTriggerInteraction.Ignore);

        bool found = false;
        nearest = default;

        for (int i = 0; i < count; i++)
        {
            if (_hoverHitBuffer[i].collider.attachedRigidbody == rb)
                continue;

            if (!found || _hoverHitBuffer[i].distance < nearest.distance)
            {
                nearest = _hoverHitBuffer[i];
                found   = true;
            }
        }

        return found;
    }

    /// <summary>
    /// The surface normal the springs should actually use: the SMOOTH (shaded)
    /// normal where the geometry can supply one, falling back to the flat triangle
    /// normal where it cannot.
    ///
    /// Why this exists. A raycast returns the flat normal of whichever TRIANGLE it
    /// hit, while the terrain the player is looking at is smooth-shaded from
    /// interpolated vertex normals. So the hover system was reading rounded ground
    /// as a series of tilted plates, and levelling torque re-levelled the chassis
    /// to each plate in turn. Measured 2026-08-12 on the mountain terrain: ground
    /// normal steps of 2.36 degrees on average (worst 17.45) arriving 5.9 times a
    /// second, with chassis pitch+roll running 36.44 deg/s in the 60ms after a
    /// crossing against 6.38 deg/s otherwise. Nearly 6x, and the owner reported it
    /// as "micro-bumpy" on rounded slopes before any of it was measured.
    ///
    /// Probing 240m of that terrain, face normals produced 7 discontinuities above
    /// one degree and interpolated normals produced ZERO. Not reduced, removed.
    ///
    /// Chosen over filtering the normal over time, which was the first instinct: a
    /// time filter cannot tell a triangle edge from a real ramp edge, so it buys
    /// smoothness by making genuine terrain arrive late. This has no such trade,
    /// because nothing is smoothed, only read correctly.
    ///
    /// The cache is a requirement, not an optimisation. Mesh.triangles and
    /// Mesh.normals each allocate a fresh array on EVERY access, so reading them
    /// per ray would generate garbage four times per FixedUpdate forever. Keyed on
    /// the shared mesh, so repeated terrain shares one entry.
    ///
    /// Falls back to hit.normal, silently and by design, whenever a smooth normal
    /// is unavailable: primitive colliders, convex mesh colliders (which report
    /// triangleIndex -1), meshes without Read/Write enabled, and meshes carrying no
    /// vertex normals. All 24 collision meshes in the current scene supply one.
    ///
    /// TransformDirection is correct for the uniform scaling the environment uses.
    /// A non-uniformly scaled collider would need the inverse-transpose; if one is
    /// ever introduced this is where it breaks, and it will read as a slight
    /// persistent lean rather than as an error.
    /// </summary>
    private Vector3 ResolveSurfaceNormal(in RaycastHit hit)
    {
        if (!useSmoothGroundNormals || hit.triangleIndex < 0)
            return hit.normal;

        MeshCollider mc = hit.collider as MeshCollider;
        if (mc == null || mc.sharedMesh == null || !mc.sharedMesh.isReadable)
            return hit.normal;

        Mesh mesh = mc.sharedMesh;

        if (!_meshNormalCache.TryGetValue(mesh, out MeshNormalData data))
        {
            data = new MeshNormalData { triangles = mesh.triangles, normals = mesh.normals };
            _meshNormalCache[mesh] = data;
        }

        if (data.normals == null || data.normals.Length == 0)
            return hit.normal;

        int i = hit.triangleIndex * 3;
        if (i + 2 >= data.triangles.Length)
            return hit.normal;

        Vector3 bc = hit.barycentricCoordinate;
        Vector3 n  = data.normals[data.triangles[i]]     * bc.x
                   + data.normals[data.triangles[i + 1]] * bc.y
                   + data.normals[data.triangles[i + 2]] * bc.z;

        // A degenerate interpolation (opposed vertex normals across a crease)
        // would normalize to garbage and throw the spring off its axis.
        if (n.sqrMagnitude < 1e-6f)
            return hit.normal;

        return hit.collider.transform.TransformDirection(n).normalized;
    }

    /// <summary>
    /// Ride height the springs actually target this tick. TWO writers want to lower
    /// it and neither may simply win: the ceiling duck, which is involuntary, and the
    /// charge squat, which is the player holding the jump button.
    ///
    /// Seed at hoverHeight, let each contributor propose a height, commit the LOWEST.
    /// Both only ever lower, so the minimum is the one combination that cannot violate
    /// either: a craft charging a jump inside a tunnel gets the tunnel's height if the
    /// tunnel is tighter, and its own squat if the squat is deeper. Whoever ran last
    /// never decides anything.
    ///
    /// The duck's result is kept separately in _rideHeightConstraint, because the two
    /// answer different questions and not every consumer wants the combined figure.
    /// The air-control clearance probe measures room ABOVE the height the craft is
    /// entitled to, and a voluntary squat does not create room; reading the combined
    /// value there would have reported the squat depth as free space and handed out
    /// attitude authority the craft has not earned.
    /// </summary>
    private float ComputeEffectiveHoverHeight()
    {
        _rideHeightConstraint = ComputeCeilingDuckHeight();
        return Mathf.Min(_rideHeightConstraint, ComputeSquatHoverHeight());
    }

    /// <summary>
    /// Advances the squat blend. The one piece of this feature with memory, so it
    /// runs as an integrator before anything reads it, never inside a getter.
    ///
    /// ASYMMETRIC ON PURPOSE, and this asymmetry is what makes the squat EXPRESS the
    /// charge rather than STORE it.
    ///
    ///   Down  tracks the charge exactly. The ride height is meant to BE the charge
    ///         meter, so anything between the fraction and the height would be a lie
    ///         about how much jump is banked. The springs supply all the smoothing
    ///         this needs; they cannot teleport the chassis.
    ///
    ///   Up    is rate-limited. Snapping the target back at release leaves the craft
    ///         compressed against a target above it, and the springs answer that with
    ///         real upward force at the exact instant the jump impulse fires. That is
    ///         a launch the arc was never tuned for, arriving from a knob nobody would
    ///         think to look at. Taking chargeSquatRelease seconds to recover means the
    ///         craft is metres clear of its own springs before the target catches up,
    ///         so the squat returns exactly nothing.
    ///
    /// The cost of that honesty is that the craft launches from wherever the squat put
    /// it, so a charged jump peaks lower by the squat depth. That is a real change to
    /// the arc and it is the price of the readout; it is bounded, it is visible, and
    /// it scales with a single fraction the owner sets.
    /// </summary>
    private void IntegrateChargeSquat()
    {
        float target = F.enableChargeSquat ? _jumpChargeFraction : 0f;

        if (target >= _squatBlend || F.chargeSquatRelease <= 0f)
        {
            _squatBlend = target;
            return;
        }

        _squatBlend = Mathf.MoveTowards(_squatBlend, target,
                                        Time.fixedDeltaTime / F.chargeSquatRelease);
    }

    /// <summary>
    /// Ride height the charge squat is asking for. Depth is a FRACTION of hoverHeight
    /// rather than a distance, so it stays proportionate when ride height is retuned
    /// and can never be authored below the floor.
    ///
    /// Linear in the charge fraction, deliberately. Any easing would make the craft's
    /// height disagree with the charge it is reporting, and the whole point is that
    /// the two are the same number.
    /// </summary>
    private float ComputeSquatHoverHeight()
    {
        if (_squatBlend <= 0f)
            return F.hoverHeight;

        return F.hoverHeight * (1f - F.chargeSquatDepth * _squatBlend);
    }

    /// <summary>
    /// Ride height the CEILING allows, reduced when something overhead would otherwise
    /// be crushed into.
    ///
    /// The springs have no force ceiling, so a ceiling below ride height is a
    /// SOFT-LOCK rather than a nuisance: net upward push is 64 m/s^2 per metre of
    /// compression, and friction against the ceiling eats the drive. Measured on
    /// the default chassis at 8m of clearance, full throttle produced a top speed
    /// of 0.7 m/s and 0.4m of travel in two seconds. At 9m it reached 59.9 m/s.
    /// The entire transition is one metre wide, which is far too sharp to leave to
    /// level-design discipline when the failure is an immobilised player.
    ///
    /// Both probes fire from the hover-point centroid so their sum is the ABSOLUTE
    /// floor-to-ceiling gap, which does not depend on where in that gap the craft
    /// currently sits. Deriving the target from the ceiling distance alone would
    /// make it a function of current height and feed back on itself.
    ///
    /// One central probe, not one per hover point, deliberately: per-corner ducking
    /// under a sloped ceiling would drive the four points to different targets and
    /// tilt the chassis.
    /// </summary>
    private float ComputeCeilingDuckHeight()
    {
        if (!F.enableCeilingDuck || hoverPoints.Length == 0)
            return F.hoverHeight;

        Vector3 centroid = Vector3.zero;
        foreach (Transform point in hoverPoints)
            centroid += point.position;
        centroid /= hoverPoints.Length;

        Vector3 up = transform.up;

        // Anything higher than this cannot constrain us, so there is no reason to
        // look for it.
        float probeRange = F.hoverHeight + F.ceilingClearance;

        if (!RaycastIgnoringSelf(centroid, up, probeRange, out RaycastHit ceiling))
            return F.hoverHeight;

        if (!RaycastIgnoringSelf(centroid, -up, F.sensorRange, out RaycastHit floor))
            return F.hoverHeight;

        float gap = floor.distance + ceiling.distance;

        return Mathf.Clamp(gap - F.ceilingClearance, F.minDuckHoverHeight, F.hoverHeight);
    }

    private void ApplyHoverForces()
    {
        bool wasHoverGrounded = IsHoverGrounded;

        IsHoverGrounded     = false;
        AverageGroundNormal = Vector3.up;
        HoverSupport        = 0f;

        // Hard landing: tick the suppression window (front-loaded taper, same
        // idiom as unstick). liftFactor is exactly 1 when the timer is idle, so
        // the normal hot path pays one comparison.
        if (hardLandingTimer > 0f)
            hardLandingTimer = Mathf.Max(0f, hardLandingTimer - Time.fixedDeltaTime);

        float liftFactor = 1f;
        if (hardLandingTimer > 0f)
        {
            float progress = hardLandingTimer / Mathf.Max(0.01f, F.hardLandingSuppressDuration);
            liftFactor = 1f - F.hardLandingSuppressStrength * hardLandingSeverity * progress;
        }

        // Total downward acceleration the springs have to hold against: Unity's own
        // gravity plus the extraGravityMultiplier added in ApplyExtraGravity. Hoisted
        // out of the loop; every point feeds forward an equal share of it.
        float gravityMagnitude = Physics.gravity.magnitude * (1f + F.extraGravityMultiplier);

        // Reduced when something overhead would otherwise be crushed into, or when the
        // player is holding a jump charge. Applied uniformly to all four points so
        // neither a sloped ceiling nor the squat can tilt the chassis.
        IntegrateChargeSquat();
        _effectiveHoverHeight = ComputeEffectiveHoverHeight();

        Vector3 normalSum     = Vector3.zero;
        float   distanceSum   = 0f;
        int     groundedCount = 0;

        // Hovering is measured as though the craft were not aiming. Solved once per
        // tick, not per point, because all four sensors must move together or the
        // springs would tilt the chassis to correct a tilt nobody asked them to see.
        Quaternion unaim = UnaimRotation();

        foreach (Transform point in hoverPoints)
        {
            // Sensor placed and pointed where it would be with the aim removed.
            // Rotated about the vehicle origin, so the four keep their layout.
            Vector3 origin = transform.position + unaim * (point.position - transform.position);
            Vector3 rayDir = unaim * -point.up;

            // QueryTriggerInteraction.Ignore (inside the helper): trigger volumes
            // (pickups, ability fields) must never feed the hover springs.
            if (!RaycastIgnoringSelf(origin, rayDir, F.sensorRange, out RaycastHit hit))
            {
                if (ShouldDrawDebug)
                    Debug.DrawRay(origin, rayDir * F.sensorRange, Color.red);
                continue;
            }

            // Resolved ONCE and used for every term below. The spring axis, the
            // damping projection, the gravity feedforward and AverageGroundNormal
            // must all agree about which way "up" is, or they push the chassis
            // along slightly different axes.
            Vector3 surfaceNormal = ResolveSurfaceNormal(hit);

            // The ray is cast along the craft's own down, so a chassis tilted to AIM
            // measures a slant, not a height: hit.distance is the true gap divided by
            // cos(aim). The springs then hold hoverHeight along the slant, and the
            // craft sinks. Measured 2026-08-16: at 36 degrees of aim the vertical ride
            // height falls from 7.00m to 5.66m while the pitched hull drops one end by
            // a further 2.4m, taking belly clearance from 6.70m to 2.97m. That is the
            // scraping.
            //
            // Converted here, ONCE, so compression and HoverSupport cannot disagree.
            // Leaving support on the raw distance is worse than the sinking it fixes:
            // avgDistance 8.65 against a supportMargin band ending at 7.75 drives
            // HoverSupport to ZERO while the craft is sitting on the ground, and
            // leveling, drag and regen all scale by support while air control and fall
            // gravity scale by (1 - support).
            //
            // MEASURED from the ray itself, never from the commanded aim angle. The
            // first version of this used cos(commanded aim) and it kicked hard on a
            // fast stick sweep, because the chassis is a torque servo and lags the
            // stick: the command reached 36 degrees while the hull was still near 12,
            // so the correction was sized for a tilt that did not exist yet. That
            // under-read the gap by about 17%, which at liftStrength 16 is roughly
            // 19 m/s^2 of phantom lift, half of gravity. Returning the stick inverted
            // it: command back at 0 while the hull was still pitched, gap over-read,
            // springs dropping out. Up on the way out and down on the way back, which
            // is exactly what full stick up and full stick down produced.
            //
            // The projection has no such failure mode. It asks the geometry what the
            // tilt IS rather than what it was asked to be, so there is nothing to lag.
            // It also makes the whole term self-consistent: distance along the normal,
            // velocity along the normal, force along the normal.
            //
            // It self-cancels where it should. A craft sitting square on a slope has
            // its ray along the surface normal, the projection is 1, and nothing
            // changes. It only bites when the hull is tilted RELATIVE to the ground,
            // which is precisely when a slant distance stops being a height.
            //
            // Clamped because the projection collapses toward zero as the hull nears
            // its side, and an uncapped version would turn a glancing wall hit into
            // enormous lift. The floor is the same ratio the sensors can physically
            // deliver (hoverHeight / sensorRange), so amplification is bounded at
            // 1.36x and the wall-hover case in TODO 5.10 stays roughly as it was.
            float alongNormal = Vector3.Dot(-rayDir, surfaceNormal);
            float gap         = hit.distance * Mathf.Max(alongNormal, MinGapProjection);

            float compression         = _effectiveHoverHeight - gap;
            // Sampled at the un-aimed position too. Every term in this loop has to
            // live in the same frame: reading the rate at the real sensor while
            // measuring the gap at the un-aimed one damps a motion the spring is not
            // responding to.
            float velocityAlongNormal = Vector3.Dot(rb.GetPointVelocity(origin), surfaceNormal);

            // Gravity feedforward: each point carries its share of the chassis weight,
            // so the spring term only has to correct ERROR instead of also holding the
            // craft up. Without this the springs must compress until k*x equals weight,
            // which parked ride height a fixed gravity/stiffness below hoverHeight
            // (0.98m at the old 39.24 / 40). Now hoverHeight is literal, and liftStrength
            // is free to be tuned purely for how hard the chassis resists being pushed
            // around -- height and looseness stop being the same knob inverted.
            //
            // Scaled by surfaceNormal.y so it supports exactly the NORMAL component of
            // weight (G*cos0) and leaves the tangential component unopposed. Feeding it
            // forward along world up instead would cancel gravity outright and glue the
            // chassis to slopes.
            float gravityShare = gravityMagnitude * surfaceNormal.y / hoverPoints.Length;

            float springForce = compression * F.liftStrength
                              - velocityAlongNormal * F.liftDamping
                              + gravityShare;

            // Clamp at zero: the spring only pushes, never pulls. Load-bearing for
            // jump feel — without it the damping term would fight jump takeoff.
            springForce = Mathf.Max(springForce, 0f);

            // Hard landing spring give: momentum carries the chassis through
            // hover height onto its collider while the taper restores strength.
            springForce *= liftFactor;

            // ForceMode.Acceleration: mass independent, like every other force in
            // the controllers (jump, dodge, unstick, gravity, all torques).
            // liftStrength/liftDamping are accelerations, so one tuning profile
            // behaves identically across vehicle masses. Values retuned from
            // Force-mode by dividing by vehicle mass (1000): 10000 -> 10,
            // 1500 -> 1.5. Identical forces at mass 1000; feel unchanged.
            // Applied at the un-aimed position, which is what makes hovering blind to
            // aim rather than merely better at measuring it. Applying at the real
            // sensor would put the front pair on a longer arm than the rear and turn
            // the springs into a pitch torque that fights the aim servo, which is the
            // two-point balancing act that produced the thump.
            rb.AddForceAtPosition(surfaceNormal * springForce, origin, ForceMode.Acceleration);

            normalSum    += surfaceNormal;
            distanceSum  += gap;
            groundedCount++;

            if (ShouldDrawDebug)
                Debug.DrawRay(origin, rayDir * hit.distance, Color.green);
        }

        if (groundedCount > 0)
        {
            IsHoverGrounded     = true;
            AverageGroundNormal = (normalSum / groundedCount).normalized;

            // See the HoverSupport docstring. Height term fades across supportMargin
            // above the AUTHORED ride height; count term is the fraction of springs
            // with ground under them.
            //
            // Keyed to F.hoverHeight and NOT to _effectiveHoverHeight, which is the
            // opposite of what it looks like it should be. Lowering the target moves
            // the band down with it, so a craft that has not sunk yet reads as being
            // above its band and support collapses to 0 -- while it is closer to the
            // ground than it has ever been. Support gates regen, leveling, drag, air
            // control, fall gravity and the drift hop, so the craft would behave as
            // though airborne while sitting on the floor, for the whole descent.
            //
            // The authored height is the correct reference because the band asks "is
            // there ground close enough under me to be holding me up", and the answer
            // to that cannot get WORSE by moving closer to the ground. A settled duck
            // or squat is below the band and clamps to 1, exactly as before.
            float avgDistance  = distanceSum / groundedCount;
            float heightFactor = Mathf.InverseLerp(F.hoverHeight + F.supportMargin,
                                                   F.hoverHeight,
                                                   avgDistance);

            HoverSupport = heightFactor * ((float)groundedCount / hoverPoints.Length);

            if (!wasHoverGrounded)
                DetectHardLanding();
        }
    }

    // -------------------------------------------------------------------------
    // 🛩 Air Control Clearance Probe
    // -------------------------------------------------------------------------
    /// <summary>
    /// One downward probe answering "is there enough room below me to be given
    /// attitude authority". A floor under air control, not a replacement for the way
    /// airtime already decides what a player can finish.
    ///
    /// It needs its own ray because the hover sensors physically cannot see far
    /// enough: sensorRange 9.5 against hoverHeight 7 measures at most 2.5m of
    /// clearance, while the tap jump alone apexes above 5m. Any threshold that
    /// separates a hop from a real jump is outside what those rays can report.
    ///
    /// Three choices here are deliberate:
    ///
    ///   Range stops at threshold. The question is yes/no, so a MISS is the pass
    ///   condition and the ray is as short as the answer allows. AirControlClearance
    ///   saturates as a result, which is fine because nothing needs the true value.
    ///
    ///   World down, not -transform.up. The question is how far the GROUND is, and a
    ///   tumbling craft's local down points at the horizon.
    ///
    ///   Clearance below, not height gained since takeoff. Height gained cannot grant
    ///   control to a craft falling off a cliff, which is the case that most obviously
    ///   deserves it. Measuring the room actually available means a hop on flat ground
    ///   grants nothing while the same hop off a ledge grants control as the floor
    ///   drops away, with no special case for either.
    /// </summary>
    private void UpdateAirControlClearance()
    {
        if (hoverPoints.Length == 0)
        {
            AirControlClearance    = 0f;
            HasAirControlClearance = false;
            return;
        }

        Vector3 centroid = Vector3.zero;
        foreach (Transform point in hoverPoints)
            centroid += point.position;
        centroid /= hoverPoints.Length;

        // _rideHeightConstraint, not _effectiveHoverHeight: see ComputeEffectiveHoverHeight.
        // A voluntary squat lowers the craft but creates no room, so measuring against it
        // would report the squat depth as clearance.
        float probeRange = _rideHeightConstraint + F.airControlMinClearance;

        if (!RaycastIgnoringSelf(centroid, Vector3.down, probeRange, out RaycastHit hit))
        {
            AirControlClearance    = float.PositiveInfinity;
            HasAirControlClearance = true;
            return;
        }

        AirControlClearance = hit.distance - _rideHeightConstraint;

        // Predictive, not present-tense, and this is the whole point of the probe.
        //
        // The gate protects an entire flight, so the question it has to answer is
        // whether the craft WILL have room, not whether it has room this instant.
        // Asking present-tense cost 0.225s of dead air at the start of every charged
        // jump: authority was withheld while the craft climbed to the threshold, which
        // is exactly the window where a flip needs to look like it already has
        // momentum. Ballistic peak from current rise velocity closes that entirely.
        //
        // Two properties make this safe rather than clever:
        //
        //   It gates the SAME jumps. Reaching the threshold from ground level needs
        //   sqrt(2 * g * clearance) of rise, which at the shipped tuning is 25.06 m/s
        //   against a tap jump's 20. The hop is still locked out, just decided at
        //   takeoff instead of a fifth of a second later.
        //
        //   The prediction is INVARIANT through the climb. Peak height is conserved
        //   under ballistic motion, so the value does not drift as the craft rises and
        //   the gate cannot flicker mid-ascent. On the way down rise velocity is zero
        //   by construction, so it decays to plain measured clearance and closes once,
        //   cleanly, at the threshold. Both matter now that the blend is instant and a
        //   flickering gate would chop the torque.
        float riseGravity   = Physics.gravity.magnitude * (1f + F.extraGravityMultiplier);
        float riseVelocity  = Mathf.Max(0f, rb.linearVelocity.y);
        float predictedPeak = AirControlClearance
                            + riseVelocity * riseVelocity / (2f * Mathf.Max(0.01f, riseGravity));

        HasAirControlClearance = predictedPeak >= F.airControlMinClearance;

        if (ShouldDrawDebug)
            Debug.DrawRay(centroid, Vector3.down * hit.distance,
                          HasAirControlClearance ? Color.green : new Color(1f, 0.5f, 0f));
    }

    // -------------------------------------------------------------------------
    // 💥 Hard Landing Detection
    // -------------------------------------------------------------------------
    /// <summary>
    /// Called on the airborne-to-grounded edge. Samples descent speed along the
    /// ground normal (rb.linearVelocity is pre-integration here, so this frame's
    /// spring forces have not contaminated the read) and, above the threshold,
    /// arms the spring-suppression window and fires OnHardLanding.
    /// </summary>
    private void DetectHardLanding()
    {
        if (!F.enableHardLanding)
            return;

        float impactSpeed = -Vector3.Dot(rb.linearVelocity, AverageGroundNormal);

        if (impactSpeed < F.hardLandingMinSpeed)
            return;

        hardLandingSeverity = Mathf.InverseLerp(F.hardLandingMinSpeed, F.hardLandingMaxSpeed, impactSpeed);
        hardLandingTimer    = F.hardLandingSuppressDuration;

        OnHardLanding?.Invoke(hardLandingSeverity);

        if (ShouldDrawDebug)
            Debug.Log($"[Foundation] Hard landing: {impactSpeed:F1} m/s, severity {hardLandingSeverity:F2}", this);
    }

    // -------------------------------------------------------------------------
    // 🌎 Extra Gravity
    // -------------------------------------------------------------------------
    /// <summary>
    /// extraGravityMultiplier always applies. It increases the effective weight of the
    /// chassis and helps it sit firmly on the hover springs.
    /// extraFallGravity applies only while airborne AND DESCENDING. Gravity is otherwise
    /// symmetric, so the only way to kill float used to be raising it globally -- which
    /// also flattened jump arcs and forced the large jump impulses that compensate for it.
    /// Weighting the descent alone lets the ascent stay generous (air-trick window) while
    /// the landing still reads as decisive.
    /// Both are tuning knobs for physical character, not player input.
    /// </summary>
    private void ApplyExtraGravity()
    {
        if (F.extraGravityMultiplier > 0f)
            rb.AddForce(Physics.gravity * F.extraGravityMultiplier, ForceMode.Acceleration);

        // Scaled by unsupported fraction rather than gated on IsHoverGrounded. Gating
        // switched fall gravity off across the whole sensor band, where the springs
        // produce zero lift and the craft is already falling: a tap jump therefore
        // descended at 39.24 instead of 52.24 for most of its arc while a charged jump
        // got the full value, which is the floatiness. Fades in with height, so nothing
        // pops on the way out.
        float support = 1f - HoverSupport;

        if (support <= 0f)
            return;

        // World-space Y, matching the world-up jump impulse. Reading the body axis here
        // would flip the sign mid-flip and yank the chassis upward during air control.
        float airGravity = rb.linearVelocity.y < 0f ? F.extraFallGravity * support : 0f;

        if (airGravity > 0f)
            rb.AddForce(Vector3.down * airGravity, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // ⚖️ Torque-Based Attitude Alignment (ground normal + aim pitch)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Single torque authority over attitude. Two responsibilities, one torque:
    ///
    ///   Grounded leveling: align chassis up to AverageGroundNormal (original role,
    ///               unchanged behavior when no aim target is set).
    ///
    ///   Aim pitch:  while Propulsion sets a target via SetAimPitch (strafe mode),
    ///               the target up is the ground normal rotated by the aim pitch.
    ///               Axis split: the pitch-axis torque component runs at
    ///               aimPitchTrackingStrength, everything else (roll, bump
    ///               following) stays at base leveling strength, so FPS-snappy aim
    ///               tuning (150 vs 12) never stiffens the ride over bumps.
    ///               Replaces the old design where Propulsion torqued the nose
    ///               *against* this leveling (competing forces cause jitter).
    ///               pitchRollDamping is the oscillation killer.
    ///
    /// Airborne with an aim target, the pitch axis still tracks (aiming mid-jump
    /// matters in combat); AverageGroundNormal is Vector3.up there, and the align
    /// component gets zero strength, preserving free airborne attitude.
    /// </summary>
    private void ApplyLevelingTorque()
    {
        // Aim tracking is honored when hover springs are engaged or the craft is
        // truly in the air. While scraping ground without hover (belly/roof
        // contact), recovery paths own attitude and aim must not fight them.
        bool aiming = aimPitchWeight > 0.001f
                   && (IsHoverGrounded || !IsContactingGround);

        // Scaled by support rather than gated on IsHoverGrounded. Leveling is the single
        // attitude authority, so this is what lets air control take over cleanly: full
        // strength while the springs carry the craft, fading to zero as they stop, with
        // no window where both are pulling on the same axis. Gating meant the chassis
        // was pinned flat at full 12 for the first 2.5m of any jump, which is why a tap
        // jump allowed no tilt at all.
        float baseStrength = F.levelingTorqueStrength * HoverSupport;

        if (!aiming && baseStrength <= 0f)
            return;

        Vector3 targetUp = aiming
            ? Quaternion.AngleAxis(aimPitchDegrees, transform.right) * AverageGroundNormal
            : AverageGroundNormal;

        Vector3 torqueAxis = Vector3.Cross(transform.up, targetUp);

        if (aiming)
        {
            Vector3 pitchPart = Vector3.Project(torqueAxis, transform.right);
            Vector3 alignPart = torqueAxis - pitchPart;

            float aimStrength = Mathf.Lerp(baseStrength, F.aimPitchTrackingStrength, aimPitchWeight);
            rb.AddTorque(pitchPart * aimStrength + alignPart * baseStrength, ForceMode.Acceleration);
        }
        else
        {
            rb.AddTorque(torqueAxis * baseStrength, ForceMode.Acceleration);
        }
    }

    // -------------------------------------------------------------------------
    // 🛩 Air Control Torque (airborne pitch/roll from Propulsion intent)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies pitch/roll torque from the air-control intent set by Propulsion
    /// (drift held while airborne). Independent per-axis authority: roll runs
    /// hotter than pitch so a car-shaped craft barrel rolls faster than it flips.
    ///
    /// Steady-state rate per axis is torque / airControlDamping (both are
    /// angular accelerations); ApplyPitchRollDamping lerps its damping toward
    /// airControlDamping by this weight so the rates are actually reachable.
    ///
    /// No grounded gate needed: airborne, leveling baseStrength is already 0 so
    /// nothing fights this; on landing the weight decays over the blend window
    /// (~0.15s) and grounded leveling wins immediately. No enable check here:
    /// enableAirControl gates at the intent source in Propulsion, same as
    /// enableStrafe gates SetAimPitch.
    /// </summary>
    private void ApplyAirControlTorque()
    {
        if (airControlWeight <= 0.001f)
            return;

        Vector3 torqueLocal = new Vector3(
             airControlPitchInput * F.airPitchTorque,   // +X = nose down
            0f,
            -airControlRollInput * F.airRollTorque      // -Z = roll right
        ) * airControlWeight;

        rb.AddRelativeTorque(torqueLocal, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // ⚙️ Torque-Based Pitch/Roll Damping
    // -------------------------------------------------------------------------
    private void ApplyPitchRollDamping()
    {
        // Aim pitch damping: extra pitch-rate damping while aiming, scaled by
        // the strafe blend. pitchRollDamping alone (8) underdamps the aim
        // tracking spring at strength 150 (~35% flick overshoot), but raising it
        // globally would stiffen terrain response too. This keeps the two
        // tunable independently: ride damping vs aim settle.
        float pitchDamping = F.pitchRollDamping + F.aimPitchDamping * aimPitchWeight;
        float rollDamping  = F.pitchRollDamping;

        // Air control needs sustained rotation; ride damping (8) would cap roll
        // at half the intended rate and double the effective torque cost. Lerp
        // toward the dedicated air-control damping by air-control weight. Safe
        // against double-dip with aim damping: strafe precedence in Propulsion
        // makes aimPitchWeight and airControlWeight complementary, never both high.
        if (airControlWeight > 0f)
        {
            pitchDamping = Mathf.Lerp(pitchDamping, F.airControlDamping, airControlWeight);
            rollDamping  = Mathf.Lerp(rollDamping,  F.airControlDamping, airControlWeight);
        }

        if (pitchDamping <= 0f && rollDamping <= 0f)
            return;

        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);

        Vector3 dampingLocal = new Vector3(
            -localAngVel.x * pitchDamping,
            0f,
            -localAngVel.z * rollDamping
        );

        rb.AddRelativeTorque(dampingLocal, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // 🔄 Flip Recovery + 📌 Ground Unstick  (split paths)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Two fully independent recovery paths:
    ///
    ///   Upright unstick: contacting ground AND not flipped AND vertically
    ///                    settled (horizontal speed not gated). After
    ///                    unstickRecoveryDelay, begins a sustained upward push
    ///                    window via ApplyUnstickForce.
    ///
    ///   Flip recovery:   contacting ground AND flipped AND slow.
    ///                    After flipRecoveryDelay, sets rightingAuthorized which gates
    ///                    the righting torque each FixedUpdate until the craft rights
    ///                    or hover springs re-engage.
    ///
    /// 180° degeneracy: Cross(transform.up, Vector3.up) is zero when perfectly inverted.
    /// A forward pitch bias breaks the symmetry.
    /// </summary>
    private void HandleRecovery()
    {
        if (!recoveryEnabled)
            return;

        float tiltAngle = Vector3.Angle(transform.up, Vector3.up);

        // flipRecoveryArmAngle, NOT flipRecoveryAngleThreshold. The two were one
        // value and the shared value was ABOVE the resting balance point this
        // method's own comment documents at ~78 degrees, so a craft that settled
        // into that band could never arm: too tipped to drive out, not tipped
        // enough to be rescued. Caught 2026-08-08 with the gizmo reading
        // "tilt 80deg (need 80) ... authorized False" while unstick fired
        // uselessly, which it must, because unstick pushes UP and the problem is
        // rotational. This is the same equilibrium the release angle already had
        // to be split out for; only the arm side was left sharing.
        //
        // Splitting rather than just lowering the shared number is the load-bearing
        // part. UpdateDownedState reads flipRecoveryAngleThreshold with NO speed
        // gate, so one number low enough to rescue a stuck craft would also strip
        // control the instant the chassis brushed a steep bank at speed. Arming
        // cannot do that: it needs a full flipRecoveryDelay under
        // flipRecoverySpeedThreshold, so it only ever catches a craft at rest.
        //
        // isFlipped also suppresses the unstick path below, and that is correct
        // at the new angle: past 70 degrees the craft needs righting, not a lift.
        bool  isFlipped = tiltAngle >= F.flipRecoveryArmAngle;
        bool  isSlow    = rb.linearVelocity.sqrMagnitude < F.flipRecoverySpeedThreshold * F.flipRecoverySpeedThreshold;

        // --- Righting torque (runs every frame while authorized) ---
        // Disarm on attitude ONLY, and at a LOWER angle than it armed at.
        //
        // The hysteresis is load-bearing. Releasing at flipRecoveryAngleThreshold
        // handed the craft over at exactly the angle where two hover points still
        // reach the ground and the resulting one-sided lift balances
        // levelingTorqueStrength: a stable equilibrium at ~78 degrees. Measured on
        // real terrain, the craft armed, righted from 112 to 80, released, sat at
        // 74-83 for a full second without converging, drifted back over the
        // threshold and re-armed -- indefinitely. Righting has to carry it clear
        // of that band before letting go.
        //
        // This was missed the first time because the 8-pose regression ran on a
        // flat plane, where the craft rotates cleanly through the band instead of
        // settling into it. Flat ground is not a sufficient test for this.
        //
        // Min() keeps the release at or below the arm angle if the two are ever
        // tuned to cross, which would otherwise disarm on the same frame it armed.
        // Guards against flipRecoveryArmAngle, since that is what arming now reads.
        float releaseAngle = Mathf.Min(F.flipRecoveryReleaseAngle, F.flipRecoveryArmAngle);

        // The previous condition also disarmed on
        // IsHoverGrounded, which broke recovery outright: a craft lying on its
        // flank still lands two hover rays on the floor anywhere between roughly
        // 40 and 88 degrees of tilt, so the righting torque revoked itself
        // MID-ROTATION, every time, at ~84 degrees. Past 90 there are no ray hits
        // at all, so there is no lift and ApplyLevelingTorque contributes zero
        // (baseStrength is 0 when not hover-grounded); the craft coasted over on
        // momentum and fell back. Measured limit cycle: 102 -> 127 -> 84 -> 134,
        // never recovering, with the timer visibly counting and never completing.
        //
        // Removing the term does not let righting fight a craft riding a wall or
        // a loop, because arming already requires a full flipRecoveryDelay of
        // ground contact under flipRecoverySpeedThreshold. Anything carrying
        // enough momentum to hold a loop is an order of magnitude above that gate
        // and can never arm. A craft genuinely pinned at >80 degrees and under
        // 2 m/s is stuck by definition and should be righted.
        if (tiltAngle < releaseAngle)
            rightingAuthorized = false;

        if (rightingAuthorized && F.flipRecoveryTorque > 0f)
        {
            Vector3 torqueAxis = Vector3.Cross(transform.up, Vector3.up);

            if (torqueAxis.sqrMagnitude < 0.001f)
                torqueAxis = transform.right * 0.1f;

            rb.AddTorque(torqueAxis.normalized * F.flipRecoveryTorque, ForceMode.Acceleration);
        }

        // --- Upright unstick path ---
        // Gate: contacting ground AND upright AND vertically settled.
        // Horizontal speed is intentionally NOT gated (unstick must work after
        // any landing). Vertical speed IS gated: a genuinely stuck craft has
        // settled vertically, while a dynamic belly scrape has not. Without
        // this gate, sustained scrapes received a lift pulse every
        // unstickRecoveryDelay on top of working hover springs (a ~5Hz kick
        // train at default tuning).
        bool verticallySettled = Mathf.Abs(rb.linearVelocity.y) < F.unstickMaxVerticalSpeed;

        // IsContactingFloor, not IsContactingGround: rubbing a wall is not being
        // stuck to it, and must not arm the lift.
        if (IsContactingFloor && !isFlipped && verticallySettled)
        {
            unstickTimer += Time.fixedDeltaTime;

            if (unstickTimer >= F.unstickRecoveryDelay)
            {
                unstickTimer = 0f;

                unstickForceDir = floorContactNormal.sqrMagnitude > 0.01f
                    ? floorContactNormal
                    : Vector3.up;

                unstickForceTimer      = F.unstickLiftDuration;
                unstickFiredFlashTimer = 0.5f;
                firedFlashWasFlip      = false;

                if (ShouldDrawDebug)
                    Debug.DrawRay(transform.position, unstickForceDir * 2f, Color.cyan);
            }
        }
        else
        {
            unstickTimer = 0f;
        }

        // --- Flip recovery path ---
        // Gate: contacting ground AND flipped AND slow.
        if (IsContactingGround && isFlipped && isSlow)
        {
            flipTimer += Time.fixedDeltaTime;

            if (flipTimer >= F.flipRecoveryDelay)
            {
                flipTimer          = 0f;
                rightingAuthorized = true;

                unstickFiredFlashTimer = 0.5f;
                firedFlashWasFlip      = true;

                if (ShouldDrawDebug)
                    Debug.DrawRay(transform.position, Vector3.up * 2f, Color.magenta);
            }
        }
        else
        {
            // DECAY, not reset. The defect in the reset was that its cost did not
            // depend on the length of the interruption: a 229ms excursion and a
            // one-frame blip both threw away everything banked. Measured on a 25 m/s
            // wipeout, that discarded a clock sitting at 0.98 of 1.00.
            //
            // What pushes the craft out of the gate is its OWN settling tumble, so the
            // recovery was interrupting its own clock. See TuningLog.md > The downed
            // window.
            //
            // At decay 1 this is symmetric: an interruption costs exactly its own
            // duration, and a craft alternating in and out of the gate makes no net
            // progress. It therefore cannot arm anything the reset would not have
            // armed eventually; it only stops throwing away work already done.
            flipTimer = Mathf.Max(0f, flipTimer - Time.fixedDeltaTime * F.flipRecoveryProgressDecay);
        }
    }

    // -------------------------------------------------------------------------
    // 🚫 Downed State (control lockout)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Two terms, ORed, and each covers a case the other cannot.
    ///
    ///   Resting term (contact AND flipped): locks control the instant the craft
    ///   comes to rest inverted, before flipRecoveryDelay has elapsed, so the
    ///   window before recovery arms is not a free escape.
    ///
    ///   Latch term (rightingAuthorized): holds the lockout through the rest of
    ///   the recovery even after ground contact is lost. Load-bearing: as the
    ///   righting torque swings the craft back through ~82 degrees the hover
    ///   springs re-acquire and fling it clear of the ground, and without this
    ///   term the lockout released mid-flip at 137 degrees of tilt and handed
    ///   back both air control and the air jump. Authority is only revoked when
    ///   tilt falls under flipRecoveryReleaseAngle (35), NOT flipRecoveryAngleThreshold
    ///   (80), which is only what ARMS the lockout. The two are deliberately
    ///   different and the gap is large. Measured 2026-08-17: control returns
    ///   mid-swing at 32-35 degrees of tilt with the craft still rotating at
    ///   4.6 rad/s, not once it has settled upright.
    ///
    /// Why contact and not IsHoverGrounded: a craft hovering a wall or a loop is
    /// tilted past the threshold but never TOUCHES it, so contact stays false and
    /// it keeps full control. A craft lying on its flank does touch. IsHoverGrounded
    /// gets this exactly backwards -- on its flank the sensor rays point sideways
    /// and find nothing, so it reads as airborne.
    ///
    /// Deliberately NOT downed: a craft tumbling in mid-air after a hit. No
    /// contact and recovery has never armed, so air control still answers and a
    /// mid-air save stays available. The lockout is the price of coming to rest
    /// inverted, not of being knocked around.
    /// </summary>
    private void UpdateDownedState()
    {
        bool restingInverted = IsContactingGround
                            && Vector3.Angle(transform.up, Vector3.up) >= F.flipRecoveryAngleThreshold;

        IsDowned = recoveryEnabled && (rightingAuthorized || restingInverted);
    }

    // -------------------------------------------------------------------------
    // 📌 Sustained Unstick Lift
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies the unstick lift over a short window each FixedUpdate.
    /// Force is front-loaded: strongest on the first frame, tapering to zero.
    /// Reads as a gentle invisible lift rather than a visible punch.
    /// Mass independent so behavior is consistent across vehicles.
    /// </summary>
    private void ApplyUnstickForce()
    {
        if (unstickForceTimer <= 0f)
            return;

        float progress       = unstickForceTimer / Mathf.Max(0.01f, F.unstickLiftDuration);
        float forceMagnitude = F.unstickLiftForce * progress;

        rb.AddForce(unstickForceDir * forceMagnitude, ForceMode.Acceleration);

        unstickForceTimer = Mathf.Max(0f, unstickForceTimer - Time.fixedDeltaTime);
    }

#if UNITY_EDITOR
    // -------------------------------------------------------------------------
    // 🎨 Unstick Debug Gizmos
    // -------------------------------------------------------------------------
    /// <summary>
    /// Live recovery state in the Scene view during play.
    ///
    ///   Hover point spheres: green (ray hitting ground) / red (no hit)
    ///   Cyan line:           floorContactNormal while touching floor (absent on walls)
    ///   Yellow bar + label:  unstickTimer progress toward unstickRecoveryDelay
    ///   Orange bar + label:  flipTimer progress toward flipRecoveryDelay
    ///   Magenta sphere:      recovery fired flash, visible for 0.5s
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!ShouldDrawDebug || !Application.isPlaying)
            return;

        // unstickFiredFlashTimer is ticked in Update. Read-only here.

        // --- Hover point spheres ---
        if (hoverPoints != null)
        {
            foreach (Transform point in hoverPoints)
            {
                if (point == null) continue;
                Gizmos.color = IsHoverGrounded ? Color.green : Color.red;
                Gizmos.DrawWireSphere(point.position, 0.15f);
            }
        }

        // --- Lowered ride height (blue). Only drawn while actually lowered, and it
        //     NAMES WHICH WRITER WON, because the two look identical from outside
        //     and have opposite fixes: a duck means low geometry, a squat means the
        //     player is holding jump. If a craft is pinned and this is NOT showing,
        //     the duck is not engaging and the ceiling probe is the thing to look
        //     at. The ceiling ray is only meaningful for the duck, so it is only
        //     drawn when the ceiling is the binding constraint. ---
        if (profile != null && _effectiveHoverHeight < F.hoverHeight - 0.01f)
        {
            bool ducking = _rideHeightConstraint < F.hoverHeight - 0.01f;

            Gizmos.color = new Color(0.3f, 0.6f, 1f);

            if (ducking)
            {
                Gizmos.DrawRay(transform.position, transform.up * F.ceilingClearance);
                Gizmos.DrawWireSphere(transform.position + transform.up * F.ceilingClearance, 0.2f);
            }

            string cause = ducking
                ? (_squatBlend > 0.01f ? "DUCKING + SQUAT" : "DUCKING")
                : $"SQUAT {_squatBlend * 100f:F0}%";

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 3.2f,
                $"{cause}  ride {_effectiveHoverHeight:F2} / {F.hoverHeight:F2}m"
            );
        }

        // --- Floor contact normal (cyan). Absent while only touching walls,
        //     which is itself the useful signal: no floor contact, no unstick. ---
        if (IsContactingFloor && floorContactNormal.sqrMagnitude > 0.01f)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(transform.position, floorContactNormal * 1.5f);
            Gizmos.DrawWireSphere(transform.position + floorContactNormal * 1.5f, 0.1f);
        }

        // --- Upright unstick timer bar (yellow) ---
        if (IsContactingFloor && unstickTimer > 0f && profile != null)
        {
            float progress  = Mathf.Clamp01(unstickTimer / Mathf.Max(0.01f, F.unstickRecoveryDelay));
            Gizmos.color    = Color.Lerp(Color.yellow, new Color(1f, 0.8f, 0f), progress);
            Vector3 start   = transform.position + Vector3.up * 0.3f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);
            Gizmos.DrawWireSphere(end, 0.08f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 1.5f,
                $"Unstick {unstickTimer:F2} / {F.unstickRecoveryDelay:F2}s"
            );
        }

        // --- Flip recovery diagnostics ---
        // Shown whenever the craft is tilted enough to care, not only while the timer is
        // already running, because the interesting failure is the timer NOT starting (or
        // resetting). Every gate is printed with its live value so a stuck craft says
        // exactly which condition is blocking it instead of leaving it to guesswork.
        if (profile != null && Application.isPlaying)
        {
            float tiltNow = Vector3.Angle(transform.up, Vector3.up);
            if (tiltNow >= F.flipRecoveryArmAngle * 0.5f)
            {
                // Arm angle, matching HandleRecovery. Reading the DOWNED threshold
                // here is what made the 2026-08-08 stuck craft unreadable: it
                // printed "need 80" while arming actually wanted a different
                // number, so the gizmo named a gate that was not the one in force.
                bool  flippedNow = tiltNow >= F.flipRecoveryArmAngle;
                float speedNow   = rb.linearVelocity.magnitude;
                bool  slowNow    = speedNow < F.flipRecoverySpeedThreshold;
                bool  contactNow = IsContactingGround;

                // Authority is reported FIRST. The old chain tested the arming
                // gates before rightingAuthorized, so a craft that was already
                // authorized and applying righting torque printed
                // "BLOCKED: moving too fast" -- the arming gates are irrelevant
                // once authority is held, and that label sent a whole debugging
                // session after the wrong condition. The gates below describe
                // only what is stopping the timer from ARMING.
                string blocker =
                      rightingAuthorized ? "RIGHTING"
                    : !contactNow  ? "BLOCKED: no ground contact"
                    : !flippedNow  ? "BLOCKED: tilt below threshold"
                    : !slowNow     ? "BLOCKED: moving too fast"
                    : "counting up";

                UnityEditor.Handles.color = rightingAuthorized ? Color.magenta
                                          : (contactNow && flippedNow && slowNow) ? new Color(1f, 0.6f, 0f)
                                          : Color.red;
                UnityEditor.Handles.Label(
                    transform.position + Vector3.up * 2.6f,
                    // Tilt to ONE DECIMAL, deliberately. At F0 a craft resting at
                    // 79.6 printed "tilt 80deg (need 80)", which reads as a gate
                    // that is satisfied while the code disagrees, and that cost a
                    // session before the split above was found. A readout must not
                    // round a value across the threshold it is being compared to.
                    $"FLIP {flipTimer:F2}/{F.flipRecoveryDelay:F2}s  {blocker}\n" +
                    $"  tilt {tiltNow:F1}deg (arm at {F.flipRecoveryArmAngle:F1}, downed at {F.flipRecoveryAngleThreshold:F1})\n" +
                    $"  speed {speedNow:F2} (need < {F.flipRecoverySpeedThreshold:F2})\n" +
                    $"  contact {contactNow}   hoverGrounded {IsHoverGrounded}   authorized {rightingAuthorized}   downed {IsDowned}"
                );
            }
        }

        // --- Flip recovery timer bar (orange) ---
        if (IsContactingGround && flipTimer > 0f && profile != null)
        {
            float progress  = Mathf.Clamp01(flipTimer / Mathf.Max(0.01f, F.flipRecoveryDelay));
            Gizmos.color    = Color.Lerp(new Color(1f, 0.4f, 0f), Color.red, progress);
            Vector3 start   = transform.position + Vector3.up * 0.6f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);
            Gizmos.DrawWireSphere(end, 0.08f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 2.0f,
                $"Flip {flipTimer:F2} / {F.flipRecoveryDelay:F2}s"
            );
        }

        // --- Fired-impulse flash ---
        if (unstickFiredFlashTimer > 0f)
        {
            float alpha     = unstickFiredFlashTimer / 0.5f;
            Gizmos.color    = new Color(1f, 0f, 1f, alpha);
            Gizmos.DrawWireSphere(transform.position, 0.6f);
            Gizmos.DrawWireSphere(transform.position, 0.9f);

            UnityEditor.Handles.color = new Color(1f, 0f, 1f, alpha);
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 2f,
                firedFlashWasFlip ? "FLIP RECOVERY ARMED" : "UNSTICK FIRED"
            );
        }
    }
#endif
}
