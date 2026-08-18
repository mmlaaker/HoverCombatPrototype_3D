using UnityEngine;
using Unity.Cinemachine;
using Unity.Cinemachine.TargetTracking;

/// <summary>
/// HoverCameraController v2.9
/// ------------------------------------------------
/// v2.9: THE BOOST REVERSE GATE READS TRAVEL, NOT THE NOSE.
///
///   ForwardGate multiplies every boost term by Clamp01(speed / forwardGateSpeed),
///   and that speed was velocity projected onto CHASSIS forward. It passes through
///   zero whenever the nose swings past perpendicular to travel, which is exactly
///   what a flip does. The craft is still moving forward at full speed and only
///   the nose has rotated, but the gate read it as reversing and slammed every
///   boost cue shut.
///
///   Measured 2026-08-14 across four boosted flips: every lurch sat on the zero
///   crossing of velocity dot chassis-forward, at tilt 86.6 to 103.1 degrees, with
///   boost held between 0.57 and 1.00. At forwardGateSpeed 2 and a measured slew
///   of 535 m/s^2 the gate closes and reopens in 7ms, so up to 10 degrees of FOV
///   and 3m of camera pull-back snap shut and back inside three frames. That is
///   what makes it read as a hitch rather than as a transition. Predicted 7ms
///   from the tuning, observed 7ms in the trace. Zero occurrences in a barrel
///   roll, where the nose never leaves the travel line, and zero across 7,496
///   grounded frames above 40 m/s, where forward speed is never negative.
///
///   Confirmed by the owner before the fix landed: flips without boost never
///   lurch, flips with boost lurch every time.
///
///   Same lesson v2.7 already learned for the heading proxy, and the same shape
///   of fix: the chassis is the untrustworthy signal mid-stunt, so read the
///   stable heading instead. Reversing still gates, because velocity then opposes
///   the heading. Strafe is deliberately unchanged: lateral travel still projects
///   to roughly zero along the heading, and whether a sideways boost should widen
///   the lens is an open question nobody has judged (see StrafeBoost below).
///
/// v2.8: TRAVEL HEADING IS LATCHED, NOT RECOMPUTED.
///
///   v2.7 skipped its bound entirely whenever horizontal speed fell under
///   travelHeadingMinSpeed, then 8 m/s. That guard was added for a real reason,
///   since a near-stationary craft has no well-defined direction of travel, and
///   it disabled the feature in precisely the case the feature existed for: a
///   flip bleeds off horizontal speed, so the craft finishes one plummeting
///   almost straight down. Measured at a real landing, 5 m/s of horizontal
///   against 60 of descent. The bound switched itself off and divergence went
///   back to accumulating, which the owner reported as a stutter "on landing
///   after flips" while mid-air flips felt fixed.
///
///   Latching beats lowering the floor because ANY floor has that failure mode,
///   and because the latched value stays correct through it: with no horizontal
///   force acting, the heading a falling craft had a moment ago is still the
///   heading it has. Refreshed OUTSIDE the air-control gate so engaging air
///   control mid-flight cannot adopt a stale heading from the previous jump.
///
///   Verified by injection both ways: at 5 m/s horizontal the proxy held 120
///   degrees off travel for an entire fall and the bound never fired; with the
///   latch it unwinds to exactly 40.000 and holds. Confirmed in play across
///   matched sessions: air-control frames breaching the 40 degree bound fell
///   from 9.9% to 0.5%, worst divergence 113 -> 50.6 degrees.
///
/// v2.7: HEADING PROXY BOUNDED AGAINST THE DIRECTION OF TRAVEL.
///
///   The fix that actually worked. v2.6 capped how FAST the heading could
///   unwind and did not touch how FAR it could drift first, so it converted a
///   snap into 0.6s of sustained pan: proxy yaw sat frozen at exactly 93.29 for
///   a whole stunt while the vehicle's swung through ~180 degrees. No catch-up
///   rate makes 126 degrees of accumulated error feel good.
///
///   Bounding against TRAVEL rather than the nose is what makes a bound safe.
///   Chassis yaw is exactly the untrustworthy signal mid-stunt: a flip sweeps
///   the nose through vertical, which swings the real heading AND makes the
///   euler decomposition jump, measured at up to 140 degrees in a single frame.
///   Travel heading has neither problem and is very nearly constant on a
///   ballistic arc, since gravity acts only on the vertical component. So the
///   stunt keeps its still frame and the chassis realigns with travel on
///   landing, leaving little to recentre.
///
///   This also explains why a FLIP provokes the stutter harder than a ROLL, an
///   observation the owner made by feel before any instrument could confirm it:
///   a roll spins about the travel axis and barely changes heading, so the
///   bucket stays empty; a flip fills it.
///
///   Gated on air control, because bounding against travel during ordinary
///   driving would fight drift, where pointing the nose off the racing line is
///   the entire manoeuvre.
///
/// v2.6: RATE CEILING ON THE HEADING PROXY.
///
///   Kept, because it is what guarantees no single frame can snap whatever the
///   branch or the travel bound do, and it is applied ONCE to the total move
///   rather than to the branch step for exactly that reason. But recorded
///   honestly: on its own it did NOT fix the reported stutter. Both converge
///   branches are proportional to an error with no upper bound, so releasing
///   air control spent whatever had accrued at a rate proportional to it, up to
///   an implied 1393 deg/s. The ceiling halved the whip (mean peak 88.4 -> 46.2
///   deg/s) and the owner could still barely feel the difference.
///
///   The general lesson is worth more than the fix: a measurable improvement in
///   the metric you chose is not evidence you chose the right metric.
///
/// v2.5: BOOST FRAMING. The first contributor with MEMORY. See CLAUDE.md's
///   module table for the full entry; it landed without a header note here and
///   is listed for continuity rather than re-described.
///
/// v2.4: THREE FIXES FROM A TUNING SESSION, ONE OF THEM STRUCTURAL.
///
///   • FRAMING GUARD. "Keep the craft in frame" turned out not to be a tuning
///     target at all. Pitch look-ahead and speed look-ahead both tip the aim,
///     independently, and neither knows the other exists, so any pair of values
///     that fits at rest gets overrun the moment the other term joins in. That
///     is what throttling at full stick-up does. No pair of numbers can fix a
///     sum neither term bounds, so ContributeFramingGuard runs last, measures
///     the finished frame, and scales look-ahead back until the hull fits.
///     Pitch Look Ahead and Speed Look Ahead Max are now maxima rather than
///     fixed amounts; the inspector reports whenever the guard intervenes.
///
///   • BOOST FOV NO LONGER FIRES IN REVERSE. Boost engages while reversing by
///     design, and the lens was widening for it. A wider lens is a forward-rush
///     cue and there is no rush to sell when backing up.
///
///   • RIGHT-STICK PITCH BLEED. X is yaw and Y is camera pitch on the same
///     stick, so holding a drift line unavoidably feeds the camera. Pitch input
///     is now attenuated by steering effort.
/// ------------------------------------------------
/// v2.3: DEFINITIVE STATE PREVIEW.
///
/// You cannot adjust a slider while both hands are on a controller. That made
/// tuning any held state a cycle of drive, notice, stop, guess, drive again,
/// which is close to useless for states that only exist while an input is held.
///
/// So the contributors stopped reading the world directly. They now take a
/// FramingInputs struct, and there are two producers for it: GatherLiveInputs
/// reads the real signals while playing, BuildPreviewInputs hand-computes the
/// settled values for a chosen CameraPreviewState. Both feed THE SAME SOLVER.
///
/// That last part is the whole design. The tempting version of this feature is
/// a preview that recomputes the framing itself, and that version starts lying
/// the first time anyone edits a contributor. Substituting inputs cannot drift,
/// because there is only ever one implementation of the math.
///
/// Nothing here writes to Propulsion. Its blends are read-only and recomputed
/// every FixedUpdate, so forcing them would be both a layering violation and
/// futile; a preview supplies its own numbers instead.
///
/// EvaluateState is the same machinery used as a measurement rather than a
/// pose: it solves a state without committing it and reports whether the craft
/// actually fits in frame. See FramingVerdict for why that question has a
/// fixed budget and why two features are always competing for it.
/// ------------------------------------------------
/// v2.2: EDIT-MODE PREVIEW.
///
/// v2.1 made the tuning fields draggable during play. This makes them
/// draggable with the game stopped as well, so framing can be composed against
/// the actual scene instead of being guessed and then checked in a play
/// session.
///
/// [ExecuteAlways] alone would be wrong here: it would run the whole runtime
/// path in the editor, which creates the CameraHeadingProxy as a real scene
/// object, demands an input provider that does not exist yet, and integrates
/// stick and drift state against a vehicle that is not moving. So the edit-mode
/// path is separate and does exactly one thing: commit the resting pose.
///
/// That is enough BY CONSTRUCTION, and it is worth saying why rather than
/// leaving it to look like a shortcut. At rest every contributor is identically
/// zero (no stick deflection, no speed, no drift, no boost) and the pitch orbit
/// reproduces the authored offset exactly at the neutral elevation, which is
/// the whole point of deriving the neutral from that offset. So the resting
/// solution IS the seed, and the preview needs no Rigidbody, no propulsion and
/// no input to be correct.
///
/// Writes are skipped when the value has not changed. At runtime that is a
/// minor saving; in edit mode it is what stops an unconditional per-tick write
/// leaving the scene permanently dirty. Strafe DAMPING is excluded from the
/// preview for the same reason: it describes how the rig chases a moving
/// target, so it shows nothing in a still frame and would dirty the scene on
/// open for no gain.
///
/// The preview and "Pull Framing From VCams" are opposite directions of the
/// same copy, so they cannot both be live. previewInEditMode is the switch: ON,
/// the tuning fields drive the vcams; OFF, the vcams are left alone so framing
/// can be composed on them directly and then adopted with the pull.
/// ------------------------------------------------
/// v2.1: EVERY SEED IS LIVE-EDITABLE.
///
/// v2.0 made the solution the single writer, and that had a consequence nobody
/// wants in the middle of a tuning drive: a vcam's Follow Offset, Tracked
/// Object Offset and Lens FOV are now OUTPUTS. They are rewritten every frame,
/// they read as micro-jitter in the inspector because their inputs never sit
/// perfectly still (hover bob shows up in the Rigidbody velocity that feeds
/// speed look-ahead), and anything typed into them is overwritten on the next
/// frame. Meanwhile the values they were seeded FROM were private fields
/// captured in Awake, so the resting framing had no editable home at all.
///
/// The seeds now live in the tuning sections alongside everything else:
/// framing.driveFollowOffset, framing.driveTargetOffset, framing.baseFov and
/// strafe.followOffset. Nothing writes to them. Everything derived from them
/// (orbit radius, neutral elevation, strafe FOV, the tilt cosine) is recomputed
/// per frame instead of cached in Awake, so dragging one during play takes
/// effect on the next frame, with no context menu and no restart.
///
/// "Pull Framing From VCams" survives, inverted: it copies the vcams' current
/// values INTO these fields, for composing in the Scene view and adopting the
/// result. It is an editor convenience, not part of startup.
///
/// Drive damping is the one value still authored on the vcam, since the
/// solution does not write it. It is re-snapshotted on every strafe ENTRY
/// rather than once in Awake, so a live edit survives the next strafe exit.
/// ------------------------------------------------
/// v2.0: SINGLE WRITE AUTHORITY.
///
/// Every feature used to write straight to the transposer, composer or lens in
/// sequence, and whoever ran last won. That was not a stylistic problem, it had
/// already destroyed two authored values:
///
///   • Drive Follow Offset Y. Authored (0, 4.5, -8.5). The pitch code rebuilt
///     the offset from scratch as y = |z|*sin(pitch), z = -|z|*cos(pitch),
///     discarding the authored Y and orbiting on a radius of 8.5 instead of the
///     authored 9.62. At the neutral elevation the camera actually sat at
///     y 3.59, z -7.70, so ride height authoring did nothing and the camera was
///     1.1m closer than the inspector claimed.
///   • Composer Target Offset Y. Authored (0, 1.5, 0). The shoulder-shift code
///     wrote new Vector3(0, 1, lookAhead) every frame with a hardcoded 1, so the
///     look point sat half a metre low, always.
///
/// The vehicle solved this exact problem once already: Foundation is the single
/// ATTITUDE authority, contributors hand it intent rather than applying their
/// own torque, and CLAUDE.md states the principle as "opposing forces cause
/// jitter, prefer mutual exclusion over tuning competitors against each other".
/// This is that shape applied to framing.
///
/// How it works now:
///   1. SEED a FramingSolution from the authored bases captured in Awake.
///   2. CONTRIBUTE. Each feature is a method that adds into the solution and
///      touches no Cinemachine state whatsoever.
///   3. COMMIT once, at the end of LateUpdate.
///
/// Adding a camera effect means adding a contributor. It cannot clobber another
/// effect, because it does not write anything.
///
/// WHAT IS DELIBERATELY NOT IN THIS CLASS: camera SHAKE. Impulse listeners on
/// the vcams apply post-body, downstream of everything here, so impulses and
/// framing already cannot fight. Folding shake into the solution would break
/// the very property that makes it safe. Shake lives in the impulse router.
///
/// ------------------------------------------------
/// v1.3 changes:
///   • Boost FOV kick, scaled by Propulsion.BoostLerp so the lens opens on the
///     same curve as the thrust. Written to BOTH vcams each frame, each against
///     its own base, because mode switching is by priority and the brain blends
///     them; writing only the active one dips FOV mid-boost on a strafe entry.
/// ------------------------------------------------
/// v1.2 changes:
///   • Heading stabilization proxy. The drive cam no longer follows the vehicle
///     transform directly: LockToTargetWithWorldUp derives yaw from the vehicle
///     rotation, and that decomposition flips 180 degrees when the nose pitches
///     through vertical (air-control flips), swinging the camera around the
///     vehicle. The drive cam follows a runtime "CameraHeadingProxy" that
///     copies vehicle position but carries yaw only: read from the transform
///     while the vehicle is upright within maxStableTilt, integrated from the
///     Rigidbody's world-Y angular velocity while tilted past it. Pure flips and
///     barrel rolls have ~zero world-Y angular velocity, so the camera holds its
///     heading while the vehicle rotates in front of it; deliberate mid-air yaw
///     still tracks. LookAt stays on the real vehicle so flips remain centered.
///     While air control is active (Propulsion.AirControlWeight), heading
///     tracking fades to zero entirely — mixed roll+pitch input wanders the
///     nose off-axis and would otherwise sway the camera sideways mid-roll;
///     the stunt gets a stable frame and heading reconverges on release.
///     Strafe cam is unaffected (air control is suppressed in strafe mode).
/// ------------------------------------------------
/// Manages all camera behavior for the hover vehicle using two Cinemachine
/// Virtual Cameras blended by a CinemachineBrain on the main camera.
///
/// Drive Mode (default):
///   • Third-person chase camera behind the vehicle.
///   • Right Stick Y orbits the camera in elevation AND pushes the look point
///     forward and up, so the stick means "show me further ahead" rather than
///     "get higher and stare at the roof".
///   • Look point also runs ahead with forward speed.
///   • During hard turns / drift, camera shifts to the opposite shoulder and
///     looks further ahead. Driven by Propulsion's DriftLerp.
///
/// Strafe Mode (Left Trigger held):
///   • Camera locks behind the vehicle, no shoulder shift.
///   • Zoom in (reduced follow distance baked on the vcam + FOV reduction).
///   • Vertical position damping only, so jumps soften without loosening aim.
///
/// Cinemachine 3.x Scene Setup (required):
///   1. Main Camera: add CinemachineBrain component.
///      • Default Blend: EaseInOut, 0.2s
///      • Update Method: SmartUpdate
///   2. Two CinemachineCamera GameObjects, NOT children of the vehicle,
///      named "VCam_Drive" and "VCam_Strafe".
///
///   VCam_Drive settings:
///      • CinemachineFollow (Body). Follow Offset is an OUTPUT; author resting
///        framing in framing.driveFollowOffset on this component instead.
///      • CinemachineRotationComposer (Aim). Tracked Object Offset is an
///        OUTPUT; author the base look point in framing.driveTargetOffset.
///      • Position/Rotation Damping ARE authored here. Nothing writes them.
///      • Lens FOV is an OUTPUT; author it in framing.baseFov.
///      • Priority: 10
///
///   VCam_Strafe settings:
///      • CinemachineFollow (Body). Follow Offset is an OUTPUT; author the
///        strafe pull-in in strafe.followOffset.
///      • CinemachineRotationComposer (Aim). Left entirely alone by this class.
///      • Damping is written every frame from strafe.verticalDamping; the
///        authored values are not used.
///      • Lens FOV is an OUTPUT; it is baseFov minus strafe.fovReduction.
///      • Priority: 10
///
///   Everything labelled OUTPUT above will visibly tick over during play. That
///   is the solution writing, not a bug, and it is why those fields are not the
///   place to tune from.
///
/// Priority management:
///   Both VCams sit at priority 10. The active one is bumped to 11.
/// </summary>
[ExecuteAlways]
public class HoverCameraController : MonoBehaviour
{
    // ── References ────────────────────────────────────────────────────────

    [Header("Cinemachine Virtual Cameras")]
    [Tooltip("The drive-mode virtual camera. See class header for Inspector setup.")]
    [SerializeField] private CinemachineCamera vcamDrive;

    [Tooltip("The strafe-mode virtual camera. See class header for Inspector setup.")]
    [SerializeField] private CinemachineCamera vcamStrafe;

    [Header("Vehicle")]
    [Tooltip("The vehicle root transform. Both VCams follow this.")]
    [SerializeField] private Transform vehicleTarget;

    [Header("Input + Modules")]
    [Tooltip("Whatever is driving this vehicle, so the camera can read look input from the same source. " +
             "Drag the vehicle's PlayerHoverInput here (any component implementing IHoverInputProvider works, " +
             "which is what lets an AI-driven vehicle be spectated with the same camera).")]
    [SerializeField] private MonoBehaviour inputProvider;

    [Tooltip("The vehicle's Propulsion component. Supplies drift, boost, strafe and air-control blends. " +
             "Without it the shoulder shift, boost FOV and look-ahead fade all stay flat.")]
    [SerializeField] private HoverController_Propulsion propulsion;

    // ── Tuning ────────────────────────────────────────────────────────────
    // Serializable sections, same shape as the vehicle's *Tuning classes.
    // Defaults are set to the values the scene was actually running before the
    // v2.0 refactor, so moving these fields into sections did not silently
    // revert anyone's tuning.

    [Header("Tuning")]
    [SerializeField] private CameraFramingTuning       framing       = new CameraFramingTuning();
    [SerializeField] private CameraLookTuning          look          = new CameraLookTuning();
    [SerializeField] private CameraBoostTuning         boost         = new CameraBoostTuning();
    [SerializeField] private CameraStrafeTuning        strafe        = new CameraStrafeTuning();
    [SerializeField] private CameraStabilizationTuning stabilization = new CameraStabilizationTuning();

    [Header("Editor")]
    [Tooltip("With the game stopped, push the resting pose onto the vcams every editor tick so " +
             "framing edits are visible immediately.\n\n" +
             "Turn this OFF to compose the other way round: the vcams are left alone, so you can " +
             "drag Follow Offset in the Scene view or inspector directly, then adopt the result " +
             "with the 'Pull Framing From VCams' context menu. With preview ON that workflow " +
             "cannot work, because the preview stamps the vcam back on the next tick.\n\n" +
             "No effect during play, where the solution always writes.")]
    [SerializeField] private bool previewInEditMode = true;

    [Tooltip("Which definitive state to preview with the game stopped: the framing you would see " +
             "once that input has fully blended in. Pick one, then tune with the mouse while " +
             "watching the Game view, instead of trying to hold a stick and drag a slider at once.\n\n" +
             "States are DERIVED from the tuning on this component, so there is nothing to author " +
             "and nothing that can fall out of sync. They are also fed through the real solver, not " +
             "a copy of it, so what you see is what the camera does.\n\n" +
             "Has NO effect during play, and cannot be left on in a way that affects the game. " +
             "Requires Preview In Edit Mode above.")]
    [SerializeField] private CameraPreviewState previewState = CameraPreviewState.Resting;

    // ── The Solution ──────────────────────────────────────────────────────

    /// <summary>
    /// One frame's resolved framing. Contributors add into this; nothing else
    /// writes to Cinemachine. Held by value so a contributor cannot stash a
    /// reference and mutate it after the commit.
    /// </summary>
    private struct FramingSolution
    {
        public Vector3 followOffset;
        public Vector3 targetOffset;
        public float   fov;

        /// <summary>
        /// What fraction of the requested look-ahead survived the framing guard,
        /// and the metres it removed. 1 and 0 mean it did not intervene.
        /// Carried on the solution rather than stashed in a field so a preview
        /// measurement cannot disturb the live camera's.
        /// </summary>
        public float guardScale;
        public float guardRemoved;

        /// <summary>
        /// Vertical position damping for the strafe rig, in seconds. Solved rather
        /// than read straight off the tuning, because it is rate-gated: see
        /// ContributeStrafeDamping. Unused by the drive solution.
        /// </summary>
        public float strafeVerticalDamping;
    }

    /// <summary>
    /// Everything the contributors are allowed to know about the world, in one
    /// place. Two producers fill it: GatherLiveInputs from the real signals
    /// while playing, BuildPreviewInputs from a chosen state with the game
    /// stopped.
    ///
    /// This exists so the preview and the live camera cannot disagree. A
    /// contributor that reads propulsion directly is a contributor a preview
    /// has to reimplement, and a reimplementation drifts.
    /// </summary>
    private struct FramingInputs
    {
        public float elevation;         // degrees above horizontal, absolute
        public float forwardSpeed;      // m/s along chassis forward
        public float driftLerp;         // 0..1

        // m/s along the camera's STABLE HEADING rather than along the chassis.
        // Only ForwardGate reads it. It exists because the chassis answer crosses
        // zero mid-flip while the craft is still travelling forward at full speed,
        // which is the v2.9 defect. Keep the two separate: look-ahead genuinely
        // wants the chassis number, since its offset is chassis-local.
        public float travelSpeed;

        // 0..1 direction gate on every boost term, ALREADY slew-limited. Carried
        // solved rather than derived in the contributors because the limit is
        // memory. See IntegrateForwardGate.
        public float forwardGate;

        // Metres the speed term pushes the look point ahead, ALREADY slew-limited.
        // Carried as a distance rather than derived from forwardSpeed inside the
        // contributor because the limit is on how fast the FRAMING may travel, and
        // that is a property of the look point rather than of the craft. See
        // IntegrateLookAheadDistance.
        public float lookAheadDistance;

        // Degrees the camera has swung around the craft while downed, ALREADY
        // integrated and latched. Carried solved for the same reason as the two
        // above: it has memory, and a contributor must stay pure.
        // See IntegrateDownedYaw.
        public float downedYaw;

        // Boost arrives as TWO numbers rather than as Propulsion.BoostLerp, because
        // BoostLerp only ever says "how far into boost am I right now" and two of
        // the four boost effects need memory. See IntegrateBoostEnvelope.
        public float boostHold;         // 0..1, sustained. Rises with the thrust, falls slower
        public float boostSurge;        // 0..1, the engage transient. Spikes then self-cancels

        public float airControlWeight;  // 0..1
        public float uprightness;       // vehicle up.y, 1 level, 0 on its side or inverted
        public float shoulderOffset;    // metres, lateral; settled value, not mid-ease
        public bool  strafing;

        // How much of the craft's weight the hover springs are actually carrying,
        // 1 grounded and 0 in free air. Only the strafe rig's damping reads it.
        // This is Foundation's own signal rather than anything derived here, for
        // the same reason Propulsion uses it: it is the project's answer to "am I
        // still on my springs", and it crossfades instead of switching.
        public float hoverSupport;
    }

    /// <summary>
    /// A solved state plus the answer to "does the craft actually fit in frame".
    /// Returned by EvaluateState for the inspector readout.
    ///
    /// The budget this measures is fixed and worth understanding before tuning
    /// against it. The craft sits at the CENTRE of the camera's orbit, so the
    /// angle from camera down to craft is always exactly the elevation angle,
    /// and no amount of distance changes that. The only way to keep the craft
    /// in frame while elevation rises is for the look axis to tip up by a
    /// similar amount. So:
    ///
    ///     elevation, minus how far the look axis tips up, must stay under
    ///     half the vertical field of view
    ///
    /// Pitch ceiling and look-ahead are therefore in direct competition for one
    /// number, and the margins below are what is left of it.
    ///
    /// Angles are measured to the craft's bounding SPHERE, not its origin, so
    /// "fits" means the whole hull rather than a point that happens to be
    /// inside the frustum.
    /// </summary>
    public struct FramingVerdict
    {
        public Vector3 followOffset;
        public Vector3 targetOffset;
        public float   fov;             // vertical, degrees
        public float   elevation;       // degrees above horizontal
        public float   orbitRadius;     // metres

        // Where the craft's CENTRE sits relative to the look axis, in degrees.
        // The diagnostic number: + vertical means below, + horizontal means right.
        public float verticalOffAxis;
        public float horizontalOffAxis;

        // Degrees of frame left over after the worst CORNER of the hull.
        // Negative means part of the craft is cropped.
        public float verticalMargin;
        public float horizontalMargin;

        // Half-frame, degrees. Vertical is fov/2; horizontal widens it by aspect.
        public float halfFovV;
        public float halfFovH;

        // Fraction of the requested look-ahead the framing guard allowed, and
        // the metres it removed. guardScale 1 means it did not intervene.
        public float guardScale;
        public float guardRemoved;

        /// <summary>Whole hull inside the frame, nothing cropped.</summary>
        public bool Fits => verticalMargin >= 0f && horizontalMargin >= 0f;

        /// <summary>
        /// The craft's centre is still on screen. This is the distinction that
        /// keeps the readout honest: clipping the rear underside at close range
        /// is normal framing, especially in strafe where the camera pulls in,
        /// while losing the centre means the craft is genuinely leaving the
        /// frame. A check that treats those the same reports a fault during
        /// ordinary driving and gets ignored, which is exactly how the movement
        /// gizmo's drive/drag warning had to be fixed once already.
        /// </summary>
        public bool CentreVisible =>
            Mathf.Abs(verticalOffAxis) <= halfFovV && Mathf.Abs(horizontalOffAxis) <= halfFovH;
    }

    /// <summary>
    /// Half-extents of the craft in its own space, from the 6.88 long x 3.81
    /// wide x 2.35 high collider union measured 2026-08-07 and recorded in
    /// CLAUDE.md. Local Z is forward, so the long axis is Z.
    ///
    /// The verdict tests the eight CORNERS of this box rather than a sphere
    /// around it, and the difference is not academic. A sphere has to use the
    /// half-diagonal, 4.10m, which at the resting distance of 8.94m subtends
    /// 27.3 degrees all by itself against a 32.5 degree half-frame. That model
    /// reports every state as off screen, including ones a screenshot shows
    /// fitting comfortably, because most of that radius is the craft's LENGTH,
    /// which points away from the camera and barely affects its footprint.
    ///
    /// The box is NOT centred on the vehicle origin, and assuming it was cost a
    /// full degree of false clipping. Measured from the collider union in
    /// vehicle-local space on 2026-08-09: centre (0.000, 0.877, 0.732), extents
    /// (1.907, 1.174, 3.439). So the hull sits 0.88m above and 0.73m forward of
    /// the transform, and a box centred on the origin hangs almost a metre
    /// below the craft. The corner that leaves frame first is the rear-bottom
    /// one, which is exactly the corner that misplacement moved.
    ///
    /// Hardcoded rather than measured at runtime on purpose: Collider.bounds is
    /// a WORLD-space AABB, so reading it back would inflate the moment the craft
    /// banks or flips, and the framing test needs an orientation-independent
    /// box. Re-measure if the hull changes.
    /// </summary>
    private static readonly Vector3 CraftBoundsCentre  = new Vector3(0f, 0.877f, 0.732f);
    private static readonly Vector3 CraftHalfExtents   = new Vector3(1.907f, 1.174f, 3.439f);

    // ── Private State ─────────────────────────────────────────────────────

    private IHoverInputProvider _input;

    // Drive orbit elevation in degrees above horizontal. Absolute, not a delta:
    // seeded from the authored offset and clamped to a range around it.
    private float _currentElevation;

    // Smoothed shoulder shift offset (local X).
    private float _shoulderOffset;

    // Boost envelope. _boostHold follows BoostLerp up and lags it down;
    // _boostSettle is a slow copy of _boostHold, and the gap between them is the
    // engage transient. See IntegrateBoostEnvelope for why BoostLerp alone
    // cannot express an overshoot or an asymmetric release.
    private float _boostHold;
    private float _boostSettle;

    // Slew-limited speed look-ahead, in metres. See IntegrateLookAheadDistance.
    private float _lookAheadDistance;

    // Slew-limited forward gate, 0..1. See IntegrateForwardGate.
    private float _forwardGate;

    // Downed camera orbit, degrees either side of directly behind the craft.
    // Positive swings the camera to the craft's right. See IntegrateDownedYaw.
    private float _downedYaw;

    // Latched copy of Foundation.IsDowned. IsDowned itself chatters during a
    // wipeout, so it is not safe to hand the stick over on. See IntegrateDownedYaw.
    private bool  _downedLatched;

    // Seconds since IsDowned last read true, for the latch above.
    private float _downedClearTimer;

    // Tracks previous strafe state for transition detection
    private bool _wasStrafingLastFrame;

    // Cached component references — avoid GetComponent per frame
    private CinemachineFollow          _driveTransposer;
    private CinemachineFollow          _strafeTransposer;
    private CinemachineRotationComposer _driveComposer;

    // ── Derived framing ───────────────────────────────────────────────────
    // Recomputed on read rather than cached in Awake. That is what makes the
    // tuning fields draggable during play: a cached derivation would freeze the
    // orbit geometry at whatever the fields held on frame one, which is exactly
    // the trap the seeds themselves used to be in. The cost is a magnitude and
    // an atan2 per frame.

    /// <summary>Orbit radius in metres: the length of the resting offset in the YZ plane.</summary>
    private float DriveOrbitRadius =>
        new Vector2(framing.driveFollowOffset.y, framing.driveFollowOffset.z).magnitude;

    /// <summary>
    /// Resting elevation in degrees above horizontal, the angle the stick's
    /// pitch ranges are measured from. Z is negative (behind the craft), so the
    /// angle is measured against -z.
    /// </summary>
    private float DriveNeutralElevation =>
        DriveOrbitRadius > 0.001f
            ? Mathf.Atan2(framing.driveFollowOffset.y, -framing.driveFollowOffset.z) * Mathf.Rad2Deg
            : 0f;

    /// <summary>Strafe FOV, authored as a reduction against the drive base rather than on the vcam.</summary>
    private float StrafeFov => Mathf.Max(20f, framing.baseFov - strafe.fovReduction);

    /// <summary>Cosine of maxStableTilt, the upright test in UpdateHeadingProxy.</summary>
    private float MinStableUpY => Mathf.Cos(stabilization.maxStableTilt * Mathf.Deg2Rad);

    // Drive cam damping, the one framing value still authored on the vcam
    // because the solution never writes it. Snapshotted on strafe ENTRY, not in
    // Awake, so an edit made mid-session survives the next strafe exit instead
    // of being reverted to whatever the value was at startup.
    private Vector3 _drivePositionDamping;
    private Vector3 _driveRotationDamping;

    // Heading stabilization proxy — the drive cam's actual Follow target.
    private Transform _headingProxy;

    /// <summary>
    /// Last well-conditioned horizontal travel heading, and whether one has been seen.
    /// Latched rather than recomputed per frame so the travel bound keeps working while
    /// the craft is falling too steeply for its horizontal velocity to define a
    /// direction. See UpdateHeadingProxy for why that case is the important one.
    /// </summary>
    private float _lastTravelYaw;
    private bool  _haveTravelYaw;

    // The heading-vs-travel offset captured when air control engaged, and the latch that
    // captures it once per stunt. The travel bound limits how far the heading may drift
    // FROM THIS, rather than from travel itself. See UpdateHeadingProxy.
    private float _airEntryTravelOffset;
    private bool  _airBoundActive;
    private float     _headingYaw;    // degrees, world yaw of the proxy
    private Rigidbody _vehicleRb;     // yaw-rate source while tilted, and speed source
    private HoverController_Foundation _vehicleFoundation;  // hover support, for strafe damping

    // ── Unity Lifecycle ───────────────────────────────────────────────────

    private void Awake()
    {
        // Edit mode takes none of this. Its setup is OnEnable, and its per-tick
        // work is LateUpdate's preview branch, neither of which needs an input
        // provider, a Rigidbody or a heading proxy.
        if (!Application.isPlaying) return;

        // A preview solo must never survive into play. Normally the domain
        // reload clears the static for us, but that reload is optional in
        // Enter Play Mode Options and this is a one-line guarantee.
        ClearSolo();

        _input = inputProvider as IHoverInputProvider;

        if (_input == null)
        {
            Debug.LogError("[HoverCameraController] inputProvider is null or missing IHoverInputProvider.", this);
            enabled = false;
            return;
        }

        if (vcamDrive == null || vcamStrafe == null)
        {
            Debug.LogError("[HoverCameraController] One or both VCam references are unassigned.", this);
            enabled = false;
            return;
        }

        if (vehicleTarget == null)
        {
            Debug.LogError("[HoverCameraController] vehicleTarget is unassigned.", this);
            enabled = false;
            return;
        }

        if (propulsion == null)
        {
            Debug.LogWarning("[HoverCameraController] Propulsion reference unassigned. Shoulder shift, " +
                             "boost FOV and the look-ahead inversion fade will all stay flat.", this);
        }

        // Heading proxy: the drive cam follows this instead of the vehicle so
        // its yaw frame stays stable through air-control flips (see v1.2 notes).
        _vehicleRb         = vehicleTarget.GetComponent<Rigidbody>();
        _vehicleFoundation = vehicleTarget.GetComponent<HoverController_Foundation>();

        _headingProxy = new GameObject("CameraHeadingProxy").transform;
        _headingProxy.position = vehicleTarget.position;
        _headingYaw            = vehicleTarget.eulerAngles.y;
        _headingProxy.rotation = Quaternion.Euler(0f, _headingYaw, 0f);

        // Assign follow/look targets. Drive cam follows the stable-yaw proxy but
        // still LOOKS at the real vehicle, so flips stay centered on screen.
        // Strafe cam keeps the vehicle for both: it must pitch with the nose,
        // and air control is suppressed in strafe mode so it can never flip.
        vcamDrive.Follow  = _headingProxy;
        vcamDrive.LookAt  = vehicleTarget;
        vcamStrafe.Follow = vehicleTarget;
        vcamStrafe.LookAt = vehicleTarget;

        if (!ResolveRigComponents())
        {
            Debug.LogError("[HoverCameraController] VCams must have a CinemachineFollow component. " +
                           "Add CinemachineFollow to both VCam GameObjects.", this);
            enabled = false;
            return;
        }

        // Start parked at the resting elevation the tuning describes.
        _currentElevation = DriveNeutralElevation;

        // Seed the damping snapshot. Refreshed on every strafe entry thereafter.
        SnapshotDriveDamping();

        // Drive cam: LockToTargetWithWorldUp — follows yaw only, ignores pitch and roll.
        var driveSettings = _driveTransposer.TrackerSettings;
        driveSettings.BindingMode = BindingMode.LockToTargetWithWorldUp;
        _driveTransposer.TrackerSettings = driveSettings;

        // Strafe cam: LockToTargetNoRoll — follows yaw AND pitch, ignores roll only.
        // Required so the camera physically tilts with the vehicle nose in strafe mode.
        var strafeSettings = _strafeTransposer.TrackerSettings;
        strafeSettings.BindingMode = BindingMode.LockToTargetNoRoll;
        _strafeTransposer.TrackerSettings = strafeSettings;

        // Drive cam active at start
        SetDriveCamActive();
    }

    /// <summary>
    /// Caches the Cinemachine stages off both vcams. Safe in either mode and
    /// cheap enough to re-run, which matters in the editor: a domain reload
    /// clears these and there is no Awake in edit mode to put them back.
    /// Returns false when the rig is not wired up yet, which is a normal state
    /// for a component being assembled rather than an error worth logging.
    /// </summary>
    private bool ResolveRigComponents()
    {
        if (vcamDrive == null || vcamStrafe == null) return false;

        // v3 uses GetComponent, not GetCinemachineComponent.
        _driveTransposer  = vcamDrive.GetComponent<CinemachineFollow>();
        _strafeTransposer = vcamStrafe.GetComponent<CinemachineFollow>();
        _driveComposer    = vcamDrive.GetComponent<CinemachineRotationComposer>();

        return _driveTransposer != null && _strafeTransposer != null;
    }

    private void OnEnable()
    {
        if (Application.isPlaying) return;

        ApplyEditorPreview();
    }

    /// <summary>
    /// Pushes the resting pose onto the vcams with the game stopped, so a
    /// framing edit is visible immediately in the Scene and Game views.
    ///
    /// The resting pose is just the seed. Every contributor evaluates to zero
    /// with the craft parked and no stick input, and the pitch orbit reproduces
    /// the authored offset exactly at the neutral elevation, so this needs none
    /// of the runtime state that does not exist yet in edit mode.
    ///
    /// Targets, binding modes and priorities are deliberately NOT written here.
    /// Those are scene authoring, and stamping them from a preview would make
    /// the component quietly rewrite parts of the scene nobody asked it to.
    /// Which rig the Game view shows is handled by Cinemachine's Solo instead,
    /// which is editor-only and unserialized, so previewing a strafe state
    /// cannot modify the scene.
    /// </summary>
    private void ApplyEditorPreview()
    {
        // The authoritative guard, and it lives HERE rather than only on the
        // callers for a reason that cost a live camera once.
        //
        // OnValidate checked isPlaying and then queued a delayCall. The check
        // was true when the work was QUEUED and false when it RAN: edit a value,
        // hit Play before the deferred call fires, and the lambda executes with
        // the game already running. It re-set Solo after Awake had cleared it,
        // stranding the camera away from the craft for the whole session.
        //
        // A guard on a deferred caller guards the wrong moment. Guarding the
        // work itself is the only version that cannot be got around.
        if (Application.isPlaying)
        {
            ClearSolo();
            return;
        }

        if (!previewInEditMode)
        {
            ClearSolo();
            return;
        }

        if (!ResolveRigComponents()) return;

        FramingInputs inp = BuildPreviewInputs(previewState);

        // Mirror the integrated state onto the fields the rest of the class
        // reads, so CurrentElevation and the inspector readout agree with what
        // was just committed.
        _currentElevation = inp.elevation;
        _shoulderOffset   = inp.shoulderOffset;
        _downedYaw        = inp.downedYaw;

        CommitDrive(SolveDriveFraming(inp));
        CommitStrafe(SolveStrafeFraming(inp));

#if UNITY_EDITOR
        // Solo the rig the state implies, rather than trusting whichever
        // priority happens to be serialized. Set explicitly in both directions
        // so a strafe state cannot leave the drive rig showing, or vice versa.
        Unity.Cinemachine.CinemachineCore.SoloCamera = inp.strafing ? vcamStrafe : vcamDrive;
#endif
    }

    /// <summary>
    /// Drops Cinemachine's editor Solo override.
    ///
    /// Solo is STATIC and survives a selection change, so every exit from the
    /// preview has to clear it or the Game view stays pinned to a rig with no
    /// visible reason why. Called when the preview is switched off, when the
    /// component is disabled, and on entering play, since play mode with domain
    /// reload disabled would otherwise carry a stale solo into the session.
    /// </summary>
    private static void ClearSolo()
    {
#if UNITY_EDITOR
        Unity.Cinemachine.CinemachineCore.SoloCamera = null;
#endif
    }

    private void OnDisable()
    {
        ClearSolo();
    }

#if UNITY_EDITOR
    /// <summary>
    /// Applies a preview the moment a tuning value changes in the inspector,
    /// rather than waiting for the editor's next tick.
    ///
    /// Deferred through delayCall because OnValidate runs during serialization,
    /// where writing to other components is not safe. The null check is not
    /// paranoia: a queued delayCall outlives the object if the component is
    /// deleted or a domain reload lands first.
    ///
    /// It also outlives EDIT MODE, which is subtler and did real damage. The
    /// isPlaying test below is about not queueing needless work; it is NOT what
    /// keeps the preview out of play mode, because by the time the lambda runs
    /// the answer may have changed. ApplyEditorPreview enforces that itself.
    /// </summary>
    private void OnValidate()
    {
        if (Application.isPlaying) return;

        UnityEditor.EditorApplication.delayCall += () =>
        {
            if (this == null) return;
            if (Application.isPlaying) return;   // play may have started since queueing

            ApplyEditorPreview();
            UnityEditor.SceneView.RepaintAll();
        };
    }
#endif

    /// <summary>
    /// Copies the vcams' current framing INTO the tuning sections, for anyone
    /// who would rather compose in the Scene view and adopt the result.
    ///
    /// This is the inverse of the v2.0 "Recapture" that it replaces. The tuning
    /// fields are the authority now, so the direction of the copy had to flip:
    /// the old version pulled from the vcams every startup, which is what kept
    /// the resting framing out of the inspector in the first place.
    ///
    /// Resolves its own components so it works with the object not playing.
    ///
    /// TURN OFF "Preview In Edit Mode" BEFORE COMPOSING, or this does nothing
    /// useful: the preview stamps the vcams from these same fields every tick,
    /// so a Scene-view edit is reverted before you can adopt it and the pull
    /// reads back exactly what it would have written.
    ///
    /// Refused during play. See the guard for why that is a correctness matter
    /// rather than a convenience one.
    /// </summary>
    [ContextMenu("Pull Framing From VCams")]
    private void PullFramingFromVCams()
    {
        if (!ResolveRigComponents())
        {
            Debug.LogWarning("[HoverCameraController] Cannot pull framing: a VCam reference is " +
                             "unassigned or missing CinemachineFollow.", this);
            return;
        }

        // Refused during play, and this one is not pedantry. In play the vcam
        // holds the SOLVED pose, so a pull would bake whatever look-ahead,
        // shoulder shift and boost happened to be active at that instant into
        // the resting values, permanently and invisibly. There is no legitimate
        // play-mode use either: the vcam cannot be composed on during play,
        // because the solution overwrites it every frame.
        if (Application.isPlaying)
        {
            Debug.LogWarning("[HoverCameraController] Pull refused during play: the vcams hold the " +
                             "solved pose, not the resting pose, so this would bake transient " +
                             "look-ahead, drift and boost into the base framing. Tune the fields " +
                             "directly instead.", this);
            return;
        }

        if (previewInEditMode)
        {
            Debug.LogWarning("[HoverCameraController] Pull is a no-op while Preview In Edit Mode is on: " +
                             "the vcams already hold these values because the preview writes them. " +
                             "Turn the preview off, compose on the vcam, then pull.", this);
            return;
        }

        framing.driveFollowOffset = _driveTransposer.FollowOffset;
        strafe.followOffset       = _strafeTransposer.FollowOffset;
        framing.baseFov           = vcamDrive.Lens.FieldOfView;

        if (_driveComposer != null)
            framing.driveTargetOffset = _driveComposer.TargetOffset;

        // Elevation is measured from the neutral, and the neutral just moved.
        _currentElevation = DriveNeutralElevation;

        Debug.Log($"[HoverCameraController] Pulled framing: drive {framing.driveFollowOffset}, " +
                  $"look {framing.driveTargetOffset}, strafe {strafe.followOffset}, fov {framing.baseFov}. " +
                  $"Orbit {DriveOrbitRadius:F2}m at {DriveNeutralElevation:F1} deg.", this);

#if UNITY_EDITOR
        UnityEditor.EditorUtility.SetDirty(this);
#endif
    }

    /// <summary>
    /// Records the drive cam's authored damping so strafe exit can put it back.
    /// Called on strafe ENTRY rather than once in Awake, which is what lets a
    /// damping value edited mid-session survive the next exit.
    /// </summary>
    private void SnapshotDriveDamping()
    {
        var s = _driveTransposer.TrackerSettings;
        _drivePositionDamping = s.PositionDamping;
        _driveRotationDamping = s.RotationDamping;
    }

    private void LateUpdate()
    {
        // Edit mode: preview the resting pose and nothing else. This branch is
        // what catches a Follow Offset dragged in the inspector, since the
        // editor ticks ExecuteAlways components while the pointer is moving.
        if (!Application.isPlaying)
        {
            ApplyEditorPreview();
            return;
        }

        if (_input == null) return;

        UpdateHeadingProxy();

        bool strafing = _input.StrafeHeld;

        // ── Mode transitions ──────────────────────────────────────────────
        if (strafing != _wasStrafingLastFrame)
        {
            if (strafing) SetStrafeCamActive();
            else          SetDriveCamActive();
        }
        _wasStrafingLastFrame = strafing;

        // ── Integrate state ───────────────────────────────────────────────
        // Separated from solving on purpose: these advance smoothed values over
        // time and must run exactly once per frame, whereas a contributor is a
        // pure read that could run twice without consequence.
        if (!strafing)
        {
            IntegrateElevation();
            IntegrateShoulderShift();
        }

        // Outside the strafe branch on purpose: boost applies in both modes, and
        // an envelope that only advanced in drive would freeze mid-transient the
        // moment you raised the crosshair.
        IntegrateBoostEnvelope();

        // Also outside the strafe branch, and for the same reason: freezing the
        // look point while the crosshair is up would let it arrive as a step the
        // moment drive resumed, which is the defect this limit exists to remove.
        IntegrateLookAheadDistance();
        IntegrateForwardGate();

        // Outside the strafe branch for a third reason: the latch must keep
        // counting while the crosshair is up, or raising it mid-wipeout would
        // freeze the hold timer and leave the camera stuck on the stick.
        IntegrateDownedYaw();

        // ── Solve and commit ──────────────────────────────────────────────
        FramingInputs inputs = GatherLiveInputs(strafing);

        CommitDrive(SolveDriveFraming(inputs));
        CommitStrafe(SolveStrafeFraming(inputs));
    }

    // ── Framing: inputs ───────────────────────────────────────────────────

    /// <summary>
    /// Reads the world once per frame. The ONLY place the framing stage touches
    /// propulsion, the Rigidbody or the vehicle transform.
    /// </summary>
    private FramingInputs GatherLiveInputs(bool strafing)
    {
        Vector3 velocity = _vehicleRb != null ? _vehicleRb.linearVelocity : Vector3.zero;

        float forwardSpeed = _vehicleRb != null
                               ? Vector3.Dot(velocity, vehicleTarget.forward)
                               : 0f;

        // Falls back to the chassis answer rather than to zero, deliberately: a
        // missing proxy should degrade to the old behaviour, not gate every boost
        // cue off permanently.
        float travelSpeed  = _headingProxy != null
                               ? Vector3.Dot(velocity, _headingProxy.forward)
                               : forwardSpeed;

        return new FramingInputs
        {
            elevation        = _currentElevation,
            forwardSpeed     = forwardSpeed,
            travelSpeed      = travelSpeed,

            // Off the integrated distance, not recomputed from forwardSpeed here,
            // for the same reason boost is read off its envelope: the memory has
            // already been advanced for this frame.
            lookAheadDistance = _lookAheadDistance,
            forwardGate       = _forwardGate,
            downedYaw         = _downedYaw,

            driftLerp        = propulsion != null ? propulsion.DriftLerp        : 0f,

            // Read off the integrated envelope, not off BoostLerp. This method is
            // the only place the world is read, and the envelope has already been
            // advanced for this frame by IntegrateBoostEnvelope.
            boostHold        = _boostHold,
            boostSurge       = Mathf.Max(0f, _boostHold - _boostSettle),

            airControlWeight = propulsion != null ? propulsion.AirControlWeight : 0f,
            uprightness      = vehicleTarget != null ? vehicleTarget.up.y : 1f,
            shoulderOffset   = _shoulderOffset,
            strafing         = strafing,
            hoverSupport     = _vehicleFoundation != null ? _vehicleFoundation.HoverSupport : 1f
        };
    }

    /// <summary>
    /// The settled inputs for a definitive state. Everything is derived from
    /// the tuning on this component, so a state cannot describe a situation the
    /// tuning does not produce.
    ///
    /// SETTLED, not mid-blend, which is the point of the whole feature: the
    /// shoulder offset is written at its terminal value rather than eased
    /// toward it, and elevation sits exactly on its limit rather than
    /// travelling there.
    ///
    /// Note Boost and FullThrottle share a speed, and that is correct rather
    /// than lazy: speedLookAheadReference is 60, equal to the vehicle's top
    /// speed, and the speed term clamps at 1, so everything at or above the
    /// reference produces identical look-ahead. Boost differs only in FOV.
    /// </summary>
    private FramingInputs BuildPreviewInputs(CameraPreviewState state)
    {
        float neutral  = DriveNeutralElevation;
        float fullFast = look.speedLookAheadReference;

        var inp = new FramingInputs
        {
            elevation   = neutral,
            uprightness = 1f
        };

        // Set ONLY by a state whose travel deliberately disagrees with its nose.
        // Null means "the craft is going where it is pointing", which is true of
        // every state above the disagreement block and is the safe default: a
        // state added later that forgets this still renders an honest pose
        // rather than a silently contradictory one.
        float? travelOverride   = null;
        float  downedYawPreview = 0f;

        switch (state)
        {
            case CameraPreviewState.Resting:
                break;

            case CameraPreviewState.StickFullUp:
                inp.elevation = neutral + framing.pitchUpRange;
                break;

            case CameraPreviewState.StickFullDown:
                inp.elevation = neutral - framing.pitchDownRange;
                break;

            case CameraPreviewState.FullThrottle:
                inp.forwardSpeed = fullFast;
                break;

            // Settled: the transient has decayed and only the sustained terms
            // remain. What a long boost actually looks like.
            case CameraPreviewState.Boost:
                inp.forwardSpeed = fullFast;
                inp.boostHold    = 1f;
                break;

            // The peak of the engage transient. Surge at 1 is the theoretical
            // maximum rather than a number the live envelope is guaranteed to
            // reach, since the gap the surge measures depends on how snappy
            // boostBlendSeconds is. So this is the worst case for framing, which
            // is exactly what a preview state is for.
            case CameraPreviewState.BoostPeak:
                inp.forwardSpeed = fullFast;
                inp.boostHold    = 1f;
                inp.boostSurge   = 1f;
                break;

            case CameraPreviewState.ClimbAtSpeed:
                inp.elevation    = neutral + framing.pitchUpRange;
                inp.forwardSpeed = fullFast;
                break;

            case CameraPreviewState.DriftLeft:
                // Drifting left means turning left, and the camera shifts to the
                // opposite shoulder. IntegrateShoulderShift negates the turn sign,
                // so this reproduces its settled result rather than guessing.
                inp.forwardSpeed   = fullFast;
                inp.driftLerp      = 1f;
                inp.shoulderOffset = framing.shoulderShiftAmount;
                break;

            case CameraPreviewState.DriftRight:
                inp.forwardSpeed   = fullFast;
                inp.driftLerp      = 1f;
                inp.shoulderOffset = -framing.shoulderShiftAmount;
                break;

            case CameraPreviewState.Strafe:
                inp.strafing = true;
                break;

            // Needs a forward speed, and the lack of one was a real bug rather
            // than an omission of detail: every boost term is multiplied by the
            // reverse gate, which reads forward speed, so at rest the gate is 0
            // and this state rendered as plain Strafe. It had done so silently
            // since the gate was added in v2.4.
            //
            // Worth knowing while judging it: the gate asks for FORWARD speed,
            // and strafe can travel at full speed in any direction. Boosting
            // sideways while aiming therefore produces no lens change at all.
            // That may be right, since the gate exists to suppress the reverse
            // case, but nobody has judged it.
            case CameraPreviewState.StrafeBoost:
                inp.strafing     = true;
                inp.forwardSpeed = fullFast;
                inp.boostHold    = 1f;
                break;

            case CameraPreviewState.Inverted:
                inp.forwardSpeed     = fullFast;
                inp.uprightness      = 0f;
                inp.airControlWeight = 1f;
                break;

            // Nose forward, travel backward. Only the SIGN of travel matters:
            // the gate saturates long before the magnitude does, so using the
            // camera reference rather than the vehicle reverse cap costs nothing
            // and keeps this state derived from camera tuning alone.
            case CameraPreviewState.ReverseBoost:
                inp.forwardSpeed = -fullFast;
                inp.boostHold    = 1f;
                travelOverride   = -fullFast;
                break;

            // Broadside: moving at full speed with the nose square across it.
            // forwardSpeed 0 collapses the speed look-ahead while travelSpeed
            // fullFast leaves the gate wide open, so the boost terms stay live.
            // The two disagreeing is the entire point of the state.
            case CameraPreviewState.NoseAcrossTravel:
                inp.forwardSpeed = 0f;
                inp.boostHold    = 1f;
                travelOverride   = fullFast;
                break;

            // At the limit of the downed orbit. No speed: a downed craft has
            // come to rest, and giving it one would quietly re-introduce the
            // aligned-travel assumption this block exists to break.
            case CameraPreviewState.Downed:
                inp.uprightness  = 0f;
                downedYawPreview = framing.downedYawRange;
                break;
        }

        // Travel follows the nose UNLESS a state explicitly said otherwise. That
        // default is the honest one and it is also the safe one: a state added
        // later cannot forget this and silently render with its boost gated off,
        // which is exactly how StrafeBoost sat wrong from v2.4 until someone
        // noticed. Disagreeing now requires saying so, which is the right way
        // round -- for most of this file's life the two were pinned equal with no
        // way to separate them at all, and two defects hid in that gap.
        inp.travelSpeed = travelOverride ?? inp.forwardSpeed;

        // Settled by definition: a preview state is a destination, never a moment
        // in transit, so the slew limit has already been paid off and the look
        // point is wherever that state's speed asks for. Assigned out here rather
        // than per case for the reason immediately above — a state added later
        // cannot forget it and silently render with its look-ahead at zero.
        inp.lookAheadDistance = SpeedLookAheadTarget(inp.forwardSpeed);

        // Settled for the same reason. Note this is what makes a preview state
        // honest about a GATED case: the warning above about a state that omits a
        // gate input silently rendering the UNGATED case applies here directly,
        // and assigning outside the switch is what stops it happening again.
        inp.forwardGate = ForwardGateTarget(inp.travelSpeed);

        // Zero unless a state swung the camera. Same pattern and same reason as
        // travelOverride above.
        inp.downedYaw = downedYawPreview;

        return inp;
    }

    // ── Framing: solve ────────────────────────────────────────────────────

    /// <summary>
    /// Builds the drive camera's framing. Seeded from the authored base, then
    /// each contributor adds to it. Nothing here touches Cinemachine, and
    /// nothing here reads the world: everything arrives in the inputs.
    /// </summary>
    private FramingSolution SolveDriveFraming(in FramingInputs inp)
    {
        var s = new FramingSolution
        {
            followOffset = framing.driveFollowOffset,
            targetOffset = framing.driveTargetOffset,
            fov          = framing.baseFov,
            guardScale   = 1f
        };

        ContributePitchOrbit(ref s, inp);
        ContributeLookAhead(ref s, inp);
        ContributeShoulderShift(ref s, inp);
        ContributeBoostLens(ref s, inp);
        ContributeBoostPullback(ref s, inp);

        // AFTER the two boost terms and the shoulder shift, because it rotates the
        // finished rig rather than contributing to it: everything that offsets the
        // camera should swing around with it. Still BEFORE the guard, which has to
        // measure the pose that actually ships.
        ContributeDownedYawOrbit(ref s, inp);

        // LAST, and it has to be. It measures the finished frame, so it needs
        // the final camera position from the orbit and the final FOV from boost.
        // Boost widening the lens genuinely buys frame back, and running the
        // guard first would throw that away.
        ContributeFramingGuard(ref s, inp);

        return s;
    }

    /// <summary>
    /// Strafe framing. The strafe rig is deliberately minimal: its follow offset
    /// is the authored pull-in and its composer is left entirely alone, because
    /// the crosshair is yaw and pitch and anything that moves the look point
    /// moves the player's aim. Only FOV is contributed to.
    ///
    /// Boost contributes NOTHING here, lens included. That is an owner rule set
    /// 2026-08-11 after a playtest: strafe mode gets full crosshair authority with
    /// zero modifiers. The previous version contributed the lens and withheld only
    /// the pull-back, on the reasoning that FOV does not move the rig under the
    /// crosshair.
    ///
    /// MEASURED 2026-08-13, and the rule is better founded than that reasoning was.
    /// The obvious defence of an FOV kick is that a centred crosshair aims along the
    /// camera axis, which no lens change can move. **This reticle is not centred.**
    /// It is a world point projected to screen (`VehicleHUD.SyncReticle`), and it
    /// sits about 477px above centre, roughly 42% of the way out. A projected point
    /// moves radially with tan(fov/2), so the shipped +4 degree kick would drag the
    /// reticle **35 pixels** across the screen at that offset, before the +6 degree
    /// overshoot is counted. That is a direct aim modifier, not merely a rescale.
    ///
    /// So do not restore the lens here as a way of making boost readable in strafe.
    /// It genuinely moves the player's aim. The cue has to come from something that
    /// touches neither the rig nor the lens; see `TODO.md` M.12.
    ///
    /// This also settles the open question in TODO 2.6 about the boost forward-gate
    /// reading forward speed only, so boosting sideways in strafe produced no lens
    /// change. That was right, and it now goes further: no lens change in strafe at
    /// all, in any direction.
    /// </summary>
    private FramingSolution SolveStrafeFraming(in FramingInputs inp)
    {
        var s = new FramingSolution
        {
            followOffset = strafe.followOffset,
            targetOffset = Vector3.zero,   // unused; strafe composer is not written
            fov          = StrafeFov,
            guardScale   = 1f              // nothing tips the strafe aim, so nothing to guard
        };

        ContributeStrafeDamping(ref s, inp);

        return s;
    }

    /// <summary>
    /// Vertical damping for the strafe rig, applied only while the craft is OFF
    /// its hover springs.
    ///
    /// The damping softens jumps, which is what it was added for and which the
    /// owner confirmed as good. Applied flat, it also lagged something nobody was
    /// thinking about: throttling forward in strafe lifts the craft about 0.56m,
    /// and a camera that lags that lift sits LOW relative to the chassis, which
    /// flattens the angle it looks down at. Measured 2026-08-11 at full throttle in
    /// strafe: the rig sat 0.91m below its authored height and the view tipped up
    /// 8.70 degrees. The arithmetic closes exactly -- atan(3.50/6.00) is 30.3
    /// degrees and atan(2.59/6.53) is 21.6 -- so that was the whole of the reported
    /// "camera looks up in strafe", and neither the boost lens (FOV moved 0.00) nor
    /// aim pitch (the chassis moved 0.03 degrees) was involved.
    ///
    /// GATED ON HOVER SUPPORT, AND THE FIRST ATTEMPT GATED ON VERTICAL SPEED, WHICH
    /// WAS WRONG AND SHIPPED BRIEFLY. The reasoning for rate sounded clean -- a jump
    /// is a fast vertical transient, the throttle lift is a slow one -- and it fell
    /// apart on slopes, which the owner found within minutes. Driving undulating
    /// ground produces vertical speeds up to 13.92 m/s against a 6 m/s saturation,
    /// so the gate swept its whole range, crossed its midpoint 5 times in 8 seconds
    /// and changed at up to 39 per second. A damping coefficient that moves that
    /// fast generates motion of its own: the rig starts lagging, then snaps to
    /// catch up, repeatedly. Smooth on flat ground, bumpy on every slope.
    ///
    /// Hover support is the right signal because it asks the question that actually
    /// separates the two cases: not "how fast am I moving vertically" but "am I
    /// still standing on my springs". Climbing a slope, the answer is yes and the
    /// rig tracks rigidly. Jumping, it goes to zero and the damping arrives in
    /// full. Measured over the same terrain it changes at a mean rate of 0.09 per
    /// second against the rate gate's 0.23, peaks at 7.6 against 12.7, and still
    /// reaches a full 1.000 during a real jump, so nothing is lost.
    ///
    /// It is also the signal Foundation and Propulsion already use for exactly this
    /// distinction, and it crossfades rather than switching, so there is no cliff
    /// to tune around. Cresting a bump dips support to about 0.28 and lets a little
    /// damping in, which is correct: a crest IS a small jump.
    /// </summary>
    private void ContributeStrafeDamping(ref FramingSolution s, in FramingInputs inp)
    {
        s.strafeVerticalDamping = strafe.verticalDamping * (1f - Mathf.Clamp01(inp.hoverSupport));
    }

    // ── Framing: contributors ─────────────────────────────────────────────

    /// <summary>
    /// Orbits the camera in elevation around the target, on the radius and from
    /// the neutral angle the authored Follow Offset describes.
    ///
    /// The look-point half of the stick's job is in ContributeLookAhead. Keeping
    /// them apart is the entire point: coupling them is what made raising the
    /// camera angle it DOWN at the roof rather than out at the horizon.
    /// </summary>
    private void ContributePitchOrbit(ref FramingSolution s, in FramingInputs inp)
    {
        float neutral = DriveNeutralElevation;
        float radius  = DriveOrbitRadius;

        // Boom out as the stick raises the camera. A pure arc swings the camera
        // closer AND more nearly overhead at the same time, which is what drops
        // the craft off the bottom of the frame; this buys some of that back
        // without touching the pitch ceiling or the look-ahead. Up only, and
        // zero by default, so it cannot disturb the resting pose.
        if (framing.pitchUpDistanceGain > 0f && framing.pitchUpRange > 0.001f)
        {
            float up = Mathf.Clamp01((inp.elevation - neutral) / framing.pitchUpRange);
            radius  += framing.pitchUpDistanceGain * up;
        }

        float rad = inp.elevation * Mathf.Deg2Rad;

        s.followOffset.y =  radius * Mathf.Sin(rad);
        s.followOffset.z = -radius * Mathf.Cos(rad);
    }

    /// <summary>
    /// Pushes the look point ahead of the craft, from two independent sources:
    /// forward speed, and how far the stick has raised the camera.
    ///
    /// Both are faded out when the craft is inverted or under air control.
    /// TargetOffset is expressed in the LookAt target's LOCAL space, so it
    /// rotates with the chassis; without the fade a flip would swing the aim
    /// point through the floor. This mirrors what UpdateHeadingProxy already
    /// does with heading tracking, and for the same underlying reason.
    /// </summary>
    private void ContributeLookAhead(ref FramingSolution s, in FramingInputs inp)
    {
        float authority = 1f;

        if (look.fadeLookAheadWhenInverted)
        {
            float upright = Mathf.Clamp01(inp.uprightness);
            float airFade = 1f - inp.airControlWeight;
            authority     = upright * airFade;
        }

        if (authority <= 0f) return;

        // Speed term. Arrives already solved and already slew-limited, so this
        // stays a pure contributor: the rate limit is memory, and memory belongs
        // in the integrate stage. Forward component only, so reversing does not
        // push the look point out behind the direction of travel — that sign is
        // handled upstream, since Clamp01 of a negative speed is zero.
        s.targetOffset.z += inp.lookAheadDistance * authority;

        // Stick term. Normalised against whichever range the stick is inside, so
        // asymmetric up/down ranges still both reach full effect at their limit.
        float delta = inp.elevation - DriveNeutralElevation;
        float range = delta >= 0f ? framing.pitchUpRange : framing.pitchDownRange;

        if (range > 0.001f)
        {
            float t = Mathf.Clamp(delta / range, -1f, 1f);

            s.targetOffset.y += look.pitchLookLift  * t * authority;
            s.targetOffset.z += look.pitchLookAhead * t * authority;
        }
    }

    /// <summary>
    /// During hard turns and drift the camera shifts to the outside shoulder and
    /// the look point runs further ahead. Drift is an aiming tool in this game,
    /// so this is framing the thing the player is actually buying with it.
    /// </summary>
    private void ContributeShoulderShift(ref FramingSolution s, in FramingInputs inp)
    {
        s.followOffset.x += inp.shoulderOffset;
        s.targetOffset.z += framing.shoulderLookAheadAmount * inp.driftLerp;
    }

    /// <summary>
    /// Swings the whole finished rig around the craft while it is downed.
    ///
    /// A rotation of the follow offset, not an addition to it. Everything the
    /// other contributors put into that offset -- the pitch orbit's radius and
    /// height, the shoulder shift, the boost pull-back -- rides around with the
    /// camera, which is what "orbit" has to mean for the pose to stay coherent
    /// at 90 degrees off-axis.
    ///
    /// Rotating about world up rather than the craft's own up is deliberate and
    /// is the whole reason this reads at all when the craft is upside down. The
    /// drive rig is bound LockToTargetWithWorldUp, so the offset lives in a frame
    /// that yaws with the chassis but never pitches or rolls with it. Rotating
    /// about Vector3.up in that frame keeps the camera level with the horizon
    /// while it circles, so the player sees the world the right way up while
    /// looking at an inverted wreck. Using the craft's up would roll the horizon
    /// upside down at exactly the moment the player is trying to read it.
    ///
    /// The LOOK point is untouched on purpose. The craft stays centred and the
    /// camera moves around it, which is what makes this a look-around rather
    /// than a pan off into space. Speed look-ahead has already faded itself out
    /// by this point (ContributeLookAhead fades on uprightness, and a downed
    /// craft has none), so there is nothing left pushing the aim off the wreck.
    /// </summary>
    private void ContributeDownedYawOrbit(ref FramingSolution s, in FramingInputs inp)
    {
        if (Mathf.Abs(inp.downedYaw) < 0.01f)
            return;

        s.followOffset = Quaternion.AngleAxis(inp.downedYaw, Vector3.up) * s.followOffset;
    }

    /// <summary>
    /// Gate that kills every boost term while reversing.
    ///
    /// Boost engages in reverse too (Propulsion v1.2 made that work deliberately)
    /// and the camera was reacting to it. A wider lens and a camera falling back
    /// are both forward-rush cues; there is no rush to sell while backing up, so
    /// it read as the camera acting at random.
    ///
    /// Gates on DIRECTION rather than scaling by speed. Scaling would delay every
    /// boost cue at onset, and they work precisely because they are transients
    /// arriving on the same curve as the thrust.
    ///
    /// Reads travelSpeed, NOT forwardSpeed, and the distinction is the whole of
    /// the v2.9 fix. "Am I backing up" is a question about TRAVEL, and the chassis
    /// only answers it while the nose points along the travel line. A flip breaks
    /// that equivalence: the nose sweeps past perpendicular, the chassis answer
    /// crosses zero, and the gate slams shut on a craft still doing 134 m/s
    /// forwards. Measured at 7ms per crossing, twice per rotation, which is a
    /// visible hitch rather than a transition. The stable heading has neither
    /// problem, for the same reason v2.7 bounded the proxy against travel.
    ///
    /// SLEW-LIMITED since 2026-08-17; see IntegrateForwardGate. This method is the
    /// TARGET, and contributors read the limited value off FramingInputs.
    /// </summary>
    private float ForwardGateTarget(float travelSpeed) =>
        boost.forwardGateSpeed > 0f
            ? Mathf.Clamp01(travelSpeed / boost.forwardGateSpeed)
            : 1f;

    /// <summary>
    /// The lens half of boost, in BOTH modes: a sustained widening plus an
    /// overshoot that decays while boost is still held.
    ///
    /// The overshoot is the part that does the work. A wider lens held steady is
    /// a sustained cue and the eye adapts to it within about a second, after
    /// which it is just an odd lens. The spike is what the player actually reads
    /// as speed, and fovIncrease is only what is left behind afterwards.
    /// </summary>
    private void ContributeBoostLens(ref FramingSolution s, in FramingInputs inp)
    {
        float gate = inp.forwardGate;

        s.fov += boost.fovIncrease  * inp.boostHold  * gate;
        s.fov += boost.fovOvershoot * inp.boostSurge * gate;
    }

    /// <summary>
    /// The rig half of boost, DRIVE ONLY: the camera falls back while boost is
    /// held, and falls back further still at the moment of engaging.
    ///
    /// Not applied to strafe, and that is the same rule the rest of the strafe
    /// rig follows. The strafe crosshair is yaw and pitch, so anything that moves
    /// the rig moves the player's aim. Strafe gets the lens and nothing else.
    ///
    /// Applied along Z rather than by extending the orbit radius, deliberately.
    /// Straight back makes the view further out AND slightly flatter, which shows
    /// more horizon and more ground streaming past the edges; booming along the
    /// radius would hold the elevation angle and only add distance. Both read as
    /// "further away", only one reads as "faster".
    ///
    /// Runs after ContributePitchOrbit, which rebuilds Y and Z from the orbit, so
    /// this genuinely lands on the finished position rather than being overwritten.
    /// The framing guard runs after both and sees the result, and since pulling
    /// back can only ever HELP the craft fit, it can only make the guard less
    /// likely to engage.
    /// </summary>
    private void ContributeBoostPullback(ref FramingSolution s, in FramingInputs inp)
    {
        float gate = inp.forwardGate;

        // Negative Z is behind the craft, so pulling back is subtraction.
        s.followOffset.z -= boost.zPullBack    * inp.boostHold  * gate;
        s.followOffset.z -= boost.zLagOnEngage * inp.boostSurge * gate;
    }

    /// <summary>
    /// Keeps the craft inside the frame, by scaling back look-ahead until it
    /// fits. The last word on framing, and the only contributor that removes
    /// rather than adds.
    ///
    /// It exists because pitch look-ahead and speed look-ahead both tip the aim
    /// and neither knows the other is doing it. Any pair of values that fits at
    /// rest can be overrun by adding the other term, which is what throttling
    /// at full stick-up does. That is not a tuning failure, it is two
    /// independent contributors spending one budget, so no pair of numbers can
    /// fix it and a limit is the only thing that can.
    ///
    /// Scales the whole delta from the authored look point, so shoulder
    /// look-ahead gives way alongside the other two. They all consume the same
    /// budget, and singling one out would just make it the one that breaks the
    /// frame.
    ///
    /// Never touches the orbit, the resting framing or the FOV. If the base
    /// framing itself does not fit, this cannot rescue it, and the inspector
    /// readout says so rather than pretending otherwise.
    ///
    /// KNOWN BLIND SPOT, measured 2026-08-11 and deliberately NOT patched here.
    /// This method reasons about the SOLVED camera pose, and Cinemachine damping
    /// sits downstream of the commit, so during a fast stick-up the rig trails its
    /// request by up to 2.498m (solved Y 9.552 against an actual 8.645) with the
    /// aim lagging too on 0.5s of composer damping. The guard therefore certifies
    /// a frame that is not the one being rendered.
    ///
    /// Feeding it the rig's real position was tried and REVERTED, because the
    /// measurements did not support it: on a pinned, repeatable manoeuvre at 60 m/s
    /// it changed the worst margin by -0.0009, and at 45 m/s it was slightly worse
    /// than leaving the guard off entirely. Correcting the position while still
    /// assuming an ideal aim makes the model less self-consistent, not more.
    ///
    /// What is actually unexplained: clipping reproduces readily while driving real
    /// terrain (measured to -0.0404 and -0.1090 of the viewport) and does NOT
    /// reproduce on flat ground at any speed with the craft held level. So the
    /// missing variable is chassis attitude, not camera lag. Note this method
    /// measures the hull as a bounding SPHERE, which is attitude-independent, while
    /// the thing that leaves frame is the rear-bottom corner of an oriented box --
    /// the exact corner CLAUDE.md warns is the first to go. Investigate that before
    /// touching anything here.
    /// </summary>
    private void ContributeFramingGuard(ref FramingSolution s, in FramingInputs inp)
    {
        if (framing.minFrameMargin <= 0f) return;

        Vector3 basis = framing.driveTargetOffset;
        Vector3 delta = s.targetOffset - basis;

        if (delta.sqrMagnitude < 1e-8f) return;

        float aspect = CurrentAspect;

        // Never demand more margin than the resting pose can supply. If the base
        // framing is already tight, the honest goal is "do not make it worse",
        // not an absolute number the geometry cannot reach. Without this clamp
        // a base sitting a fraction of a degree short collapses look-ahead to
        // zero, which is a cliff rather than a limit.
        float baseMargin = MinMargin(s.followOffset, basis, s.fov, aspect);
        float target     = Mathf.Min(framing.minFrameMargin, baseMargin);

        if (MinMargin(s.followOffset, s.targetOffset, s.fov, aspect) >= target)
            return;

        // Bisect the surviving fraction. Margin is monotonic in it (more tip,
        // less frame), so this always converges. Twelve halvings resolve to
        // under 0.03% of the request, far below anything visible, for a handful
        // of dot products each.
        float lo = 0f, hi = 1f;

        for (int i = 0; i < 12; i++)
        {
            float mid = (lo + hi) * 0.5f;

            if (MinMargin(s.followOffset, basis + delta * mid, s.fov, aspect) >= target)
                lo = mid;
            else
                hi = mid;
        }

        s.targetOffset = basis + delta * lo;
        s.guardScale   = lo;
        s.guardRemoved = delta.magnitude * (1f - lo);
    }

    private static float MinMargin(Vector3 follow, Vector3 target, float fov, float aspect)
    {
        FramingVerdict v = Measure(follow, target, fov, aspect);
        return Mathf.Min(v.verticalMargin, v.horizontalMargin);
    }

    /// <summary>Aspect of whatever is rendering, so the horizontal check matches the real frame.</summary>
    private static float CurrentAspect
    {
        get
        {
            Camera c = Camera.main;
            return c != null && c.aspect > 0.01f ? c.aspect : 16f / 9f;
        }
    }

    // ── Framing: state integration ────────────────────────────────────────

    /// <summary>
    /// Right Stick Y drives orbit elevation. Released, it springs back to the
    /// authored neutral. Elevation is clamped to a range AROUND that neutral
    /// rather than to absolute angles, so re-authoring the Follow Offset moves
    /// the limits with it instead of silently reshaping the travel.
    /// </summary>
    private void IntegrateElevation()
    {
        float lookY   = _input.CameraLookY;
        float neutral = DriveNeutralElevation;

        // Suppress pitch in proportion to how hard the player is steering.
        //
        // This is thumb geometry, not preference. Right stick X is yaw and
        // right stick Y is camera pitch, so holding a line through a drift
        // means holding X hard over, and no thumb pushes a stick perfectly
        // horizontally. The vertical bleed rides along and swings the camera
        // while the player is trying to hold that line.
        //
        // Scaled on turn magnitude rather than on DriftLerp on purpose: the
        // same bleed happens in any hard corner, and scoping it to drift would
        // leave the annoyance everywhere else.
        if (framing.turnBleedSuppression > 0f)
        {
            float steering = Mathf.Clamp01(Mathf.Abs(_input.TurnInput));
            lookY *= 1f - framing.turnBleedSuppression * steering;
        }

        if (Mathf.Abs(lookY) > framing.lookDeadzone)
        {
            _currentElevation += lookY * framing.pitchSensitivity * Time.deltaTime * 60f;
        }
        else
        {
            _currentElevation = Mathf.Lerp(_currentElevation, neutral,
                                           framing.pitchRecenterSpeed * Time.deltaTime);
        }

        _currentElevation = Mathf.Clamp(
            _currentElevation,
            neutral - framing.pitchDownRange,
            neutral + framing.pitchUpRange);
    }

    /// <summary>
    /// Right Stick X swings the camera AROUND the craft while it is downed and
    /// the player has nothing else to do with that thumb. Released, or once
    /// control comes back, it springs back to directly behind.
    ///
    /// Deliberately the same arithmetic as IntegrateElevation: accumulate while
    /// the stick is past the shared deadzone, Lerp home when it is not, clamp to
    /// a range. Matching downedYawSensitivity to pitchSensitivity therefore gives
    /// the two axes the same feel, which is the point.
    ///
    /// THE LATCH IS NOT OPTIONAL, and this is the part that is easy to get wrong.
    /// IsDowned is not a clean interval: measured 2026-08-17 on a scripted 25 m/s
    /// wipeout it dropped and re-engaged THREE TIMES inside the first 0.9 seconds,
    /// because a bouncing craft keeps breaking ground contact and the lockout
    /// reads contact directly. Wiring the stick to IsDowned hands the camera over
    /// and snatches it back twice before the craft has even settled. The latch
    /// engages instantly and releases only after downedCameraHold seconds of
    /// continuous recovery, which costs nothing on the way in and hides the gaps.
    ///
    /// Only the drive rig reads the result. The strafe rig is left alone on
    /// purpose -- its composer is the player's aim, and you cannot aim while
    /// downed anyway.
    /// </summary>
    private void IntegrateDownedYaw()
    {
        // Advance the latch first, so the stick and the release agree this frame.
        bool downedNow = _vehicleFoundation != null && _vehicleFoundation.IsDowned;

        if (downedNow)
        {
            _downedLatched    = true;
            _downedClearTimer = 0f;
        }
        else if (_downedLatched)
        {
            _downedClearTimer += Time.deltaTime;

            if (_downedClearTimer >= framing.downedCameraHold)
                _downedLatched = false;
        }

        // Disabled outright, or not downed: unwind. Unwinding rather than holding
        // matters more here than it does for pitch, because control returns at 35
        // degrees of tilt with the craft already moving, so the camera can be a
        // long way off-axis at the exact moment the player needs to drive.
        if (framing.downedYawSensitivity <= 0f || !_downedLatched)
        {
            _downedYaw = Mathf.Lerp(_downedYaw, 0f,
                                    framing.downedYawRecenterSpeed * Time.deltaTime);
            return;
        }

        float lookX = _input.TurnInput;

        if (Mathf.Abs(lookX) > framing.lookDeadzone)
        {
            _downedYaw += lookX * framing.downedYawSensitivity * Time.deltaTime * 60f;
        }
        else
        {
            _downedYaw = Mathf.Lerp(_downedYaw, 0f,
                                    framing.downedYawRecenterSpeed * Time.deltaTime);
        }

        _downedYaw = Mathf.Clamp(_downedYaw, -framing.downedYawRange, framing.downedYawRange);
    }

    /// <summary>
    /// Eases the shoulder offset toward the drift target. Negative turn input
    /// means drifting right, which shifts the camera left.
    /// </summary>
    private void IntegrateShoulderShift()
    {
        if (propulsion == null) return;

        float turnSign = Mathf.Sign(_input.TurnInput);
        float target   = -turnSign * framing.shoulderShiftAmount * propulsion.DriftLerp;

        _shoulderOffset = Mathf.Lerp(_shoulderOffset, target,
                                     framing.shoulderShiftLerpSpeed * Time.deltaTime);
    }

    /// <summary>
    /// Advances the boost envelope: the sustained level and the engage transient.
    ///
    /// WHY THIS EXISTS. Propulsion.BoostLerp is a pure statement of "how far into
    /// boost am I right now". It rises, holds at 1, falls. Two of the four boost
    /// effects cannot be written as a function of it at all:
    ///
    ///   an FOV overshoot has to exceed the sustained value and come back DOWN
    ///   while boost is still held, and BoostLerp is flat at 1 by then;
    ///
    ///   a release slower than the entry has to know which direction it is
    ///   travelling, and a single 0..1 number does not carry that.
    ///
    /// So the camera keeps its own memory. Two values, and the transient is the
    /// gap between them:
    ///
    ///   _boostHold    tracks BoostLerp EXACTLY on the way up, so the camera and
    ///                 the thrust still cannot disagree about when boost started,
    ///                 which was the whole reason the old FOV kick scaled by
    ///                 BoostLerp. On the way down it lags, at releaseSpeed.
    ///
    ///   _boostSettle  a slow copy of _boostHold in both directions.
    ///
    ///   surge = hold - settle
    ///
    /// Measuring the transient as a value minus a slower copy of itself is worth
    /// the trick: it needs no timers and no edge detection, and it CANCELS ITSELF.
    /// Hold boost and settle catches up, the gap closes to zero, and the spike is
    /// over without anything having to remember to end it. A decaying counter
    /// would need a reset path for re-engaging mid-decay; this has none to get
    /// wrong.
    ///
    /// It also scales with how snappy the thrust ramp is, for free: a shorter
    /// boostBlendSeconds opens a wider gap and produces a bigger camera kick.
    /// That is the correct coupling. The camera should be more emphatic about a
    /// boost that arrives more suddenly.
    /// </summary>
    private void IntegrateBoostEnvelope()
    {
        float target = propulsion != null ? propulsion.BoostLerp : 0f;

        // Asymmetric by construction. Rising is not smoothed at all, because
        // BoostLerp is already the authored ramp and smoothing it again would
        // add lag the tuning profile did not ask for.
        _boostHold = target >= _boostHold
            ? target
            : Mathf.Lerp(_boostHold, target, boost.releaseSpeed * Time.deltaTime);

        _boostSettle = Mathf.Lerp(_boostSettle, _boostHold, boost.settleSpeed * Time.deltaTime);
    }

    /// <summary>
    /// Where the speed term WANTS the look point, before any limit. The single
    /// expression of that mapping, so the live integrator and the preview states
    /// cannot answer the question differently.
    /// </summary>
    private float SpeedLookAheadTarget(float forwardSpeed) =>
        look.speedLookAheadReference > 0f
            ? look.speedLookAheadMax * Mathf.Clamp01(forwardSpeed / look.speedLookAheadReference)
            : 0f;

    /// <summary>
    /// Advances the speed look-ahead under a slew limit.
    ///
    /// WHY THIS EXISTS. The look point used to be a pure function of speed,
    /// recomputed every frame with no memory, so it travelled exactly as fast as
    /// the craft accelerated. That is fine everywhere except one case, and the
    /// 2026-08-17 measurement isolated it by running the same lane three times:
    ///
    ///   flooring it from rest    6.00m of look-point swing, no lens change
    ///   boosting at speed        1.91m of swing, plus the lens and the pull-back
    ///   boosting from rest       6.00m of swing, AND the lens and the pull-back
    ///
    /// Each of the first two was already judged good. The third is the only case
    /// carrying both ingredients, and it is the one that was reported as jarring.
    /// So the defect was never in the boost package — measured identical at rest
    /// and at speed — it was that boost is the only thing that drags the whole
    /// look-ahead swing through in the same third of a second the package lands in.
    ///
    /// A slew limit is the right instrument rather than smoothing, because the
    /// quantity that separates the good cases from the bad one is a PEAK RATE
    /// (7.7 m/s versus 11.5 m/s over 50ms windows), and a lerp cannot bound a
    /// peak rate: its speed scales with the gap, so a 0-to-6m gap starts fast no
    /// matter what constant is chosen.
    ///
    /// Symmetric on purpose. Retracting is limited too, which additionally stops
    /// the look point snapping back the full six metres in a single frame when a
    /// collision takes the craft from top speed to a standstill.
    ///
    /// Reads the Rigidbody directly, as the other integrators read propulsion and
    /// input directly. GatherLiveInputs owns the world for the SOLVE stage; the
    /// integrate stage necessarily runs before it.
    ///
    /// NOTE: seeds from zero, so a craft that begins life already at speed eases
    /// its look point out over roughly speedLookAheadMax / speedLookAheadSlew
    /// seconds. Harmless today, since the craft starts at rest and respawn is
    /// still a stub (TODO 1.1). Worth a seed if respawn ever hands back a moving
    /// craft.
    /// </summary>
    private void IntegrateLookAheadDistance()
    {
        float forwardSpeed = _vehicleRb != null && vehicleTarget != null
            ? Vector3.Dot(_vehicleRb.linearVelocity, vehicleTarget.forward)
            : 0f;

        float target = SpeedLookAheadTarget(forwardSpeed);

        _lookAheadDistance = look.speedLookAheadSlew > 0f
            ? Mathf.MoveTowards(_lookAheadDistance, target, look.speedLookAheadSlew * Time.deltaTime)
            : target;
    }

    /// <summary>
    /// Advances the forward gate under a slew limit.
    ///
    /// WHY THIS EXISTS. forwardGateSpeed is 2 m/s, deliberately, so the gate is a
    /// DIRECTION test rather than a speed ramp — see ForwardGateTarget for why
    /// scaling by speed would be the wrong instrument. The cost of that narrow
    /// band is that the gate behaves as a STEP against anything that reverses,
    /// and reversing under boost is not exotic: boost in reverse works on purpose,
    /// so travel speed crossing zero while boost is held is ordinary play.
    ///
    /// Measured 2026-08-17, boost pinned at 1.00 through a forward-to-reverse
    /// flick: the gate went 1.000 to 0.000 in 23ms, carrying FOV 69.0 to 65.0 and
    /// the rig 0.49m, which is about 21 m/s of camera travel. Flicking back
    /// snapped the whole package on again the same way, and that oscillation is
    /// what the owner reported. With the engage surge still live rather than
    /// decayed, the same 23ms would carry about 7.4 degrees and 3m.
    ///
    /// The limit is on the RESULT, not on the threshold, and that split is the
    /// point. Widening forwardGateSpeed would soften the step too, but it would
    /// buy that by turning a direction test into a speed ramp, which is exactly
    /// what its tooltip argues against and what TuningLog rejected for 0.20.
    ///
    /// Symmetric, because the reported artifact is an oscillation and limiting
    /// only the closing edge would leave the re-opening snap intact.
    ///
    /// IT IS NOT FREE AT ENGAGE, and the cost is larger than the arithmetic first
    /// suggested, because the gate multiplies the OVERSHOOT as well as the
    /// sustained terms. Measured from a standstill: the gate takes about 0.31s to
    /// open, and through the first quarter second the lens sits up to 1.8 degrees
    /// narrower than unslewed. What is NOT affected is the peak — 72.48 against
    /// 72.43 — because the overshoot crests at 0.35s, by which time the gate has
    /// finished opening. So the ramp changes shape and the transient still lands
    /// at full strength, which is the part TuningLog says does the selling.
    ///
    /// That cost pushes in the same direction as 0.20 rather than against it: a
    /// slightly softer first third to an engage from a standstill is what that
    /// item wanted. It is still a change to a feel that was signed off, so it is
    /// called out here rather than buried.
    /// </summary>
    private void IntegrateForwardGate()
    {
        float travelSpeed = _vehicleRb != null
            ? Vector3.Dot(_vehicleRb.linearVelocity,
                          _headingProxy != null ? _headingProxy.forward
                                                : (vehicleTarget != null ? vehicleTarget.forward : Vector3.forward))
            : 0f;

        float target = ForwardGateTarget(travelSpeed);

        _forwardGate = boost.forwardGateSlew > 0f
            ? Mathf.MoveTowards(_forwardGate, target, boost.forwardGateSlew * Time.deltaTime)
            : target;
    }

    // ── Framing: commit ───────────────────────────────────────────────────
    // The only place in this class that writes to Cinemachine.

    private void CommitDrive(FramingSolution s)
    {
        ApplyFollowOffset(_driveTransposer, s.followOffset);

        if (_driveComposer != null && _driveComposer.TargetOffset != s.targetOffset)
            _driveComposer.TargetOffset = s.targetOffset;

        ApplyLensFov(vcamDrive, s.fov);
    }

    /// <summary>
    /// Writes a follow offset only when it actually changed. Vector3's equality
    /// tolerance works out around a hundredth of a millimetre, far below
    /// anything visible in camera framing.
    ///
    /// Skipping the no-op write is a rounding error at runtime and the whole
    /// point in EDIT mode, where an unconditional per-tick write would leave
    /// the scene permanently dirty and the save prompt permanently armed.
    /// </summary>
    private static void ApplyFollowOffset(CinemachineFollow follow, Vector3 offset)
    {
        if (follow.FollowOffset != offset)
            follow.FollowOffset = offset;
    }

    private void CommitStrafe(FramingSolution s)
    {
        ApplyFollowOffset(_strafeTransposer, s.followOffset);

        ApplyLensFov(vcamStrafe, s.fov);

        // Damping is written per frame rather than on mode entry, so
        // strafe.verticalDamping is draggable during play like everything else.
        // It also closes an asymmetry: entry zeroed this rig's damping and
        // nothing ever restored it, so the authored values on VCam_Strafe were
        // only ever in force until the first strafe of the session.
        //
        // Play mode only. Damping describes how the rig CHASES a moving target,
        // so it shows nothing in a still preview, and writing it in edit mode
        // would dirty the scene on open for no visible gain.
        if (!Application.isPlaying) return;

        // Rate-gated by ContributeStrafeDamping rather than read flat off the
        // tuning, so the throttle lift no longer drags the crosshair. Committing a
        // SOLVED value keeps this the same shape as every other output here.
        var wanted = new Vector3(0f, s.strafeVerticalDamping, 0f);
        var ts     = _strafeTransposer.TrackerSettings;

        if (ts.PositionDamping != wanted || ts.RotationDamping != Vector3.zero)
        {
            ts.PositionDamping = wanted;
            ts.RotationDamping = Vector3.zero;
            _strafeTransposer.TrackerSettings = ts;
        }
    }

    /// <summary>
    /// Writes a field of view onto a vcam. Lens is a struct property, so it has
    /// to be copied out, modified and assigned back; the equality check skips
    /// that round trip on the frames where nothing moved, which is most of them.
    /// </summary>
    private static void ApplyLensFov(CinemachineCamera vcam, float fov)
    {
        var lens = vcam.Lens;

        if (Mathf.Approximately(lens.FieldOfView, fov)) return;

        lens.FieldOfView = fov;
        vcam.Lens        = lens;
    }

    // ── Drive Mode: Heading Stabilization ─────────────────────────────────

    /// <summary>
    /// Keeps the heading proxy on the vehicle's position with a yaw that stays
    /// well-behaved through flips. Yaw read directly from the transform is only
    /// well-conditioned while the vehicle is reasonably upright; past
    /// maxStableTilt (nose through vertical mid-flip) the euler decomposition
    /// flips 180 degrees — exactly the camera swing this proxy prevents.
    /// </summary>
    private void UpdateHeadingProxy()
    {
        _headingProxy.position = vehicleTarget.position;

        // While air control is active, the camera should NOT track heading at
        // all: mixed roll+pitch input makes the nose wander off-axis, and once
        // rolled toward 90 degrees, pitch input rotates about a near-vertical
        // axis — real world-Y angular velocity the integrator would faithfully
        // follow, swinging the camera sideways mid-roll. A stunt wants a stable
        // frame. Authority fades tracking to zero at full air control and
        // restores it as the blend releases (drift release or landing); any
        // heading change made mid-stunt is then caught up via the converge path.
        float trackAuthority = 1f - (propulsion != null ? propulsion.AirControlWeight : 0f);

        bool uprightEnough = vehicleTarget.up.y >= MinStableUpY;

        float step;

        if (uprightEnough)
        {
            // Exponential converge: error is ~0 in normal driving so tracking is
            // effectively instant; after a flip lands with a changed heading it
            // catches up smoothly instead of snapping.
            float t = 1f - Mathf.Exp(-stabilization.headingSyncSpeed * trackAuthority * Time.deltaTime);
            step = Mathf.DeltaAngle(_headingYaw, Mathf.LerpAngle(_headingYaw, vehicleTarget.eulerAngles.y, t));
        }
        else if (_vehicleRb != null)
        {
            // Tilted past the stable range: integrate the true yaw rate instead.
            // A pure pitch flip or barrel roll has ~zero world-Y angular
            // velocity, so the heading holds; yaw during non-air-control tumbles
            // (EMP hits) still tracks.
            step = _vehicleRb.angularVelocity.y * Mathf.Rad2Deg * trackAuthority * Time.deltaTime;
        }
        else
        {
            // No Rigidbody on the target: heading simply holds while tilted.
            step = 0f;
        }

        float desired = _headingYaw + step;

        // TRAVEL BOUND. The rate ceiling below caps how fast the heading may unwind,
        // and on its own that was not the fix: measured, it converted a snap into 0.6s
        // of sustained pan, because `trackAuthority` freezes the heading absolutely and
        // nothing bounded how far it could drift before the unwinding started. Divergence
        // reached 126 degrees, and no catch-up rate makes 126 degrees feel good.
        //
        // Bounding against the DIRECTION OF TRAVEL rather than the nose is what makes a
        // bound safe here. The chassis yaw is exactly the signal that is untrustworthy
        // mid-stunt: a flip sweeps the nose through vertical, which both swings the real
        // heading and makes the euler decomposition jump (measured at up to 140 degrees
        // in a single frame, always airborne with air control held). Travel heading has
        // neither problem, and on a ballistic arc it is very nearly constant because
        // gravity acts only on the vertical component. So the stunt still gets its still
        // frame, and there is little left to recentre on landing because the chassis
        // realigns with travel anyway.
        //
        // Gated on air control because that is the only state that freezes tracking, and
        // because bounding against travel during ordinary driving would fight drift,
        // where pointing the nose off the racing line is the entire manoeuvre.
        // The travel heading is LATCHED, and refreshed whether or not the bound is
        // currently applying. The first version recomputed it inside the air-control
        // gate and simply skipped the bound whenever horizontal speed fell under the
        // threshold, which turned out to disable it in precisely the case it was built
        // for. A flip bleeds off horizontal speed, so the craft finishes one plummeting
        // almost straight down: measured at a real landing, 5 m/s of horizontal against
        // 60 of descent, which sat under the old 8 m/s floor. The bound switched itself
        // off and the divergence went back to accumulating freely, which is why the
        // owner reported the remaining stutter as happening "on landing after flips".
        // Verified by injection: at 5 m/s horizontal the proxy held 120 degrees off
        // travel for an entire fall and the bound never fired, while the same test at
        // 25 m/s pulled it to the limit immediately.
        //
        // Latching is better than merely lowering the floor because ANY floor has this
        // failure mode, and because the latched value stays correct through it: with no
        // horizontal force acting, the heading a falling craft had a moment ago is still
        // the heading it has. Refreshed outside the air-control gate so engaging air
        // control mid-flight cannot adopt a stale heading from the previous jump.
        if (_vehicleRb != null)
        {
            Vector3 flatVel = _vehicleRb.linearVelocity;
            flatVel.y = 0f;

            float minSpeed = stabilization.travelHeadingMinSpeed;
            if (flatVel.sqrMagnitude >= minSpeed * minSpeed)
            {
                _lastTravelYaw = Mathf.Atan2(flatVel.x, flatVel.z) * Mathf.Rad2Deg;
                _haveTravelYaw = true;
            }
        }

        float airWeight = propulsion != null ? propulsion.AirControlWeight : 0f;

        // The bound limits DRIFT SINCE THE STUNT BEGAN, not absolute offset from travel.
        //
        // It used to clamp the absolute offset, which assumed the craft takes off pointing
        // roughly where it is going. Take off sliding BACKWARDS and that assumption inverts:
        // the offset is already ~180 degrees at entry, so the clamp had to move the heading
        // 140 degrees to satisfy itself, and it did so at the rate ceiling, over about 0.75s,
        // in the middle of the player's barrel roll. The owner reported exactly that, and
        // their own trace caught it: vehicle yaw pinned at 117.13 for the whole roll while the
        // proxy swung 117 -> 255.55, stopping dead at travel_div = -40.00, which is this
        // limit. Frame times were 3.2-4.6ms throughout, so it was never a hitch.
        //
        // Confirmed by A/B on an identical airborne roll: travelling forward swung the camera
        // 10.5 degrees, travelling backward swung it 91.2.
        //
        // Anchoring to the entry offset keeps the original fix intact -- a stunt that changes
        // the craft's heading still cannot let the proxy drift more than the limit, which is
        // what stopped the 126-degree divergence and the landing stutter -- while making entry
        // free. Enter aligned and this behaves exactly as before, since the entry offset is
        // then ~0.
        if (airWeight > 0f && _haveTravelYaw)
        {
            if (!_airBoundActive)
            {
                _airBoundActive       = true;
                _airEntryTravelOffset = Mathf.DeltaAngle(_lastTravelYaw, _headingYaw);
            }

            float limit = stabilization.headingMaxTravelDivergence;
            float off   = Mathf.DeltaAngle(_lastTravelYaw, desired);
            float drift = Mathf.DeltaAngle(_airEntryTravelOffset, off);

            if (drift > limit || drift < -limit)
                desired = _lastTravelYaw + _airEntryTravelOffset + Mathf.Clamp(drift, -limit, limit);
        }
        else if (airWeight <= 0f)
        {
            _airBoundActive = false;
        }

        // RATE CEILING, applied ONCE to the whole move rather than to the branch step.
        // Both branches are proportional to an error or to a raw angular velocity with no
        // upper bound of their own, and the travel bound above can also jump if the
        // velocity direction swings. Saturating the total is what guarantees none of the
        // three can produce a snap, whatever the others do. Small errors still settle
        // instantly; large ones take longer rather than arriving faster.
        float maxStep = stabilization.headingCatchUpMaxRate * Time.deltaTime;
        _headingYaw += Mathf.Clamp(Mathf.DeltaAngle(_headingYaw, desired), -maxStep, maxStep);

        _headingProxy.rotation = Quaternion.Euler(0f, _headingYaw, 0f);
    }

    // ── Mode Switching ────────────────────────────────────────────────────

    /// <summary>
    /// Activates the drive camera by giving it priority advantage.
    /// CinemachineBrain handles the blend automatically.
    /// </summary>
    private void SetDriveCamActive()
    {
        vcamDrive.Priority  = 11;
        vcamStrafe.Priority = 10;

        // Restore Inspector-authored damping — strafe mode changed it.
        var s = _driveTransposer.TrackerSettings;
        s.PositionDamping = _drivePositionDamping;
        s.RotationDamping = _driveRotationDamping;
        _driveTransposer.TrackerSettings = s;
    }

    /// <summary>
    /// Activates the strafe camera. Rotation damping is zeroed so the camera is
    /// bolted to vehicle heading with no lag, which is what aiming needs.
    ///
    /// Position damping is zeroed on X and Z only. The VERTICAL axis keeps
    /// strafe.verticalDamping, because fully locked vertical is what made jumps
    /// read as rigid, and softening it does not touch aim: the crosshair is yaw
    /// and pitch, both of which live in rotation. That write now happens in
    /// CommitStrafe every frame; all this does is switch priority.
    ///
    /// The drive rig's damping is snapshotted HERE, on the way out, so whatever
    /// is on the vcam at the moment of entry is what comes back on exit. Taking
    /// it in Awake instead meant a damping value edited mid-session was quietly
    /// reverted to the startup value by the next strafe.
    /// </summary>
    private void SetStrafeCamActive()
    {
        vcamStrafe.Priority = 11;
        vcamDrive.Priority  = 10;
        _shoulderOffset     = 0f;

        SnapshotDriveDamping();
    }

    private void OnDestroy()
    {
        // The heading proxy is a runtime-created scene object; clean it up with
        // us. It is never created in edit mode, but ExecuteAlways means this
        // callback now runs there too, and Destroy is a no-op outside play, so
        // the mode has to be checked rather than assumed.
        if (_headingProxy == null) return;

        if (Application.isPlaying) Destroy(_headingProxy.gameObject);
        else                       DestroyImmediate(_headingProxy.gameObject);
    }

    // ── Public API ────────────────────────────────────────────────────────

    /// <summary>
    /// True when strafe camera is active. Read by UI or other systems
    /// that need to know current camera mode.
    /// </summary>
    public bool IsInStrafeMode => _input != null && _input.StrafeHeld;

    /// <summary>
    /// Current drive orbit elevation in degrees above horizontal, and the
    /// neutral it springs back to. Exposed for debug readouts and for the
    /// re-tune pass that the v2.0 change to relative pitch limits requires.
    /// </summary>
    public float CurrentElevation  => _currentElevation;
    public float NeutralElevation  => DriveNeutralElevation;

    /// <summary>
    /// Call immediately AFTER teleporting the vehicle. Holds the camera at the
    /// same relative pose across the jump instead of letting it fly the length
    /// of the level to catch up.
    ///
    /// Two things go stale on a warp and only one of them is ours. The proxy
    /// position is hard-assigned from the vehicle every update, so it follows a
    /// teleport for free. What does NOT follow is the damping state inside
    /// CinemachineFollow, which still holds the pre-jump world position and
    /// would spend the next second interpolating across the map. That is what
    /// OnTargetObjectWarped exists to clear, and it wants the DELTA, not the
    /// destination.
    ///
    /// Yaw is snapped here rather than slewed. The rate limit at the end of
    /// UpdateHeadingProxy exists so a flip cannot whip the orbit around, and it
    /// is right for a flip and wrong for a teleport: at maxStep the camera
    /// would grind to the new heading over the better part of a second while
    /// the craft sits perfectly still, which reads as a bug rather than a reset.
    ///
    /// The travel latches are dropped rather than kept. Both describe a
    /// trajectory that no longer exists, and clearing them leaves exactly the
    /// state a fresh session starts in.
    ///
    /// Debug utility. Nothing in the shipped loop teleports the craft; if
    /// something ever does, it has to call this too.
    /// </summary>
    /// <param name="positionDelta">New vehicle position minus the old one.</param>
    public void NotifyVehicleWarped(Vector3 positionDelta)
    {
        if (_headingProxy == null || vehicleTarget == null) return;

        _headingProxy.position = vehicleTarget.position;
        _headingYaw            = vehicleTarget.eulerAngles.y;
        _headingProxy.rotation = Quaternion.Euler(0f, _headingYaw, 0f);

        _haveTravelYaw  = false;
        _airBoundActive = false;

        if (vcamDrive  != null) vcamDrive.OnTargetObjectWarped(_headingProxy, positionDelta);
        if (vcamStrafe != null) vcamStrafe.OnTargetObjectWarped(vehicleTarget, positionDelta);
    }

    /// <summary>
    /// Solves a definitive state WITHOUT committing it, and reports whether the
    /// craft actually fits in frame. Pure: safe to call from an inspector
    /// repaint, and it disturbs neither the live camera nor the preview.
    ///
    /// This is the same solver the camera runs, which is the only reason the
    /// answer means anything. Measuring a reimplementation would just tell you
    /// about the reimplementation.
    ///
    /// Geometry: the camera sits at the solved follow offset, aims at the
    /// solved look point, and the craft is at the origin of that space. Angles
    /// are reported from the look axis, and the margins subtract the craft's
    /// bounding sphere so "fits" means the whole hull.
    /// </summary>
    /// <param name="state">Which definitive state to measure.</param>
    /// <param name="aspect">Viewport width over height, for the horizontal check.</param>
    public FramingVerdict EvaluateState(CameraPreviewState state, float aspect)
    {
        FramingInputs   inp = BuildPreviewInputs(state);
        FramingSolution s   = inp.strafing ? SolveStrafeFraming(inp) : SolveDriveFraming(inp);

        FramingVerdict v = Measure(s.followOffset, s.targetOffset, s.fov, aspect);

        v.elevation    = inp.elevation;
        v.guardScale   = s.guardScale;
        v.guardRemoved = s.guardRemoved;

        return v;
    }

    /// <summary>
    /// Pure geometry: given where the camera sits, where it aims and how wide
    /// the lens is, does the craft fit, and by how much.
    ///
    /// Shared by EvaluateState and by the framing guard, deliberately. The
    /// guard has to search against exactly the test the readout reports, or the
    /// inspector would promise a margin the runtime does not deliver.
    ///
    /// Everything is in the follow target's local space, with the craft at the
    /// origin.
    /// </summary>
    private static FramingVerdict Measure(Vector3 followOffset, Vector3 targetOffset, float fov, float aspect)
    {
        var v = new FramingVerdict
        {
            followOffset = followOffset,
            targetOffset = targetOffset,
            fov          = fov,
            orbitRadius  = followOffset.magnitude,
            guardScale   = 1f
        };

        v.halfFovV = fov * 0.5f;
        v.halfFovH = Mathf.Atan(Mathf.Tan(v.halfFovV * Mathf.Deg2Rad) * Mathf.Max(0.01f, aspect)) * Mathf.Rad2Deg;

        Vector3 axis     = targetOffset - followOffset;
        Vector3 toCraft  = -followOffset;
        float   distance = toCraft.magnitude;

        // Camera sitting on the craft, or aiming at itself. Nothing meaningful
        // to report, and both would divide by zero.
        if (axis.sqrMagnitude < 1e-6f || distance < 1e-4f) return v;

        Vector3 fwd   = axis.normalized;
        Vector3 right = Vector3.Cross(Vector3.up, fwd);

        // Degenerate only if the camera aims straight up or down, which the
        // pitch ranges do not allow, but the fallback costs one branch.
        right = right.sqrMagnitude < 1e-6f ? Vector3.right : right.normalized;

        Vector3 up  = Vector3.Cross(fwd, right);
        Vector3 dir = toCraft / distance;

        // Centre angles, reported for diagnosis. Positive vertical reads as
        // "craft is BELOW the look axis", which is the direction this fails in.
        v.verticalOffAxis   = Mathf.Atan2(-Vector3.Dot(dir, up),    Vector3.Dot(dir, fwd)) * Mathf.Rad2Deg;
        v.horizontalOffAxis = Mathf.Atan2( Vector3.Dot(dir, right), Vector3.Dot(dir, fwd)) * Mathf.Rad2Deg;

        // Margins come from the worst CORNER, not the centre, because the
        // question is whether the hull fits rather than whether a point does.
        float worstV = 0f;
        float worstH = 0f;

        for (int i = 0; i < 8; i++)
        {
            var corner = CraftBoundsCentre + new Vector3(
                (i & 1) == 0 ? -CraftHalfExtents.x : CraftHalfExtents.x,
                (i & 2) == 0 ? -CraftHalfExtents.y : CraftHalfExtents.y,
                (i & 4) == 0 ? -CraftHalfExtents.z : CraftHalfExtents.z);

            Vector3 toCorner = corner - followOffset;

            if (toCorner.sqrMagnitude < 1e-6f) continue;

            toCorner.Normalize();
            float cf = Vector3.Dot(toCorner, fwd);

            // Behind the camera plane. atan2 would fold this back into a small
            // angle and report a corner that is nowhere on screen as in frame.
            if (cf <= 0f)
            {
                worstV = Mathf.PI;
                worstH = Mathf.PI;
                break;
            }

            worstV = Mathf.Max(worstV, Mathf.Abs(Mathf.Atan2(Vector3.Dot(toCorner, up),    cf)));
            worstH = Mathf.Max(worstH, Mathf.Abs(Mathf.Atan2(Vector3.Dot(toCorner, right), cf)));
        }

        v.verticalMargin   = v.halfFovV - worstV * Mathf.Rad2Deg;
        v.horizontalMargin = v.halfFovH - worstH * Mathf.Rad2Deg;

        return v;
    }

    /// <summary>Every state, in declaration order. Convenience for the inspector's sweep.</summary>
    public static CameraPreviewState[] AllPreviewStates =>
        (CameraPreviewState[])System.Enum.GetValues(typeof(CameraPreviewState));
}
