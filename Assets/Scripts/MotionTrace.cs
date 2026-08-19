using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// MotionTrace v1.4
///
/// v1.4: six columns for the aim-on-slopes question (TODO 0.29). The owner reported that
///       aiming feels good on flat ground and unusable on a slope, and NOTHING in this
///       trace could tell the two candidate causes apart, because the only attitude
///       column was `tilt_deg` and tilt cannot distinguish a nose-up craft from a
///       banked one, let alone say what the nose is up RELATIVE TO.
///
///       `nose_elev`, `nose_vs_ground` and `ground_elev` are the three angles the
///       question needs, and each is measured directly rather than derived from the
///       other two, for the reason v1.3 records: reconstructing a world-frame quantity
///       from body-frame parts is a different measurement, not a shortcut. They very
///       nearly sum, and where they do not is itself the signal.
///
///       All three are ELEVATIONS OF THE NOSE, not Euler pitch. Euler pitch has a sign
///       convention that inverts the reader's intuition (positive X is nose DOWN) and it
///       is contaminated by the decomposition once the craft is banked. The elevation of
///       the forward vector above a reference plane has neither problem, and it is the
///       quantity the player is actually complaining about: where the gun points.
///
///       `reticle_y` is READ FROM `VehicleHUD`, never recomputed here. The reticle's
///       screen position is already derived in exactly one place and the guns share that
///       ray by design; a second derivation in the instrument could disagree with the
///       game and would be believed. `reticle_valid` is false in drive mode, where the
///       reticle is hidden, so an empty column is obviously empty rather than silently
///       zero (the v1.3 lesson).
///
/// v1.3: `horiz_speed` added, and the camera/attitude columns moved into `FillShared`.
///
///       WORLD HORIZONTAL SPEED. Its absence cost two invalid analyses in one session.
///       `phys_vert` is the LOCAL vertical component, and during a flip the local frame
///       is rotating, so it stops meaning "vertical" exactly when a fall is being judged.
///       Both attempts to reconstruct horizontal speed as sqrt(speed^2 - physVert^2) were
///       therefore measuring something else, and one of them returned a confident null
///       result (10.2% against 9.9%) that looked like evidence. The gate that actually
///       broke the v2.7 camera bound is expressed in precisely this quantity, so it is
///       now recorded rather than reconstructed. **Reconstructing a world-frame quantity
///       from body-frame components is not a shortcut, it is a different measurement.**
///
///       `tilt_deg`, `travel_div`, `proxy_yaw`, `veh_yaw` and `yaw_diverge` moved from the
///       render path into `FillShared`, because MARKER and tick rows were being written
///       with all five at zero. Every marker in the session that motivated this read
///       `tilt=0.0 travelDiv=0.0` while airborne mid-flip, which is worse than missing
///       data: it is data that looks answered. A field that only some row kinds populate
///       has to be obviously empty, not silently zero.
///
/// v1.2: two columns added, one removed.
///
///       `tilt_deg` and `travel_div`. The owner observed by feel that a FLIP provokes the
///       stutter harder than a ROLL, and no column could confirm it: nothing recorded
///       attitude at all. `tilt_deg` is the separator. `travel_div` is the proxy's yaw
///       against the horizontal velocity heading, which is the exact quantity the v2.6
///       camera bound limits, so it is what says whether that bound is engaging rather
///       than merely compiled.
///
///       `cam_ang_rate` REMOVED rather than left reading zero. It used `Quaternion.Angle`,
///       which works from a dot product and returns exactly 0 for the sub-0.1-degree
///       rotations a 300fps frame contains (trap 15). It reported that the camera failed
///       to rotate on half of all frames, which looked like a major finding and was
///       entirely an artifact of the instrument. A column that reads zero for a broken
///       reason invites the same wrong conclusion twice, so it is gone; `cam_yaw_rate`,
///       computed with `Mathf.DeltaAngle` on angles, was correct all along and remains.
///
/// v1.1 closes the three blind spots the first real session exposed, and corrects two
/// claims v1.0 made that the data did not support.
///
///       CAMERA ROTATION. v1.0 logged where the camera WAS and never which way it was
///       POINTING. A yaw swing about the camera's own axis barely moves its position
///       while swinging the entire image, so the single most visible class of camera
///       artifact was close to invisible to the instrument. Three of the four markers in
///       the first session landed airborne with nothing on any logged channel, and two
///       of those sat at `AirControlWeight` 1.0, which is exactly the state where
///       `UpdateHeadingProxy` fades heading tracking to zero. The one state the markers
///       pointed at was the one state the trace could not see.
///
///       HEADING PROXY DIVERGENCE. The drive camera orbits on the proxy's yaw rather
///       than the vehicle's, and `trackAuthority` scales to zero at full air control, so
///       the two are DESIGNED to come apart mid-stunt and reconverge on release. That
///       makes the gap between them a tuning value, not a fault, and it was not recorded
///       anywhere. Logged as both yaw angles plus their signed wrapped difference, so
///       the reconverge can be read as a curve instead of guessed at.
///
///       VERTICAL VELOCITY. v1.0 logged forward and lateral and omitted the third axis,
///       which left the impacts impossible to decompose: a craft with 58 m/s forward and
///       6 m/s lateral reading 83 m/s total has 59 m/s going somewhere unrecorded.
///
///       COLLISION CONTACTS. The first session found 26 velocity discontinuities past
///       1000 m/s^2, confirmed against `rb.linearVelocity` rather than inferred, and had
///       no way at all to say WHAT was hit. Recorded now with the other collider's name,
///       the impulse and the contact normal.
///
/// TWO CORRECTIONS to v1.0, both found by its own data.
///
///       THE RESIDUAL IS INVALID ON A COLLISION TICK, and v1.0 presented it as though it
///       were not. It projects the drawn displacement onto the CURRENT velocity
///       direction; when a collision swings that direction 25 degrees in one tick the
///       projection collapses and reports a large draw error where the renderer did
///       nothing wrong. The worst-error list from the first session was mostly this
///       artifact. `residValid` now flags any frame whose travel direction moved more
///       than `residMaxTurnDeg`, so those rows can be excluded rather than misread.
///
///       A CLEAN RESIDUAL DOES NOT MEAN A SMOOTH IMAGE. The one marker that caught a
///       genuine delivery hitch, a 13.37ms frame at 90 m/s among 3ms neighbours, had a
///       residual of exactly 0.000m: the craft was drawn perfectly correctly and simply
///       held on screen four times too long. Draw error and frame time catch different
///       faults and neither substitutes for the other.
///
/// Answers the one question `FrameSpikeWatch` cannot: when the craft appears to lurch
/// forward or snap backward, WHICH of the three layers moved wrongly.
///
/// `FrameSpikeWatch` measures how long a frame took. That is a duration, and it fires
/// only past `minSpikeMs` 12 against a ~3ms baseline, so the entire 3-12ms band where
/// ordinary pacing jitter lives is invisible to it by construction. Its 2 second
/// heartbeat cannot resolve a per-tick wobble at all. Neither tool overlaps the other:
/// that one reports what the game was doing when a frame died, this one reports whether
/// what you SAW matched what the physics was actually doing.
///
/// THE SYMPTOM METRIC. The rigidbody is interpolated, so the transform that gets drawn
/// is a blend between two physics poses rather than either of them. If pacing is clean
/// that blend advances at exactly the body's velocity and the motion reads smooth. A
/// lurch is precisely a frame where it does not. So the measurement is the RESIDUAL:
/// take the frame-to-frame delta of the drawn transform, project it onto the direction
/// of travel, divide by that frame's delta time, and subtract the rigidbody's actual
/// speed. Zero means the drawing agreed with the physics. Positive means the craft was
/// drawn further along than it had really travelled, negative means it was drawn short,
/// and a negative frame followed by a positive one is the snap-back that reads as a
/// lurch. The sign IS the direction the owner reported, which is why the residual is
/// signed along travel rather than kept as a magnitude.
///
/// It is recorded twice, in m/s and in metres. The m/s figure is comparable across
/// speeds; the metre figure is how far out of place the craft was actually drawn that
/// frame, which is the one that says whether it was visible.
///
/// THREE SEPARABLE CHANNELS, because "jitter" could be any of them and the fix differs:
///   - the VEHICLE residual above, which is physics against render,
///   - the CAMERA's own per-frame speed and its frame-to-frame change, since the camera
///     has no rigidbody and so cannot be checked against a velocity, only for smoothness,
///   - the RELATIVE vector, vehicle position minus camera position, projected on camera
///     forward and right. This is the one the eye actually judges. A craft and a camera
///     that both move roughly smoothly can still produce visible chop in the gap between
///     them, and that gap is the image. Logging only the first two would miss it.
/// Only sampling all three in ONE pass can attribute the symptom, which is recipe 10 in
/// `CLAUDE.md`: the map cannot be held constant across two runs, so correlate rather
/// than A/B.
///
/// SAMPLED AT BEGIN-CONTEXT-RENDERING, not in `LateUpdate`. The camera pose has to be
/// read after `CinemachineBrain` has committed it, or the camera delta is one frame
/// stale and every relative figure derived from it is fabricated rather than merely
/// noisy. Hooking the render callback reads exactly the transforms that are about to be
/// drawn, and avoids `DefaultExecutionOrder`, which `CLAUDE.md` reserves as a last
/// resort.
///
/// ZERO ALLOCATION IN THE HOT PATH, and this is load-bearing rather than tidiness.
/// `FrameSpikeWatch` records that its own console logging became a plausible source of
/// the allocation it was reporting, and an instrument you cannot rule out is not
/// evidence. That tool gets away with building strings because it writes one row every
/// two seconds; a per-frame logger doing the same would allocate several hundred strings
/// a second and manufacture the GC pressure it exists to detect. So samples go into a
/// struct array allocated once in `Awake`, and formatting happens only at dump.
///
/// The buffer is BOUNDED rather than a ring, and stops when full. A ring would keep the
/// most recent window and silently discard the start, which is wrong here because the
/// analysis wants one contiguous timeline from a known beginning. Running out is
/// reported rather than hidden.
///
/// MARKER KEY (M by default; was F8 until it was found never to arrive on this project's
/// laptop — see the markerKey tooltip). Press it the moment a lurch is felt. Perception and
/// reaction put the press somewhere around 200-400ms AFTER the event, so a marker is a
/// pointer to a window that ends there, never a timestamp for the event itself. Without
/// it a clean-looking session is unfalsifiable: there is no way to tell a trace with no
/// artifacts from a trace where the artifacts happened and nothing flagged them.
///
/// Usage: drop on any scene object, play, drive, press the marker key on each felt lurch,
/// stop. CONFIRM THE FIRST PRESS LOGS TO THE CONSOLE before relying on a session.
/// Writes `PerfLogs/motiontrace_*.csv` alongside `FrameSpikeWatch`'s output.
/// </summary>
public class MotionTrace : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 🎯 Targets
    // -------------------------------------------------------------------------
    [Header("🎯 Targets")]
    [Tooltip("The craft to trace. Left empty, it finds the object carrying PlayerHoverInput, " +
             "which is how FrameSpikeWatch resolves the same thing.")]
    [SerializeField] private Transform vehicle;

    [Tooltip("The rendering camera. Left empty, it uses Camera.main. This must be the actual " +
             "Camera and not a virtual camera: the vcams are inputs to the brain, and what gets " +
             "drawn is whatever pose the brain settled on.")]
    [SerializeField] private Camera view;

    // -------------------------------------------------------------------------
    // 📐 Capture
    // -------------------------------------------------------------------------
    [Header("📐 Capture")]
    [Tooltip("Seconds of trace to reserve room for. Set it to the length of run you actually " +
             "intend to make, plus a little: capture STOPS when the buffer fills, and the rest " +
             "of the session is simply not recorded (it is reported, not silently wrapped).\n\n" +
             "Sizing: a row costs 132 bytes and the trace writes one per frame PLUS one per " +
             "physics tick, so at 100Hz fixed it is (fps + 100) rows per second.\n" +
             "  600s vsynced at 165Hz  ->  ~159k rows, ~23MB, covers the full 10 minutes\n" +
             "  240s at editor speeds  ->  ~96k rows,  ~13MB, covers about 4 minutes\n" +
             "The buffer is allocated once before capture starts, so its cost never lands " +
             "inside the measurement. Memory is the cheap resource here and a truncated " +
             "session is the expensive one, so round UP.")]
    [Min(5f)]
    [SerializeField] private float captureSeconds = 600f;

    [Tooltip("Upper bound on frame rate. Sizing only -- nothing reads it at runtime, so " +
             "overshooting costs 132 bytes per second per surplus fps and nothing else, while " +
             "undershooting silently ends the capture early. Round UP.\n\n" +
             "The ceiling is the DISPLAY, not the game: QualitySettings has vSyncCount 1 on the " +
             "only quality level and nothing calls Application.targetFrameRate, so a build " +
             "cannot exceed the refresh rate of the monitor it opens on. 180 covers a 165Hz " +
             "panel with margin. The editor is not vsynced the same way and runs far higher, " +
             "which is why this used to sit at 400.")]
    [Min(60)]
    [SerializeField] private int assumedMaxFps = 180;

    [Tooltip("Frames to ignore at startup, matching FrameSpikeWatch. Scene load and first-sight " +
             "shader compilation produce genuine hitches that say nothing about how the game runs " +
             "once it is going, and including them would also skew every distribution computed " +
             "from the trace.")]
    [Min(0)]
    [SerializeField] private int warmupFrames = 120;

    [Tooltip("Also record one row per FixedUpdate. This is what exonerates or convicts the " +
             "physics: if the per-tick motion is smooth while the render residual spikes, the " +
             "solver is fine and the problem is delivery or the camera.")]
    [SerializeField] private bool traceFixedSteps = true;

    [Tooltip("Key pressed on each felt lurch. Read through the Input System because the project " +
             "runs activeInputHandler 1, where the legacy Input class throws rather than returning " +
             "false.\n\n" +
             "Deliberately NOT an F-row key. It was F8 (the impulse router owns F9-F12), and on " +
             "this project's laptop F8 never arrived: eight sessions in a row recorded zero " +
             "markers while injecting the same key in code registered fine, so the presses were " +
             "being eaten before Unity saw them — an Fn media layer and a keyboard with known " +
             "liquid damage are both candidates. A marker key must be a plain key you can confirm.\n\n" +
             "Whatever you set it to, watch the console for the MARKER log on your first press. " +
             "The failure mode is silent: the trace looks perfectly healthy without them.")]
    [SerializeField] private UnityEngine.InputSystem.Key markerKey = UnityEngine.InputSystem.Key.M;

    [Tooltip("Write the CSV when play mode ends. On for the same reason FrameSpikeWatch has it " +
             "on: the rows live in memory and stopping play would otherwise throw them away.")]
    [SerializeField] private bool autoDumpOnStop = true;

    [Tooltip("How far the direction of travel may swing in one frame before the draw-error " +
             "figure is marked invalid.\n" +
             "The residual projects drawn motion onto the CURRENT velocity direction, so a " +
             "collision that swings that direction mid-frame collapses the projection and " +
             "manufactures a large error where the renderer did nothing wrong. That artifact " +
             "produced most of v1.0's worst-error list. 5 degrees per frame is far above " +
             "anything ordinary steering produces at 300fps and far below a collision.")]
    [Min(0.5f)]
    [SerializeField] private float residMaxTurnDeg = 5f;

    [Tooltip("Record collisions on the vehicle root. This is what turns 'the speed dropped 22 m/s " +
             "in one tick' into a named collider, which is the difference between knowing there " +
             "is a problem and knowing where it is.")]
    [SerializeField] private bool traceCollisions = true;

    // -------------------------------------------------------------------------
    // Sample
    // -------------------------------------------------------------------------
    // One struct for both row kinds rather than two arrays. The analysis wants frame
    // rows and tick rows interleaved on a single timeline, because the question at a
    // lurch is what the physics was doing on the ticks either side of it, and splitting
    // the files would mean rebuilding that ordering by hand.
    private enum Kind : byte { Frame = 0, Tick = 1, Marker = 2, Hit = 3 }

    private struct Sample
    {
        public float t;
        public Kind  kind;

        // Frame rows
        public float dtMs;
        public short steps;
        public float visAlong;      // drawn speed along travel, m/s
        public float physSpeed;     // rigidbody speed, m/s
        public float residMs;       // visAlong - physSpeed, signed
        public float residM;        // the same error expressed as metres misdrawn
        public bool  residValid;    // false when a direction swing invalidated the projection
        public float visSpeed;      // drawn speed, unprojected
        public float camSpeed;
        public float relAlong;      // vehicle-minus-camera drift along camera forward, m/s
        public float relLat;        // and across it

        // Camera aim. The image is mostly rotation, so this is the channel that was
        // missing rather than a refinement of the position one.
        // v1.2 removed `camAngRate`, which used `Quaternion.Angle` and returned exactly 0
        // for the sub-0.1-degree rotations a 300fps frame contains (trap 15). Leaving a
        // column that reads zero invites the same wrong conclusion a second time.
        public float camYawRate;    // deg/s about world up, via Mathf.DeltaAngle on euler
        public float proxyYaw;      // heading proxy yaw, which is what the drive cam orbits on
        public float vehYaw;
        public float yawDiverge;    // signed wrapped proxyYaw - vehYaw

        // Added v1.2. `tilt` separates a flip from a roll, which the owner identified by
        // feel before there was any column that could confirm it. `travelDiv` is the
        // quantity the v2.6 camera bound actually limits, so it is the one that says
        // whether that bound is engaging rather than merely compiled.
        public float tilt;          // degrees from upright
        public float travelDiv;     // signed proxyYaw - horizontal velocity heading
        public float horizSpeed;    // WORLD horizontal speed, v1.3

        // Added v1.4 for TODO 0.29. Three elevations, positive nose UP, each measured
        // directly. `noseElev` is where the gun points against the horizon and is the
        // one the acceptance test is written in. `noseVsGround` is the deviation the
        // hover sensors already remove. `groundElev` is the slope itself along the
        // direction the craft faces.
        public float noseElev;      // forward above WORLD horizontal, deg
        public float noseVsGround;  // forward above the GROUND plane, deg
        public float groundElev;    // the ground plane's own slope along forward, deg
        public float aimCmd;        // commanded aim pitch, deg, before the servo
        public float reticleY;      // canvas Y of the reticle, from VehicleHUD
        public bool  reticleValid;  // false in drive mode, where there is no reticle

        // Hit rows
        public float hitImpulse;
        public float hitNormalY;    // 1 is a floor, 0 is a wall; separates landings from catches
        public short hitIndex;      // into _hitNames

        // Every kind
        public float physFwd, physLat, physVert, yawRate;
        public float support;
        public bool  grounded;
        public float throttle, turn;
        public float boost, drift, strafe, air;
        public short gc0;
    }

    /// <summary>
    /// Forwards `OnCollisionEnter` from the vehicle root, added at runtime and destroyed
    /// with the trace. Unity delivers collision callbacks only to the GameObject owning
    /// the Rigidbody and never to children, which is the same constraint that forced
    /// `VehicleCollisionRelay` to exist. That component is deliberately left alone: it
    /// belongs to the shipping camera shake path and a diagnostic has no business
    /// widening it. Added by code rather than authored in the scene so a temporary
    /// measurement never becomes a saved scene change.
    /// </summary>
    public class CollisionProbe : MonoBehaviour
    {
        internal MotionTrace owner;
        private void OnCollisionEnter(Collision c) => owner?.ReportCollision(c);
    }

    private Sample[] _buf;
    private int      _count;
    private bool     _full;
    private bool     _capturing;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private Rigidbody                  _rb;
    private HoverController_Foundation _foundation;
    private HoverController_Propulsion _propulsion;
    private PlayerHoverInput           _input;
    private VehicleHUD                 _hud;

    private int     _frames;
    private int     _fixedSteps;
    private int     _lastGc0;

    private Vector3    _lastVehPos;
    private Vector3    _lastCamPos;
    private Vector3    _lastRel;
    private float      _lastCamYaw;
    private bool       _havePrev;

    private Transform      _proxy;
    private CollisionProbe _probe;

    private Vector3 _lastTravel;
    private bool    _lastTravelValid;

    // Collisions are rare, a few dozen in a session, so the name lookup is allowed to
    // allocate. Nothing on the per-frame path does.
    private readonly List<string> _hitNames = new();
    private int _hitCount;

    private Vector3 _lastTickPos;
    private bool    _haveTickPrev;

    private int _markerCount;

    private readonly StringBuilder _sb = new();

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        // Sized and allocated before anything is measured, deliberately. Growing a list
        // mid-capture would allocate on exactly the frames being judged.
        int capacity = Mathf.CeilToInt(captureSeconds * (assumedMaxFps + (traceFixedSteps ? 110 : 0)));
        _buf = new Sample[capacity];
    }

    private void OnEnable()  => RenderPipelineManager.beginContextRendering += OnBeginContextRendering;
    private void OnDisable()
    {
        RenderPipelineManager.beginContextRendering -= OnBeginContextRendering;

        // The probe was added by code, so it must be removed by code. Leaving a
        // diagnostic component welded to the vehicle after the trace is off is exactly
        // the kind of residue that turns into a mystery three sessions later.
        if (_probe != null)
        {
            if (Application.isPlaying) Destroy(_probe);
            else                       DestroyImmediate(_probe);
            _probe = null;
        }

        if (autoDumpOnStop) DumpCsv();
        LogSummary();
    }

    private void Start()
    {
        if (vehicle == null)
        {
            var player = FindFirstObjectByType<PlayerHoverInput>();
            if (player != null) vehicle = player.transform;
        }

        if (vehicle != null)
        {
            _rb          = vehicle.GetComponent<Rigidbody>();
            _foundation  = vehicle.GetComponent<HoverController_Foundation>();
            _propulsion  = vehicle.GetComponent<HoverController_Propulsion>();
            _input       = vehicle.GetComponent<PlayerHoverInput>();
        }

        if (view == null) view = Camera.main;

        // The HUD lives on the Canvas, not on the craft, so it cannot come off `vehicle`
        // like the four above. Read rather than recomputed: see the v1.4 note.
        _hud = FindFirstObjectByType<VehicleHUD>();

        // Found by name rather than by reflecting a private field: the controller creates
        // it as a real GameObject, and a name is stable across refactors in a way that
        // "_headingProxy" is not. Start runs after every Awake, so it exists by now.
        var proxyGo = GameObject.Find("CameraHeadingProxy");
        if (proxyGo != null) _proxy = proxyGo.transform;

        if (traceCollisions && vehicle != null)
        {
            _probe = vehicle.gameObject.AddComponent<CollisionProbe>();
            _probe.owner = this;
        }

        _lastGc0 = System.GC.CollectionCount(0);

        if (vehicle == null || view == null)
            Debug.LogWarning("[MotionTrace] Vehicle or camera unresolved; trace will be empty.", this);
        if (_proxy == null)
            Debug.LogWarning("[MotionTrace] CameraHeadingProxy not found; yaw divergence will read zero.", this);
    }

    /// <summary>
    /// One row per collision, carrying what was hit. `relativeVelocity` is recorded
    /// alongside the impulse because they answer different questions: the impulse is how
    /// hard the solver pushed back, the relative velocity is how fast the craft was
    /// closing, and a large closing speed with a small impulse is a graze rather than a
    /// crash. The contact normal's Y separates a landing from a wall, which is the
    /// distinction that decides whether 26 impacts in a session are ordinary driving.
    /// </summary>
    internal void ReportCollision(Collision c)
    {
        if (!_capturing || _full) return;

        _hitCount++;

        ref Sample s = ref Next(Kind.Hit);
        s.t          = Time.unscaledTime;
        s.hitImpulse = c.impulse.magnitude;
        s.physSpeed  = c.relativeVelocity.magnitude;
        s.hitNormalY = c.contactCount > 0 ? c.GetContact(0).normal.y : 0f;

        _hitNames.Add(c.collider != null ? c.collider.name : "null");
        s.hitIndex = (short)(_hitNames.Count - 1);

        FillShared(ref s);
    }

    private void FixedUpdate()
    {
        _fixedSteps++;

        if (!_capturing || !traceFixedSteps || _rb == null) return;

        // Per-tick displacement uses rb.position rather than the transform: the transform
        // is the INTERPOLATED pose and would fold render smoothing back into the physics
        // channel, which is the one measurement here that has to stay independent of it.
        Vector3 p = _rb.position;
        float   d = _haveTickPrev ? (p - _lastTickPos).magnitude : 0f;
        _lastTickPos  = p;
        _haveTickPrev = true;

        ref Sample s = ref Next(Kind.Tick);
        s.t        = Time.unscaledTime;
        s.visAlong = d / Time.fixedDeltaTime;   // per-tick speed, the physics-side smoothness check
        FillShared(ref s);
    }

    private void Update()
    {
        _frames++;
        if (!_capturing && _frames > warmupFrames) _capturing = true;

        if (!_capturing || _full) return;

        var kb = UnityEngine.InputSystem.Keyboard.current;
        if (kb != null && kb[markerKey].wasPressedThisFrame)
        {
            _markerCount++;
            ref Sample m = ref Next(Kind.Marker);
            m.t = Time.unscaledTime;
            FillShared(ref m);

            // Logged immediately, not just tallied in the end-of-session summary. A marker
            // that silently fails to register is worse than no marker at all: it costs a
            // whole playtest, because the trace looks healthy and the absence is only
            // discovered afterwards. Happened 2026-08-13 — eight consecutive sessions
            // recorded zero markers while the key path itself was verified working by
            // injection, so the presses were never arriving. Confirm the key lands ONCE at
            // the start of a session and the rest of the run is trustworthy.
            Debug.Log($"[MotionTrace] MARKER {_markerCount} at t={m.t:F2}s", this);
        }
    }

    /// <summary>
    /// The render-time sample. Runs after every `LateUpdate` and after the Cinemachine
    /// brain has written the camera, so the transforms read here are the ones about to
    /// be drawn. Everything in it is a read of resident state plus a handful of vector
    /// operations; nothing walks the scene and nothing allocates.
    /// </summary>
    private void OnBeginContextRendering(ScriptableRenderContext ctx, List<Camera> cameras)
    {
        if (!_capturing || _full || vehicle == null || view == null || _rb == null) return;

        // Only the main view. In the editor the scene view renders through the same
        // callback, and counting it would double-sample every frame and halve every
        // measured delta time.
        bool isView = false;
        for (int i = 0; i < cameras.Count; i++)
            if (cameras[i] == view) { isView = true; break; }
        if (!isView) return;

        float dt = Time.unscaledDeltaTime;
        if (dt <= 0f) return;

        Vector3 vehPos = vehicle.position;
        Vector3 camPos = view.transform.position;
        Vector3 camFwd = view.transform.forward;
        Vector3 camRight = view.transform.right;
        Vector3 rel    = vehPos - camPos;

        int steps   = _fixedSteps;
        _fixedSteps = 0;

        float      camYaw = view.transform.eulerAngles.y;

        if (!_havePrev)
        {
            _lastVehPos = vehPos;
            _lastCamPos = camPos;
            _lastRel    = rel;
            _lastCamYaw = camYaw;
            _havePrev   = true;
            return;
        }

        Vector3 dVeh = vehPos - _lastVehPos;
        Vector3 dCam = camPos - _lastCamPos;
        Vector3 dRel = rel    - _lastRel;

        Vector3 vel       = _rb.linearVelocity;
        float   physSpeed = vel.magnitude;

        // Project the drawn motion onto the direction the body is actually travelling.
        // Below a walking pace the direction is meaningless noise, so fall back to raw
        // magnitude rather than reporting a residual against a random axis.
        Vector3 travel   = physSpeed > 0.5f ? vel / physSpeed : dVeh.normalized;
        float   visAlong = Vector3.Dot(dVeh, travel) / dt;
        float   visSpeed = dVeh.magnitude / dt;

        float camSpeed = dCam.magnitude / dt;

        // A collision can swing the travel direction mid-frame, which collapses the
        // projection above and fabricates a draw error. Measure the swing and flag it
        // rather than silently reporting the artifact as a rendering fault.
        float turnDeg = _lastTravelValid ? Vector3.Angle(_lastTravel, travel) : 0f;
        _lastTravel      = travel;
        _lastTravelValid = true;

        ref Sample s = ref Next(Kind.Frame);
        s.t          = Time.unscaledTime;
        s.dtMs       = dt * 1000f;
        s.steps      = (short)steps;
        s.visAlong   = visAlong;
        s.physSpeed  = physSpeed;
        s.residMs    = visAlong - physSpeed;
        s.residM     = (visAlong - physSpeed) * dt;
        s.residValid = turnDeg <= residMaxTurnDeg;
        s.visSpeed   = visSpeed;
        s.camSpeed   = camSpeed;
        s.relAlong   = Vector3.Dot(dRel, camFwd) / dt;
        s.relLat     = Vector3.Dot(dRel, camRight) / dt;

        s.camYawRate = Mathf.DeltaAngle(_lastCamYaw, camYaw) / dt;

        FillShared(ref s);

        _lastVehPos = vehPos;
        _lastCamPos = camPos;
        _lastRel    = rel;
        _lastCamYaw = camYaw;
    }

    /// <summary>
    /// State carried by every row kind, so a marker and a tick can be read against the
    /// frames around them without joining across files.
    /// </summary>
    private void FillShared(ref Sample s)
    {
        if (_rb != null)
        {
            Vector3 local = vehicle.InverseTransformDirection(_rb.linearVelocity);
            s.physFwd  = local.z;
            s.physLat  = local.x;
            s.physVert = local.y;
            s.yawRate  = _rb.angularVelocity.y * Mathf.Rad2Deg;

            // WORLD horizontal speed, added v1.3. The local components above are useless
            // for judging a fall, because during a flip the local frame is rotating and
            // local "vertical" stops meaning vertical. Two analyses were run against
            // sqrt(speed^2 - physVert^2) as a stand-in for this and both were invalid;
            // the gate that actually broke the camera bound is expressed in exactly this
            // quantity, so it has to be recorded rather than reconstructed.
            Vector3 flat = _rb.linearVelocity;
            flat.y = 0f;
            s.horizSpeed = flat.magnitude;

            s.tilt = Vector3.Angle(vehicle.up, Vector3.up);

            if (_proxy != null)
            {
                s.proxyYaw   = _proxy.eulerAngles.y;
                s.vehYaw     = vehicle.eulerAngles.y;
                s.yawDiverge = Mathf.DeltaAngle(s.vehYaw, s.proxyYaw);
                s.travelDiv  = flat.sqrMagnitude > 1f
                    ? Mathf.DeltaAngle(Mathf.Atan2(flat.x, flat.z) * Mathf.Rad2Deg, s.proxyYaw)
                    : 0f;
            }
        }

        if (_foundation != null)
        {
            s.grounded = _foundation.IsHoverGrounded;
            s.support  = _foundation.HoverSupport;
        }

        // Attitude, v1.4. Outside the _rb block because none of it needs the rigidbody,
        // and outside the _foundation block because nose elevation against the WORLD is
        // still meaningful with no ground under the craft. AverageGroundNormal falls back
        // to world up airborne, which is Foundation's own convention, so `nose_vs_ground`
        // and `nose_elev` agreeing in mid-air is correct rather than a bug.
        if (vehicle != null)
        {
            Vector3 fwd = vehicle.forward;
            Vector3 n   = _foundation != null ? _foundation.AverageGroundNormal : Vector3.up;

            s.noseElev     = Mathf.Asin(Mathf.Clamp(fwd.y, -1f, 1f))              * Mathf.Rad2Deg;
            s.noseVsGround = Mathf.Asin(Mathf.Clamp(Vector3.Dot(fwd, n), -1f, 1f)) * Mathf.Rad2Deg;

            Vector3 alongSlope = Vector3.ProjectOnPlane(fwd, n);
            s.groundElev = alongSlope.sqrMagnitude > 1e-6f
                ? Mathf.Asin(Mathf.Clamp(alongSlope.normalized.y, -1f, 1f)) * Mathf.Rad2Deg
                : 0f;
        }

        if (_hud != null)
        {
            s.reticleValid = _hud.ReticleVisible;
            s.reticleY     = _hud.ReticleCanvasY;
        }

        if (_propulsion != null)
        {
            s.boost  = _propulsion.BoostLerp;
            s.drift  = _propulsion.DriftLerp;
            s.strafe = _propulsion.StrafeModeBlend;
            s.air    = _propulsion.AirControlWeight;
            s.aimCmd = _propulsion.CommandedAimPitch;
        }

        if (_input != null)
        {
            s.throttle = _input.ThrottleInput;
            s.turn     = _input.TurnInput;
        }

        int gc0 = System.GC.CollectionCount(0);
        s.gc0    = (short)(gc0 - _lastGc0);
        _lastGc0 = gc0;
    }

    private ref Sample Next(Kind kind)
    {
        if (_count >= _buf.Length)
        {
            _full = true;
            return ref _buf[_buf.Length - 1];
        }

        ref Sample s = ref _buf[_count++];
        s = default;
        s.kind = kind;
        return ref s;
    }

    // -------------------------------------------------------------------------
    // Reporting
    // -------------------------------------------------------------------------
    private void LogSummary()
    {
        if (_count == 0) return;

        // The distribution is computed here rather than left for the CSV, because the
        // single number that decides where to look next is the share of frames whose
        // drawn motion disagreed with the physics, and nobody should have to open a
        // spreadsheet to learn it.
        int   frames = 0;
        float worstBack = 0f, worstFwd = 0f;
        int   badFrames = 0;
        float dtSum = 0f, dtSumSq = 0f;
        float dtMax = 0f;

        for (int i = 0; i < _count; i++)
        {
            if (_buf[i].kind != Kind.Frame) continue;
            frames++;

            // Only frames whose travel direction held still. Including collision frames
            // is what made v1.0 report a 2m snap-back that the renderer never caused.
            if (_buf[i].residValid)
            {
                float r = _buf[i].residM;
                if (r < worstBack) worstBack = r;
                if (r > worstFwd)  worstFwd  = r;
                if (Mathf.Abs(r) > 0.05f) badFrames++;
            }

            float dt = _buf[i].dtMs;
            dtSum   += dt;
            dtSumSq += dt * dt;
            if (dt > dtMax) dtMax = dt;
        }

        if (frames == 0) return;

        float mean = dtSum / frames;
        float sd   = Mathf.Sqrt(Mathf.Max(0f, dtSumSq / frames - mean * mean));

        Debug.Log($"[MotionTrace] SESSION: {frames} frames, {_count - frames} other rows, "
                + $"{_markerCount} markers. Frame time {mean:F2}ms mean, {sd:F2}ms sd, {dtMax:F1}ms max."
                + (_full ? " BUFFER FILLED, capture ended early." : ""), this);

        Debug.Log($"[MotionTrace] DRAW ERROR (direction-stable frames only): worst snap-back "
                + $"{worstBack:F3}m, worst jump-ahead {worstFwd:F3}m, {badFrames} frames "
                + $"({badFrames * 100f / frames:F1}%) drawn more than 5cm out of place. Note a clean "
                + $"figure here does NOT mean a smooth image: a late frame drawn correctly still "
                + $"reads as a stutter, so judge that from the frame time above.", this);

        // Collisions get their own line because the last session found 26 severe velocity
        // discontinuities and had no way to say what caused them. The floor/wall split is
        // the whole point: landings are expected, wall catches while grounded are not.
        if (_hitCount > 0)
        {
            int walls = 0, floors = 0;
            float worstImpulse = 0f;
            string worstWhat = "";
            for (int i = 0; i < _count; i++)
            {
                if (_buf[i].kind != Kind.Hit) continue;
                if (_buf[i].hitNormalY > 0.7f) floors++; else walls++;
                if (_buf[i].hitImpulse > worstImpulse)
                {
                    worstImpulse = _buf[i].hitImpulse;
                    int hi = _buf[i].hitIndex;
                    worstWhat = hi >= 0 && hi < _hitNames.Count ? _hitNames[hi] : "?";
                }
            }

            Debug.Log($"[MotionTrace] COLLISIONS: {_hitCount} total, {floors} floor-ish "
                    + $"(normal.y > 0.7), {walls} wall-ish. Hardest {worstImpulse:F0} Ns against "
                    + $"'{worstWhat}'.", this);
        }
    }

    [ContextMenu("Dump Motion CSV")]
    private void DumpCsv()
    {
        if (_count == 0)
        {
            Debug.Log("[MotionTrace] Nothing captured.", this);
            return;
        }

        string dir = System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "..", "PerfLogs"));
        System.IO.Directory.CreateDirectory(dir);

        string path = System.IO.Path.Combine(dir, $"motiontrace_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");

        _sb.Clear();
        _sb.Append("time_s,kind,dt_ms,fixed_steps,vis_along,phys_speed,resid_ms,resid_m,resid_valid,")
           .Append("vis_speed,cam_speed,rel_along,rel_lat,")
           .Append("cam_yaw_rate,proxy_yaw,veh_yaw,yaw_diverge,tilt_deg,travel_div,horiz_speed,")
           .Append("nose_elev,nose_vs_ground,ground_elev,aim_cmd,reticle_y,reticle_valid,")
           .Append("hit_what,hit_impulse,hit_normal_y,")
           .Append("phys_fwd,phys_lat,phys_vert,yaw_rate,support,grounded,throttle,turn,")
           .Append("boost,drift,strafe,air,gc0\n");

        for (int i = 0; i < _count; i++)
        {
            ref Sample s = ref _buf[i];
            _sb.Append(s.t.ToString("F4")).Append(',')
               .Append(s.kind == Kind.Frame  ? "frame"
                     : s.kind == Kind.Tick   ? "tick"
                     : s.kind == Kind.Marker ? "MARKER" : "HIT").Append(',')
               .Append(s.dtMs.ToString("F3")).Append(',')
               .Append(s.steps).Append(',')
               .Append(s.visAlong.ToString("F3")).Append(',')
               .Append(s.physSpeed.ToString("F3")).Append(',')
               .Append(s.residMs.ToString("F3")).Append(',')
               .Append(s.residM.ToString("F4")).Append(',')
               .Append(s.kind == Kind.Frame && s.residValid ? "1" : "0").Append(',')
               .Append(s.visSpeed.ToString("F3")).Append(',')
               .Append(s.camSpeed.ToString("F3")).Append(',')
               .Append(s.relAlong.ToString("F3")).Append(',')
               .Append(s.relLat.ToString("F3")).Append(',')
               .Append(s.camYawRate.ToString("F2")).Append(',')
               .Append(s.proxyYaw.ToString("F2")).Append(',')
               .Append(s.vehYaw.ToString("F2")).Append(',')
               .Append(s.yawDiverge.ToString("F2")).Append(',')
               .Append(s.tilt.ToString("F1")).Append(',')
               .Append(s.travelDiv.ToString("F2")).Append(',')
               .Append(s.horizSpeed.ToString("F2")).Append(',')
               .Append(s.noseElev.ToString("F2")).Append(',')
               .Append(s.noseVsGround.ToString("F2")).Append(',')
               .Append(s.groundElev.ToString("F2")).Append(',')
               .Append(s.aimCmd.ToString("F2")).Append(',')
               .Append(s.reticleValid ? s.reticleY.ToString("F1") : "").Append(',')
               .Append(s.reticleValid ? "1" : "0").Append(',')
               .Append(s.kind == Kind.Hit && s.hitIndex >= 0 && s.hitIndex < _hitNames.Count
                         ? _hitNames[s.hitIndex] : "").Append(',')
               .Append(s.hitImpulse.ToString("F1")).Append(',')
               .Append(s.hitNormalY.ToString("F2")).Append(',')
               .Append(s.physFwd.ToString("F2")).Append(',')
               .Append(s.physLat.ToString("F2")).Append(',')
               .Append(s.physVert.ToString("F2")).Append(',')
               .Append(s.yawRate.ToString("F1")).Append(',')
               .Append(s.support.ToString("F2")).Append(',')
               .Append(s.grounded ? "True" : "False").Append(',')
               .Append(s.throttle.ToString("F2")).Append(',')
               .Append(s.turn.ToString("F2")).Append(',')
               .Append(s.boost.ToString("F2")).Append(',')
               .Append(s.drift.ToString("F2")).Append(',')
               .Append(s.strafe.ToString("F2")).Append(',')
               .Append(s.air.ToString("F2")).Append(',')
               .Append(s.gc0).Append('\n');
        }

        // Streamed in slices rather than File.WriteAllText(_sb.ToString()), and the
        // reason is length, not taste. ToString() materializes the ENTIRE csv as one
        // contiguous string: at 10 minutes and 265 rows/s that is ~165k rows, roughly
        // 31M chars, so a 62MB single object on the large-object heap on top of the
        // ~62MB the builder is already holding in chunks. It very probably succeeds.
        // "Very probably" is the wrong standard here -- this runs at the END of the
        // session, so the one thing it can cost is the entire session's data, which
        // is trap 30 exactly. Slicing removes the contiguous allocation for eight
        // lines and no measurement impact, since play has already stopped.
        const int chunkChars = 1 << 16;
        using (var writer = new System.IO.StreamWriter(path, false, System.Text.Encoding.UTF8, chunkChars))
        {
            char[] chunk = new char[chunkChars];
            for (int i = 0; i < _sb.Length; i += chunkChars)
            {
                int n = Mathf.Min(chunkChars, _sb.Length - i);
                _sb.CopyTo(i, chunk, 0, n);
                writer.Write(chunk, 0, n);
            }
        }

        Debug.Log($"[MotionTrace] Wrote {_count} rows to {path}", this);
    }
}
