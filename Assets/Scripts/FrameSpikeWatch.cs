using System.Collections.Generic;
using System.Text;
using UnityEngine;

/// <summary>
/// FrameSpikeWatch v1.1
///
/// v1.1: Three additions, each closing a hole the first real sessions exposed.
///
///       HEARTBEAT rows, written on a timer whether or not anything went wrong. v1.0
///       recorded only spikes, which meant a session containing a two minute CLEAN
///       stretch produced almost no rows for exactly the part worth understanding, and
///       no trend could be read through it. A tool that only records failures cannot
///       tell you what normal looked like.
///
///       A CPU BENCHMARK: fixed arithmetic, timed once a second. "The game got heavier"
///       and "the machine got weaker" are indistinguishable from inside a frame timer,
///       and only a constant workload can separate them. Reported as a verdict at the
///       end rather than left as a column to squint at.
///
///       A measured ALLOCATION RATE instead of inferring it from the sawtooth in total
///       memory, which only ever gave the average between collections and hid whether
///       allocation was steady or bursty.
///

/// Catches frame time spikes and records what the game was doing at that instant.
/// Built for the case a profiler is bad at: an intermittent hitch nobody can
/// reproduce on demand, where by the time you have opened the Profiler window the
/// moment has passed and you are watching an idle craft behave perfectly.
///
/// The design rule is that DETECTION must be nearly free and CAPTURE can be
/// expensive, because capture only runs on the frames that were already ruined.
/// Per frame this costs one subtract, one multiply and two compares. Everything
/// that walks the scene happens inside the spike branch or on a slow timer.
///
/// What it deliberately does NOT do is decide anything. It reports the state of
/// several suspects at once (particles, GC, physics catch-up, vehicle state) so a
/// pattern can be read across many spikes. One spike tells you nothing; twenty
/// spikes that all landed on a GC tell you exactly where to look.
///
/// Detection is RELATIVE, not a fixed millisecond threshold. Editor play mode runs
/// around 3ms on this project while a build runs somewhere else entirely, and a
/// fixed threshold would either cry wolf in one and stay silent in the other. A
/// spike is a frame that is both meaningfully slower than this session's own
/// baseline AND slow enough in absolute terms to be felt.
///
/// Baseline updates only on NON-spike frames. Feeding spikes back into the average
/// would let a bad patch raise the bar until it stopped reporting, which is the
/// failure mode where a tool goes quiet exactly when things get worst.
///
/// Usage: drop it on any GameObject in the scene, play, and read the console. It
/// prints a session summary on stop, and "Dump Spike CSV" in the component context
/// menu writes every captured spike to a file for reading side by side.
/// </summary>
public class FrameSpikeWatch : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 🔍 Detection
    // -------------------------------------------------------------------------
    [Header("🔍 Detection")]
    [Tooltip("A frame counts as a spike when it is this many times the running baseline. " +
             "2.5 catches a stutter you would feel without reporting ordinary jitter.")]
    [Min(1.1f)]
    [SerializeField] private float spikeMultiplier = 2.5f;

    [Tooltip("Absolute floor in milliseconds. Both this and the multiplier must be exceeded.\n" +
             "Load-bearing: at a 3ms baseline the multiplier alone fires at 7.5ms, which nobody " +
             "can feel and which would bury the real hitches in noise. 12ms is roughly where a " +
             "60Hz frame starts slipping.")]
    [Min(1f)]
    [SerializeField] private float minSpikeMs = 12f;

    [Tooltip("Frames to ignore at startup. Scene load, shader compilation and the first sight of " +
             "any new material all produce genuine multi-frame hitches that say nothing about how " +
             "the game runs once it is going.")]
    [Min(0)]
    [SerializeField] private int warmupFrames = 120;

    [Tooltip("Minimum seconds between console lines. A sustained bad patch would otherwise print " +
             "every frame and the logging itself becomes the performance problem. Spikes are still " +
             "COUNTED during the cooldown; only the printing is throttled.")]
    [Min(0f)]
    [SerializeField] private float logCooldownSeconds = 0.25f;

    [Header("📋 Output")]
    [Tooltip("Print each spike to the console as it happens. OFF by default, and that default is a " +
             "measurement decision rather than a preference.\n" +
             "Editor Debug.Log captures a full stack trace and allocates on every call. In the first " +
             "session this ran, managed memory sat flat at 766MB for sixty seconds and then climbed " +
             "3MB per second from the exact moment spikes began -- which is the moment this started " +
             "logging. The instrument was a plausible source of the allocation it was reporting, and " +
             "an instrument you cannot rule out is not evidence. The CSV is the real artifact; turn " +
             "this on only to watch live, and do not trust memory figures from a session where it was.")]
    [SerializeField] private bool logToConsole = false;

    [Tooltip("Write the CSV automatically when play mode ends. On by default because the first " +
             "session's data was lost exactly this way: the rows live in memory and stopping play " +
             "threw them away before anyone thought to press the dump button.")]
    [SerializeField] private bool autoDumpOnStop = true;

    [Tooltip("How often the cached list of ParticleSystems is rebuilt, in seconds. Scanning the " +
             "scene is far too expensive to do per frame and pointless to do inside a spike, where " +
             "it would add to the very cost being measured.")]
    [Min(0.5f)]
    [SerializeField] private float particleRescanSeconds = 5f;

    [Tooltip("Seconds between HEARTBEAT rows, which are written whether or not anything went wrong.\n" +
             "Recording only spikes turned out to be a blind spot: a session with a two minute clean " +
             "stretch produced almost no rows for it, so there was no way to tell whether the CPU " +
             "benchmark or the allocation rate were drifting during exactly the part that mattered. " +
             "A trend needs samples from the good stretches too.")]
    [Min(0.25f)]
    [SerializeField] private float heartbeatSeconds = 2f;

    // -------------------------------------------------------------------------
    // 🌡 CPU Benchmark
    // -------------------------------------------------------------------------
    [Header("🌡 CPU Benchmark")]
    [Tooltip("Times a FIXED lump of arithmetic once a second. The work never changes, so if the time " +
             "it takes climbs over a session, the CPU itself has got slower and the cause is thermal " +
             "or power throttling rather than anything in the game.\n" +
             "This is the one measurement that can separate 'the game got heavier' from 'the machine " +
             "got weaker', and those two look identical from inside a frame timer.")]
    [SerializeField] private bool runBenchmark = true;

    [Tooltip("Iterations per benchmark run. Sized so a run costs a fraction of a millisecond: big " +
             "enough to be measurable, small enough that the probe never becomes the spike.")]
    [Min(1000)]
    [SerializeField] private int benchmarkIterations = 100000;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private float _baselineMs = -1f;
    private int   _frames;
    private float _lastLogTime = -999f;
    private float _nextParticleScan;

    private int   _spikeCount;
    private float _worstMs;
    private int   _spikesWithGc;

    private int _lastGc0;

    /// <summary>
    /// When the craft was last downed, and how many times it has been. A flip is the
    /// leading suspect for what starts a bad patch, and "seconds since the last
    /// recovery" is the column that turns that hunch into something the data can
    /// answer: if every spike carries a small value the flip is the trigger, and if
    /// the numbers wander then the chop outlived its supposed cause and the flip was
    /// just where the player happened to be.
    ///
    /// Sampled every frame rather than only on spikes, because a recovery that
    /// happens BETWEEN two spikes still has to be seen for the gap to mean anything.
    /// </summary>
    private float _lastDownedTime = -1f;
    private float _firstDownedTime = -1f;
    private int   _downedEvents;
    private bool  _wasDowned;

    /// <summary>
    /// FixedUpdate calls since the last Update. A long frame forces the physics loop
    /// to run several catch-up steps on the NEXT frame, which can make that one long
    /// too. That feedback is how one hitch becomes a stutter, and seeing the step
    /// count separates "something was slow once" from a loop that is eating itself.
    /// </summary>
    private int _fixedSteps;

    // Benchmark state. The stopwatch and the sink are both cached deliberately: this
    // component is used to hunt allocation, so it must not allocate on a timer itself.
    // _benchSink exists only so the loop cannot be optimised away as dead code.
    private readonly System.Diagnostics.Stopwatch _benchSw = new();
    private double _benchSink;
    private float  _benchMs;
    private float  _benchFirst = -1f;
    private float  _benchWorst;
    private float  _nextBenchmark;

    private float _nextHeartbeat;

    /// <summary>
    /// Allocation rate in MB/s over a rolling one second window. Measured directly
    /// rather than inferred from the sawtooth in total memory, because that only tells
    /// you the average between collections and hides whether allocation is steady or
    /// bursty. Negative deltas are dropped: those are collections, not allocations.
    /// </summary>
    private long  _lastMonoBytes;
    private long  _allocWindowBytes;
    private float _allocWindowStart;
    private float _allocRateMbs;

    private Rigidbody                  _rb;
    private HoverController_Foundation _foundation;
    private HoverController_Propulsion _propulsion;

    private ParticleSystem[] _particles = new ParticleSystem[0];

    private readonly List<string> _rows = new();
    private readonly StringBuilder _sb  = new();

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------
    private void Start()
    {
        var player = FindFirstObjectByType<PlayerHoverInput>();
        if (player != null)
        {
            _rb         = player.GetComponent<Rigidbody>();
            _foundation = player.GetComponent<HoverController_Foundation>();
            _propulsion = player.GetComponent<HoverController_Propulsion>();
        }

        _lastGc0 = System.GC.CollectionCount(0);
        RescanParticles();

        _lastMonoBytes    = UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong();
        _allocWindowStart = Time.unscaledTime;

        _rows.Add("time_s,kind,frame_ms,baseline_ms,fixed_steps,bench_ms,alloc_mb_s,"
                + "speed,fwd,lat,grounded,support,downed,"
                + "tilt_deg,since_downed_s,recoveries,"
                + "boost,drift,strafe,air,ps_playing,ps_particles,gc0_delta,mem_mb");
    }

    private void FixedUpdate() => _fixedSteps++;

    private void Update()
    {
        float ms = Time.unscaledDeltaTime * 1000f;
        _frames++;

        int steps   = _fixedSteps;
        _fixedSteps = 0;

        if (_baselineMs < 0f)
            _baselineMs = ms;

        if (Time.unscaledTime >= _nextParticleScan)
            RescanParticles();

        TickAllocationRate();

        // Benchmark runs BEFORE the spike test so a heartbeat or spike on this frame
        // reports a fresh number rather than one up to a second stale.
        if (runBenchmark && Time.unscaledTime >= _nextBenchmark)
        {
            RunBenchmark();
            _nextBenchmark = Time.unscaledTime + 1f;
        }

        // Downed tracking runs every frame, spike or not. A recovery that lands
        // between two spikes still has to be recorded for the gap to be meaningful.
        // Two bool reads; cheap enough for the hot path.
        if (_foundation != null)
        {
            bool downedNow = _foundation.IsDowned;
            if (downedNow)
            {
                _lastDownedTime = Time.unscaledTime;
                if (!_wasDowned)
                {
                    _downedEvents++;
                    if (_firstDownedTime < 0f) _firstDownedTime = _lastDownedTime;
                }
            }
            _wasDowned = downedNow;
        }

        bool spike = _frames > warmupFrames
                  && ms > minSpikeMs
                  && ms > _baselineMs * spikeMultiplier;

        if (!spike)
        {
            // Exponential average, non-spike frames only. See the class docstring for
            // why spikes must never feed this.
            _baselineMs += (ms - _baselineMs) * 0.05f;

            // Heartbeat: a row from an ordinary frame, so trends stay visible through
            // the stretches where nothing goes wrong. Never printed to console.
            if (_frames > warmupFrames && Time.unscaledTime >= _nextHeartbeat)
            {
                _nextHeartbeat = Time.unscaledTime + heartbeatSeconds;
                Capture(ms, steps, "beat", false);
            }
            return;
        }

        Capture(ms, steps, "spike", true);
    }

    /// <summary>
    /// Rolling one second allocation rate. Uses the profiler's mono counter rather than
    /// GC.GetTotalMemory, which is heavier and would be paid every frame. Returns 0 in
    /// non-development builds, where the counter is not maintained.
    /// </summary>
    private void TickAllocationRate()
    {
        long mono  = UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong();
        long delta = mono - _lastMonoBytes;
        _lastMonoBytes = mono;

        // Negative means a collection happened, which is not an allocation.
        if (delta > 0) _allocWindowBytes += delta;

        float elapsed = Time.unscaledTime - _allocWindowStart;
        if (elapsed < 1f) return;

        _allocRateMbs     = _allocWindowBytes / 1048576f / elapsed;
        _allocWindowBytes = 0;
        _allocWindowStart = Time.unscaledTime;
    }

    /// <summary>
    /// Fixed workload, timed. Identical every call by construction, so any change in
    /// how long it takes is a change in the machine rather than in the game.
    /// </summary>
    private void RunBenchmark()
    {
        _benchSw.Restart();

        double acc = 0;
        for (int i = 1; i <= benchmarkIterations; i++)
            acc += System.Math.Sqrt(i);

        _benchSw.Stop();
        _benchSink = acc;                       // keeps the loop from being elided
        _benchMs   = (float)_benchSw.Elapsed.TotalMilliseconds;

        if (_benchFirst < 0f) _benchFirst = _benchMs;
        if (_benchMs > _benchWorst) _benchWorst = _benchMs;
    }

    private void OnDisable()
    {
        // Dump BEFORE the summary. The rows are the artifact and they only exist in
        // memory; the first session was lost precisely because stopping play threw
        // them away before anyone pressed the button.
        if (autoDumpOnStop) DumpCsv();
        LogSummary();
    }

    // -------------------------------------------------------------------------
    // Capture
    // -------------------------------------------------------------------------
    /// <summary>
    /// Runs only on ruined frames, so it is allowed to be costly. Everything here is
    /// a read of already-resident state apart from the particle sum, which walks a
    /// cached array rather than the scene.
    /// </summary>
    private void Capture(float ms, int steps, string kind, bool isSpike)
    {
        if (isSpike)
        {
            _spikeCount++;
            if (ms > _worstMs) _worstMs = ms;
        }

        int gc0      = System.GC.CollectionCount(0);
        int gcDelta  = gc0 - _lastGc0;
        _lastGc0     = gc0;
        if (gcDelta > 0 && isSpike) _spikesWithGc++;

        int psPlaying   = 0;
        int psParticles = 0;
        for (int i = 0; i < _particles.Length; i++)
        {
            var ps = _particles[i];
            if (ps == null) continue;
            if (ps.isPlaying) psPlaying++;
            psParticles += ps.particleCount;
        }

        float speed = 0f, fwd = 0f, lat = 0f, support = 0f, tilt = 0f;
        bool grounded = false, downed = false;
        float boost = 0f, drift = 0f, strafe = 0f, air = 0f;

        if (_rb != null)
        {
            speed = _rb.linearVelocity.magnitude;
            Vector3 local = _rb.transform.InverseTransformDirection(_rb.linearVelocity);
            fwd = local.z;
            lat = local.x;
            tilt = Vector3.Angle(_rb.transform.up, Vector3.up);
        }

        if (_foundation != null)
        {
            grounded = _foundation.IsHoverGrounded;
            support  = _foundation.HoverSupport;
            downed   = _foundation.IsDowned;
        }

        if (_propulsion != null)
        {
            boost  = _propulsion.BoostLerp;
            drift  = _propulsion.DriftLerp;
            strafe = _propulsion.StrafeModeBlend;
            air    = _propulsion.AirControlWeight;
        }

        float memMb = System.GC.GetTotalMemory(false) / (1024f * 1024f);
        float t     = Time.unscaledTime;

        // -1 means the craft has never been downed this session, which must stay
        // distinguishable from "downed a very long time ago".
        float sinceDowned = _lastDownedTime < 0f ? -1f : t - _lastDownedTime;

        _rows.Add($"{t:F2},{kind},{ms:F2},{_baselineMs:F2},{steps},{_benchMs:F3},{_allocRateMbs:F2},"
                + $"{speed:F1},{fwd:F1},{lat:F1},"
                + $"{grounded},{support:F2},{downed},{tilt:F0},{sinceDowned:F1},{_downedEvents},"
                + $"{boost:F2},{drift:F2},{strafe:F2},{air:F2},"
                + $"{psPlaying},{psParticles},{gcDelta},{memMb:F1}");

        // Heartbeats are data only. Printing them would put the console cost back into
        // the hot path, which is the exact contamination that turned logging off.
        if (!isSpike || !logToConsole || t - _lastLogTime < logCooldownSeconds)
            return;

        _lastLogTime = t;

        _sb.Clear();
        _sb.Append("[FrameSpikeWatch] ").Append(ms.ToString("F1")).Append("ms")
           .Append("  (baseline ").Append(_baselineMs.ToString("F1")).Append("ms)")
           .Append("  t=").Append(t.ToString("F1")).Append('s')
           .Append("  physSteps=").Append(steps)
           .Append("  spd=").Append(speed.ToString("F0"))
           .Append("  grounded=").Append(grounded)
           .Append("  tilt=").Append(tilt.ToString("F0"))
           .Append("  sinceDowned=").Append(sinceDowned < 0f ? "never" : sinceDowned.ToString("F1") + "s")
           .Append("  blends b/d/s/a=").Append(boost.ToString("F1")).Append('/')
                                      .Append(drift.ToString("F1")).Append('/')
                                      .Append(strafe.ToString("F1")).Append('/')
                                      .Append(air.ToString("F1"))
           .Append("  particles=").Append(psParticles).Append(" in ").Append(psPlaying).Append(" systems")
           .Append("  GC0+").Append(gcDelta)
           .Append("  mem=").Append(memMb.ToString("F0")).Append("MB");

        Debug.Log(_sb.ToString(), this);
    }

    private void RescanParticles()
    {
        _particles        = FindObjectsByType<ParticleSystem>(FindObjectsSortMode.None);
        _nextParticleScan = Time.unscaledTime + particleRescanSeconds;
    }

    // -------------------------------------------------------------------------
    // Reporting
    // -------------------------------------------------------------------------
    /// <summary>
    /// The line that actually matters. A single spike is anecdote; the GC correlation
    /// across all of them is the finding. If most spikes carry a collection, the
    /// problem is allocation churn and no amount of tuning particles will help.
    /// </summary>
    private void LogSummary()
    {
        if (_frames <= warmupFrames)
            return;

        float gcShare = _spikeCount > 0 ? _spikesWithGc * 100f / _spikeCount : 0f;

        Debug.Log($"[FrameSpikeWatch] SESSION: {_frames} frames, baseline {_baselineMs:F2}ms, "
                + $"{_spikeCount} spikes, worst {_worstMs:F1}ms, "
                + $"{_spikesWithGc} of them ({gcShare:F0}%) landed on a GC collection. "
                + $"Flip recoveries: {_downedEvents}"
                + (_lastDownedTime < 0f ? " (never downed)." : $", last at t={_lastDownedTime:F1}s."), this);

        // The throttling verdict, stated rather than left to be eyeballed. The benchmark
        // is fixed work, so drift in how long it takes is drift in the machine. Anything
        // past ~15% is well outside run to run noise and means the CPU genuinely slowed.
        if (runBenchmark && _benchFirst > 0f)
        {
            float drift = (_benchMs - _benchFirst) / _benchFirst * 100f;
            float worst = (_benchWorst - _benchFirst) / _benchFirst * 100f;

            string verdict = worst > 15f
                ? "THROTTLING LIKELY: the CPU got measurably slower during this session."
                : "No throttling signal: the CPU held its speed, so the stalls are not thermal.";

            Debug.Log($"[FrameSpikeWatch] CPU BENCHMARK: first {_benchFirst:F3}ms, "
                    + $"last {_benchMs:F3}ms ({drift:+0.0;-0.0}%), worst {_benchWorst:F3}ms "
                    + $"({worst:+0.0;-0.0}%). {verdict}", this);
        }

        // The split that answers the question the session was run to answer. If spikes
        // are heavily bunched after the first recovery, the flip is a trigger. If they
        // are spread evenly either side of it, the flip was a coincidence and the chop
        // has some other cause that merely started around the same time.
        if (_downedEvents > 0 && _spikeCount > 0)
        {
            int before = 0, after = 0;
            for (int i = 1; i < _rows.Count; i++)
            {
                // Heartbeat rows must not be counted here: they fire on a timer, so
                // including them would just measure how long each half of the session
                // was and report it as a spike distribution.
                int c = _rows[i].IndexOf(',');
                if (c <= 0) continue;
                int c2 = _rows[i].IndexOf(',', c + 1);
                if (c2 <= c) continue;
                if (_rows[i].Substring(c + 1, c2 - c - 1) != "spike") continue;

                if (float.TryParse(_rows[i].Substring(0, c), out float rowTime))
                {
                    if (rowTime < _firstDownedTime) before++;
                    else after++;
                }
            }

            Debug.Log($"[FrameSpikeWatch] FLIP SPLIT: first recovery at t={_firstDownedTime:F1}s. "
                    + $"{before} spikes before it, {after} after.", this);
        }
    }

    [ContextMenu("Dump Spike CSV")]
    private void DumpCsv()
    {
        if (_rows.Count <= 1)
        {
            Debug.Log("[FrameSpikeWatch] No spikes recorded.", this);
            return;
        }

        // Project root rather than persistentDataPath: findable without digging through
        // AppData, and OUTSIDE Assets so Unity does not import it as a TextAsset and
        // trigger a reimport every time a session ends. Worth adding PerfLogs/ to
        // .gitignore so these never land in a commit.
        string dir = System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "..", "PerfLogs"));
        System.IO.Directory.CreateDirectory(dir);

        string path = System.IO.Path.Combine(dir, $"framespikes_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
        System.IO.File.WriteAllLines(path, _rows);
        Debug.Log($"[FrameSpikeWatch] Wrote {_rows.Count - 1} spikes to {path}", this);
    }
}
