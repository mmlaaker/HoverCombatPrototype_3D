using System.Collections;
using System.Collections.Generic;
using Unity.Profiling;
using UnityEngine;

/// <summary>
/// AllocationBisect v1.0
///
/// Finds what is allocating, by switching things off and watching what stops.
///
/// Built after three failed attempts to identify a 5.5 MB/s idle allocation by
/// reasoning about the code. Each theory was plausible, each was wrong, and each cost
/// a measurement cycle to disprove. This does not have theories. It disables one
/// MonoBehaviour at a time, measures the allocation rate, puts it back, and ranks
/// them by what actually changed.
///
/// Two measurements come before the per-component sweep and both are load-bearing:
///
///   BASELINE, everything running, so each component has something to be compared to.
///
///   FLOOR, every candidate script disabled at once. This is the one that decides
///   whether the sweep is worth reading at all. If allocation barely drops with all
///   game code switched off, then no amount of per-component blame will find it and
///   the source is the engine, the editor, or a subsystem rather than a script. That
///   is a genuinely useful answer, and it is the answer nobody gets by guessing,
///   because a guess always names a script.
///
/// Allocation is read from the profiler's "GC Allocated In Frame" counter rather than
/// from total-memory deltas. Total memory cannot see allocation that happens in the
/// same window as a collection, which is exactly the window you care about when
/// something is allocating fast enough to trigger collections.
///
/// The craft should be PARKED. The sweep disables movement scripts among others, so a
/// vehicle in motion will drift, land, or fall during the run and every phase after
/// that measures a different situation.
/// </summary>
public class AllocationBisect : MonoBehaviour
{
    [Header("🔬 Sweep")]
    [Tooltip("Seconds of measurement per component. Shorter is faster but noisier; the whole run " +
             "is roughly this times the number of scripts in the scene, plus two.")]
    [Min(0.5f)]
    [SerializeField] private float windowSeconds = 2.5f;

    [Tooltip("Optional root to restrict the sweep to, usually the vehicle. Leave empty to sweep " +
             "every MonoBehaviour in the scene.")]
    [SerializeField] private Transform restrictTo;

    [Tooltip("Components whose names contain any of these are never touched. The measuring " +
             "instruments are excluded by default for obvious reasons.")]
    [SerializeField] private string[] excludeNameContains = { "AllocationBisect", "FrameSpikeWatch" };

    [Tooltip("Only report components that account for at least this much, in MB/s. Filters the " +
             "long tail of near-zero results that would otherwise bury the answer.")]
    [SerializeField] private float reportThresholdMbs = 0.05f;

    private ProfilerRecorder _gcRecorder;
    private bool _running;

    private readonly List<MonoBehaviour> _candidates = new();
    private readonly List<string> _rows = new();

    private void OnEnable()
    {
        _gcRecorder = ProfilerRecorder.StartNew(ProfilerCategory.Memory, "GC Allocated In Frame");
    }

    private void OnDisable()
    {
        if (_gcRecorder.Valid) _gcRecorder.Dispose();
    }

    [ContextMenu("Run Allocation Sweep")]
    private void RunSweep()
    {
        if (!Application.isPlaying)
        {
            Debug.LogWarning("[AllocationBisect] Enter play mode first.", this);
            return;
        }
        if (_running)
        {
            Debug.LogWarning("[AllocationBisect] Already running.", this);
            return;
        }
        StartCoroutine(Sweep());
    }

    private IEnumerator Sweep()
    {
        _running = true;

        if (!_gcRecorder.Valid)
        {
            Debug.LogError("[AllocationBisect] GC Allocated In Frame counter unavailable. "
                         + "Nothing can be measured, so nothing is reported.", this);
            _running = false;
            yield break;
        }

        CollectCandidates();

        if (_candidates.Count == 0)
        {
            Debug.LogWarning("[AllocationBisect] No enabled MonoBehaviours found to test.", this);
            _running = false;
            yield break;
        }

        float estimate = (_candidates.Count + 2) * windowSeconds;
        Debug.Log($"[AllocationBisect] Sweeping {_candidates.Count} components. "
                + $"About {estimate:F0}s. Leave the craft parked.", this);

        _rows.Clear();
        _rows.Add("phase,component,alloc_mb_s,delta_vs_baseline_mb_s,share_pct");

        yield return new WaitForSecondsRealtime(0.5f);

        // --- Baseline -------------------------------------------------------
        float baseline = 0f;
        yield return Measure(v => baseline = v);
        Debug.Log($"[AllocationBisect] BASELINE: {baseline:F2} MB/s", this);
        _rows.Add($"baseline,ALL_ENABLED,{baseline:F3},0,0");

        // --- Floor: every candidate off at once ------------------------------
        SetAll(false);
        float floor = 0f;
        yield return Measure(v => floor = v);
        SetAll(true);

        float scriptShare = baseline > 0f ? (baseline - floor) / baseline * 100f : 0f;
        _rows.Add($"floor,ALL_DISABLED,{floor:F3},{baseline - floor:F3},{scriptShare:F1}");

        Debug.Log($"[AllocationBisect] FLOOR (all scripts off): {floor:F2} MB/s. "
                + $"Scripts account for {scriptShare:F0}% of baseline allocation.", this);

        if (scriptShare < 20f)
        {
            Debug.LogWarning("[AllocationBisect] Most allocation SURVIVES with every game script "
                           + "disabled. The source is not your scripts: look at the engine, the "
                           + "editor, or a subsystem. The per-component sweep below will find "
                           + "nothing meaningful, and that is the finding.", this);
        }

        yield return new WaitForSecondsRealtime(0.25f);

        // --- Per component ---------------------------------------------------
        var results = new List<(string name, float rate, float delta)>();

        foreach (var mb in _candidates)
        {
            if (mb == null) continue;

            bool toggled = false;
            try { mb.enabled = false; toggled = true; }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[AllocationBisect] Could not disable {mb.GetType().Name}: {e.Message}", this);
            }

            if (!toggled) continue;

            float rate = 0f;
            yield return Measure(v => rate = v);

            try { mb.enabled = true; } catch { }

            results.Add((mb.GetType().Name, rate, baseline - rate));
        }

        results.Sort((a, b) => b.delta.CompareTo(a.delta));

        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"[AllocationBisect] RESULTS (baseline {baseline:F2} MB/s, floor {floor:F2} MB/s)");

        int shown = 0;
        foreach (var r in results)
        {
            float share = baseline > 0f ? r.delta / baseline * 100f : 0f;
            _rows.Add($"component,{r.name},{r.rate:F3},{r.delta:F3},{share:F1}");

            if (r.delta < reportThresholdMbs) continue;
            sb.AppendLine($"   {r.name,-38} -{r.delta:F2} MB/s  ({share:F0}%)");
            shown++;
        }

        if (shown == 0)
            sb.AppendLine("   Nothing above the reporting threshold. No single script owns this.");

        Debug.Log(sb.ToString(), this);
        WriteCsv();

        _running = false;
    }

    /// <summary>
    /// Sums the per-frame GC allocation counter across the window and divides by real
    /// elapsed time. Per-frame counter rather than a memory delta, so a collection
    /// landing inside the window cannot swallow the very allocation being measured.
    /// </summary>
    private IEnumerator Measure(System.Action<float> result)
    {
        // Let the change settle before counting; a disabled component can still have
        // one more frame of work in flight.
        yield return new WaitForSecondsRealtime(0.25f);

        long  bytes = 0;
        float start = Time.realtimeSinceStartup;

        while (Time.realtimeSinceStartup - start < windowSeconds)
        {
            long v = _gcRecorder.LastValue;
            if (v > 0) bytes += v;
            yield return null;
        }

        float elapsed = Mathf.Max(0.001f, Time.realtimeSinceStartup - start);
        result(bytes / 1048576f / elapsed);
    }

    private void CollectCandidates()
    {
        _candidates.Clear();

        MonoBehaviour[] all = restrictTo != null
            ? restrictTo.GetComponentsInChildren<MonoBehaviour>(true)
            : FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None);

        foreach (var mb in all)
        {
            if (mb == null || mb == this) continue;
            if (!mb.enabled) continue;          // already off; nothing to learn by disabling it

            string n = mb.GetType().Name;
            bool excluded = false;
            foreach (var ex in excludeNameContains)
                if (!string.IsNullOrEmpty(ex) && n.Contains(ex)) { excluded = true; break; }

            if (!excluded) _candidates.Add(mb);
        }
    }

    private void SetAll(bool enabled)
    {
        foreach (var mb in _candidates)
        {
            if (mb == null) continue;
            try { mb.enabled = enabled; } catch { }
        }
    }

    private void WriteCsv()
    {
        string dir = System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "..", "PerfLogs"));
        System.IO.Directory.CreateDirectory(dir);
        string path = System.IO.Path.Combine(dir, $"allocbisect_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
        System.IO.File.WriteAllLines(path, _rows);
        Debug.Log($"[AllocationBisect] Wrote {_rows.Count - 1} rows to {path}", this);
    }
}
