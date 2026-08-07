using System;
using UnityEngine;

/// <summary>
/// Spin-up behaviour for sustained-fire weapons. Automatic type only.
/// Pure designer-facing values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class WeaponWindUpTuning
{
    // -------------------------------------------------------------------------
    // 🌀 Wind-up
    // -------------------------------------------------------------------------
    [Header("🌀 Wind-up")]
    [Tooltip("Whether the gun has to spin up before it reaches full speed. Off for a plain machine " +
             "gun that fires at full rate instantly, on for the Minigun.")]
    public bool useWindUp = false;

    [Tooltip("Seconds to go from cold to fully spun up.")]
    [Min(0.01f)]
    public float windUpDuration = 1.5f;

    [Tooltip("Shape of the spin-up. Left edge is cold, right edge is fully spun. Height is the share " +
             "of the Emitter section's Emission Rate at that moment.\n" +
             "A straight line ramps evenly. A curve that stays low then rises late gives a slow " +
             "growl that suddenly opens up.")]
    public AnimationCurve windUpCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);

    [Tooltip("Seconds to spin back down to cold after you let go of fire.\n" +
             "0 snaps back instantly, which throws away the Minigun feel: tapping fire would give " +
             "full rate every time.")]
    [Min(0f)]
    public float windDownDuration = 0.5f;
}
