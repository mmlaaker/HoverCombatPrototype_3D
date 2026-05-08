using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Energy. Pure designer-facing
/// values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class EnergyTuning
{
    // -------------------------------------------------------------------------
    // ⚡ Pool
    // -------------------------------------------------------------------------
    [Header("⚡ Pool")]
    [Tooltip("Maximum energy. Treat as an abstract unit, tuned relative to ability costs.")]
    [Min(1f)]
    public float maxEnergy = 100f;

    [Tooltip("Energy on spawn. Set to Max Energy to start full.")]
    [Min(0f)]
    public float startingEnergy = 100f;

    // -------------------------------------------------------------------------
    // 🔋 Regeneration
    // -------------------------------------------------------------------------
    [Header("🔋 Regeneration")]
    [Tooltip("How fast the pool refills per second once the lockout expires.")]
    [Min(0f)]
    public float regenRate = 20f;

    [Tooltip("How long after a spend attempt before regen resumes. Set to 0 to disable the lockout.")]
    [Min(0f)]
    public float regenDelay = 1f;
}
