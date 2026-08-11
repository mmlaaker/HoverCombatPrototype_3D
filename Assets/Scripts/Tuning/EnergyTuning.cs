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
    [Tooltip("The size of your whole ability budget. An abstract unit: only its ratio to the costs " +
             "means anything.\n\n" +
             "Read it against the spends elsewhere in this profile (EMP, Shield, jump, dodge, and " +
             "boost per second). Raising this makes every ability cheaper in practice, so it is the " +
             "one knob that loosens all of them at once.")]
    [Min(1f)]
    public float maxEnergy = 100f;

    [Tooltip("Energy on spawn. Set it to Max Energy to start full.")]
    [Min(0f)]
    public float startingEnergy = 100f;

    // -------------------------------------------------------------------------
    // 🔋 Regeneration
    // -------------------------------------------------------------------------
    [Header("🔋 Regeneration")]
    [Tooltip("How fast the pool refills, once the lockout has expired.\n\n" +
             "With Regen Delay, this is what sets the tempo of the whole game: how long after " +
             "spending before you can act again. The boost readout in Propulsion shows the full " +
             "burn-and-refill cycle.")]
    [Min(0f)]
    public float regenRate = 20f;

    [Tooltip("How long after spending before refilling starts.\n\n" +
             "This is the punish window, and it is what makes a spend a commitment rather than a " +
             "rhythm. Raise it to make emptying the tank genuinely costly. Set 0 to remove the " +
             "pause entirely and let energy refill the instant you stop spending.")]
    [Min(0f)]
    public float regenDelay = 1f;
}
