using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Shield. Pure designer-facing
/// values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class ShieldTuning
{
    // -------------------------------------------------------------------------
    // 🛡 Shield
    // -------------------------------------------------------------------------
    [Header("🛡 Shield")]
    [Tooltip("Master toggle. When false, the shield component is inert and TryActivate always fails.")]
    public bool enableShield = true;

    [Tooltip("How long the shield stays active after a successful activation. " +
             "Cannot be cancelled by the player. EMP cancels it immediately.")]
    [Min(0.05f)]
    public float shieldDuration = 2f;

    [Tooltip("Flat energy cost paid once on activation. Higher than other abilities " +
             "since shield is the only one granting full damage immunity.")]
    [Min(0f)]
    public float shieldEnergyCost = 35f;
}
