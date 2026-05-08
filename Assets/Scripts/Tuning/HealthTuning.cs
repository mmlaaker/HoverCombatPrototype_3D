using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by VehicleHealth. Pure designer-facing values.
/// No scene refs, no runtime state.
/// </summary>
[Serializable]
public class HealthTuning
{
    // -------------------------------------------------------------------------
    // ❤️ Health
    // -------------------------------------------------------------------------
    [Header("❤️ Health")]
    [Tooltip("Maximum HP.")]
    [Min(1f)]
    public float maxHealth = 100f;

    [Tooltip("Starting HP on spawn. Set to Max Health to start full.")]
    [Min(0f)]
    public float startingHealth = 100f;
}
