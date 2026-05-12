using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_EMP. Pure designer-facing
/// values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class EmpTuning
{
    // -------------------------------------------------------------------------
    // ⚡ EMP
    // -------------------------------------------------------------------------
    [Header("⚡ EMP")]
    [Tooltip("Master toggle. When false, the EMP component is inert and TryActivate always fails.")]
    public bool enableEmp = true;

    [Tooltip("Flat energy cost paid once per shot on activation. " +
             "High by design — EMP is the most expensive of the four energy abilities.")]
    [Min(0f)]
    public float empEnergyCost = 70f;

    [Tooltip("Seconds of EMP freeze applied on hit. Additive on the receiver — " +
             "multiple hits stack duration.")]
    [Min(0.05f)]
    public float empFreezeDuration = 2.5f;

    // -------------------------------------------------------------------------
    // 🎯 Soft-Homing Acquisition
    // -------------------------------------------------------------------------
    [Header("🎯 Soft-Homing Acquisition")]
    [Tooltip("Maximum distance to consider a target for soft-homing acquisition. " +
             "Tune against expected engagement range.")]
    [Min(1f)]
    public float empScanRange = 50f;

    [Tooltip("Half-angle (degrees) of the forward acquisition cone. " +
             "Same convention as WeaponDefinition.lockConeAngle. Wider = more forgiving aim.")]
    [Range(1f, 60f)]
    public float empScanConeAngle = 25f;
}
