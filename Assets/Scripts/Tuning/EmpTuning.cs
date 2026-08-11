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

    [Tooltip("Energy paid once per shot, on activation.\n\n" +
             "Deliberately the most expensive thing you can do. Read it against Max Energy: at 70 " +
             "of 100 it costs most of a tank, so firing an EMP means giving up boost and shield " +
             "until you have refilled. That trade IS the ability.")]
    [Min(0f)]
    public float empEnergyCost = 70f;

    [Tooltip("How long a hit vehicle is frozen.\n\n" +
             "Stacks: multiple hits add their durations together on the receiver. Long enough to " +
             "convert into a kill is the bar to aim at, so tune it against how long it takes you to " +
             "close distance and land shots.")]
    [Min(0.05f)]
    public float empFreezeDuration = 2.5f;

    // -------------------------------------------------------------------------
    // 🎯 Soft-Homing Acquisition
    // -------------------------------------------------------------------------
    [Header("🎯 Soft-Homing Acquisition")]
    [Tooltip("How far away the EMP will look for something to curve toward.\n\n" +
             "Set it by the range you actually fight at. Beyond this the shot still fires, it just " +
             "flies straight.")]
    [Min(1f)]
    public float empScanRange = 50f;

    [Tooltip("How far off-centre a target can be and still get picked up, measured from straight " +
             "ahead.\n\n" +
             "Wider is more forgiving to aim with, but makes it harder to choose WHICH of two " +
             "nearby targets you hit. Narrow it if the EMP keeps grabbing the wrong one.")]
    [Range(1f, 60f)]
    public float empScanConeAngle = 25f;
}
