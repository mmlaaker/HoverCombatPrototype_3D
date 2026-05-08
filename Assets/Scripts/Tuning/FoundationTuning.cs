using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Foundation. Pure designer-facing
/// values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class FoundationTuning
{
    // -------------------------------------------------------------------------
    // 🚀 Hover Lift
    // -------------------------------------------------------------------------
    [Header("🚀 Hover Lift")]
    [Tooltip("How far above the ground the chassis floats at rest. Higher values lift the body further off the surface.")]
    public float hoverHeight = 3f;

    [Tooltip("How far the hover rays look down for ground. Set this generous enough that the chassis can drop a small distance before losing lift.")]
    public float sensorRange = 5f;

    [Tooltip("Stiffness of the hover spring. Higher values make the chassis correct height faster and feel firmer. " +
             "Push too high and the vehicle starts bouncing.")]
    public float liftStrength = 50000f;

    [Tooltip("Damping on the hover spring. Higher values absorb bounce after landings or slope transitions. " +
             "Pair with Lift Strength.")]
    public float liftDamping = 5000f;

    // -------------------------------------------------------------------------
    // 🧮 Slope Lift Compensation
    // -------------------------------------------------------------------------
    [Header("🧮 Slope Lift Compensation")]
    [Tooltip("Adds extra lift on slopes so the chassis doesn't sag into hills. Recommended on for any non-flat terrain.")]
    public bool enableSlopeLiftCompensation = true;

    [Tooltip("How much extra lift the slope correction adds at the steepest angle. " +
             "1.0 disables the boost, 1.3 is subtle, 1.6 is strong.")]
    [Range(1f, 2f)]
    public float slopeLiftMultiplier = 1.3f;

    // -------------------------------------------------------------------------
    // ⚖️ Leveling
    // -------------------------------------------------------------------------
    [Header("⚖️ Leveling")]
    [Tooltip("How aggressively the chassis rotates to match the ground beneath it. " +
             "Higher levels out faster but can wobble. Start around 8 to 15.")]
    [Range(0f, 50f)]
    public float levelingTorqueStrength = 12f;

    [Tooltip("Resistance to tilting on the pitch and roll axes. Pair with Leveling Torque Strength to kill wobble. " +
             "Higher values feel more planted.")]
    [Range(0f, 30f)]
    public float pitchRollDamping = 8f;

    // -------------------------------------------------------------------------
    // 📌 Ground Unstick
    // -------------------------------------------------------------------------
    [Header("📌 Ground Unstick")]
    [Tooltip("How long the chassis can rest belly-down upright before the unstick lift fires. " +
             "Keep this short. This is a physics correction, not a gameplay moment. Try 0.1 to 0.3.")]
    [Min(0f)]
    public float unstickRecoveryDelay = 0.2f;

    [Tooltip("Strength of the upward push that frees a stuck chassis. " +
             "Front-loaded and fades over the lift window. Mass independent. Try 40 to 80.")]
    [Min(0f)]
    public float unstickLiftForce = 25f;

    [Tooltip("How long the unstick lift lasts as it tapers to zero. " +
             "Longer reads as a gentle bump, shorter is a quick snap. Try 0.1 to 0.25.")]
    [Min(0.01f)]
    public float unstickLiftDuration = 0.15f;

    // -------------------------------------------------------------------------
    // 🔄 Flip Recovery
    // -------------------------------------------------------------------------
    [Header("🔄 Flip Recovery")]
    [Tooltip("Tilt angle that counts as flipped. 90 is on its side, 180 is fully upside down. Try 70 to 100.")]
    [Range(10f, 180f)]
    public float flipRecoveryAngleThreshold = 80f;

    [Tooltip("How long the chassis stays flipped before the righting torque kicks in. " +
             "Longer values let the player feel the flip as a real setback. Try 0.75 to 1.5.")]
    [Min(0f)]
    public float flipRecoveryDelay = 1.0f;

    [Tooltip("Strength of the righting torque that flips the chassis upright. " +
             "Must be stronger than Leveling Torque Strength to win at extreme angles. Try 20 to 40.")]
    [Range(0f, 250f)]
    public float flipRecoveryTorque = 28f;

    [Tooltip("Speed below which flip recovery is allowed to start. " +
             "Prevents recovery from firing while the chassis is still tumbling. Try 0.5.")]
    [Min(0f)]
    public float flipRecoverySpeedThreshold = 0.5f;

    // -------------------------------------------------------------------------
    // 🌎 Gravity
    // -------------------------------------------------------------------------
    [Header("🌎 Gravity")]
    [Tooltip("Extra gravity applied at all times. Pulls the chassis down harder onto the hover springs for a heavier feel. 0 disables.")]
    [Range(0f, 5f)]
    public float extraGravityMultiplier = 0f;

    [Tooltip("Extra downward pull that only applies while airborne. " +
             "Reduces hangtime after jumps and ramps without affecting grounded feel.")]
    [Range(0f, 30f)]
    public float extraAirGravity = 0f;
}
