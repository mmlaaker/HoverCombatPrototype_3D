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

    [Tooltip("Stiffness of the hover spring, PER HOVER POINT. Higher values make the chassis correct height " +
             "faster and feel firmer; push too high and the vehicle starts bouncing.\n" +
             "Force is applied as mass-independent acceleration and every hover point contributes its own, " +
             "so the stiffness the chassis actually feels is this value times the number of hover points. " +
             "At 4 points and 10 that is 40 m/s^2 per metre of compression. This is why the number looks small.\n" +
             "It also sets the resting sag: the springs settle where lift balances weight, so " +
             "sag = total gravity / total stiffness. At default gravity tuning (39.24) and 40 that is " +
             "0.98m below Hover Height. Raising this reduces sag proportionally.")]
    public float liftStrength = 10f;

    [Tooltip("Damping on the hover spring, PER HOVER POINT. Absorbs bounce after landings and slope transitions.\n" +
             "This and Lift Strength together set how the chassis settles. Damping ratio is " +
             "(points x this) / (2 x sqrt(points x Lift Strength)). Below 1 the craft overshoots and bobs; " +
             "at 1 it settles without ever crossing the target.\n" +
             "At 4 points, Lift Strength 10 and this at 1.5 the ratio is 0.47: about 18% overshoot and " +
             "1.3s to settle, which reads as a soft float. Around 2.2 gives a firmer 0.7 ratio; " +
             "3.2 removes the bounce entirely. Retune whenever Lift Strength changes.")]
    public float liftDamping = 1.5f;

    // -------------------------------------------------------------------------
    // 🧮 Slope Lift Compensation
    // -------------------------------------------------------------------------
    [Header("🧮 Slope Lift Compensation")]
    [Tooltip("Adds extra lift on slopes so the chassis doesn't sag into hills. Recommended on for any non-flat terrain.")]
    public bool enableSlopeLiftCompensation = true;

    [Tooltip("How much extra lift the slope correction adds AT A VERTICAL WALL. " +
             "1.0 disables the boost, 1.3 is subtle, 1.6 is strong.\n" +
             "Read the range carefully: this is the value at 90 degrees, and the ramp toward it is not " +
             "linear in angle. It scales on (1 - normal.y), which is only 0.13 at a 30 degree slope and " +
             "0.29 at 45. So a setting of 1.15 delivers just 1.02x on a 30 degree hill, which is " +
             "effectively nothing. If you want a real effect on the slopes you actually drive, this has " +
             "to sit well above the value you have in mind for them.")]
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
             "Higher values feel more planted. Also the sole oscillation killer for aim pitch tracking.")]
    [Range(0f, 30f)]
    public float pitchRollDamping = 8f;

    // -------------------------------------------------------------------------
    // 🎯 Aim Pitch Tracking
    // -------------------------------------------------------------------------
    [Header("🎯 Aim Pitch Tracking")]
    [Tooltip("Torque strength while an aim pitch target is active (strafe mode). Propulsion sets the target angle " +
             "via SetAimPitch and leveling drives toward it as the single torque authority, replacing the old " +
             "design where Propulsion's pitch torque fought leveling (competing forces = jitter). " +
             "Applies to the PITCH AXIS ONLY: ground-normal alignment (roll, bump following) stays at Leveling " +
             "Torque Strength, so cranking this for FPS-snappy aim does not stiffen the ride over bumps. " +
             "Pitch Roll Damping kills the overshoot; raise it if the nose oscillates.")]
    [Range(0f, 300f)]
    public float aimPitchTrackingStrength = 150f;

    [Tooltip("EXTRA pitch-rate damping applied only while aiming, on top of Pitch Roll Damping, scaled by the " +
             "strafe blend. Lets aim settle be tuned independently of ride stiffness. With tracking strength 150: " +
             "Pitch Roll Damping 8 alone = ~35% overshoot on a flick; +8 here (16 total) = lands clean with snap. " +
             "More damping also slows the approach slightly; raise tracking strength to compensate if it feels heavy.")]
    [Range(0f, 30f)]
    public float aimPitchDamping = 8f;

    // -------------------------------------------------------------------------
    // 🛩 Air Control
    // -------------------------------------------------------------------------
    [Header("🛩 Air Control")]
    [Tooltip("Pitch torque while air control is active (drift held airborne). Stick up = nose down. " +
             "Steady flip rate is this divided by Air Control Damping: 14 over 4 = ~200 deg/s, a full flip " +
             "in about 2 seconds. Keep below Air Roll Torque: a car-shaped craft flips slower than it rolls. " +
             "Try 10 to 20.")]
    [Range(0f, 60f)]
    public float airPitchTorque = 14f;

    [Tooltip("Roll torque while air control is active. Stick right = roll right. Steady roll rate is this " +
             "divided by Air Control Damping: 24 over 4 = ~344 deg/s, a full barrel roll in just over a second. " +
             "Try 18 to 32.")]
    [Range(0f, 60f)]
    public float airRollTorque = 24f;

    [Tooltip("Pitch/roll damping while air control is active, replacing Pitch Roll Damping as the air-control " +
             "blend rises. Sets both the rotation ceiling (rate = torque / this) and stop precision on stick " +
             "release (~95% stopped in 3 / this seconds). Lower spins faster but drifts past the release point; " +
             "higher stops on a dime but needs more torque. Try 3 to 6.")]
    [Range(0f, 30f)]
    public float airControlDamping = 4f;

    // -------------------------------------------------------------------------
    // 📌 Ground Unstick
    // -------------------------------------------------------------------------
    [Header("📌 Ground Unstick")]
    [Tooltip("How long the chassis can rest belly-down upright before the unstick lift fires. " +
             "Keep this short. This is a physics correction, not a gameplay moment. Try 0.1 to 0.3.")]
    [Min(0f)]
    public float unstickRecoveryDelay = 0.2f;

    [Tooltip("Unstick only arms while vertical speed is below this (m/s). A genuinely stuck craft has settled " +
             "vertically; a dynamic belly scrape (ramp grind, bottoming out at speed) has not, so scrapes no longer " +
             "receive periodic lift pulses on top of working hover springs. Horizontal speed remains ungated: " +
             "unstick still works after any landing. Try 0.5 to 2.")]
    [Min(0f)]
    public float unstickMaxVerticalSpeed = 1f;

    [Tooltip("Strength of the upward push that frees a stuck chassis. " +
             "Front-loaded and fades over the lift window. Mass independent. Try 40 to 80.\n" +
             "The push tapers linearly to zero, so the speed it actually adds is " +
             "this x Unstick Lift Duration / 2. At 60 over 0.15s that is 4.5 m/s.")]
    [Min(0f)]
    public float unstickLiftForce = 60f;

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
             "Must be stronger than Leveling Torque Strength to win at extreme angles. Try 30 to 70.\n" +
             "Unlike leveling, this torque is constant regardless of how far over the craft is, so it does " +
             "not ease off as the chassis comes back up. The righting speed it settles at is this divided by " +
             "Pitch Roll Damping: 60 over 8 is about 430 deg/s, so the flip itself is near instant and " +
             "Flip Recovery Delay is what the player actually feels as the setback.")]
    [Range(0f, 100f)]
    public float flipRecoveryTorque = 60f;

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

    // -------------------------------------------------------------------------
    // 💥 Hard Landing
    // -------------------------------------------------------------------------
    [Header("💥 Hard Landing")]
    [Tooltip("Master switch for hard landing impacts. When on, falls faster than Hard Landing Min Speed " +
             "make the hover springs give way so the chassis slams the ground before popping back to ride " +
             "height. Feel only: no damage, no control loss.")]
    public bool enableHardLanding = true;

    [Tooltip("Downward speed (m/s along the ground normal) at the moment the hover sensors first see ground " +
             "that counts as a hard landing. Keep this above the fastest jump return speed so ordinary jumps " +
             "never trigger it.\n" +
             "Jumps write velocity directly, so there is no guesswork and gravity does not enter into it. " +
             "A jump returns to the ground at exactly the speed it launched with: Jump Impulse Max. Stacking " +
             "an air jump at the peak lands at sqrt(Jump Impulse Max^2 + Air Jump Impulse^2).\n" +
             "At the current 40 and 25 that is 40 m/s for a max-charge jump and 47.2 m/s for a stacked one, " +
             "so this sitting at 45 IS crossed by the stacked jump. Severity comes out around 0.09, which " +
             "reads as a faint dust puff rather than a slam, but it is a false positive: set this above 48 " +
             "to clear it. Recheck the sqrt whenever you retune either jump impulse.")]
    [Min(0f)]
    public float hardLandingMinSpeed = 45f;

    [Tooltip("Downward speed at which the crash reads at full severity. Roughly a 60m drop at default " +
             "gravity tuning. Try 65 to 80.")]
    [Min(0f)]
    public float hardLandingMaxSpeed = 70f;

    [Tooltip("How much of the hover spring gives way at full severity. 1 cuts the springs out completely " +
             "so the chassis free-falls onto its collider; 0.5 is a heavy sag. Try 0.8 to 1.")]
    [Range(0f, 1f)]
    public float hardLandingSuppressStrength = 0.9f;

    [Tooltip("How long the springs stay weakened after a hard landing. Front-loaded: weakest on the first " +
             "frame, tapering back to full strength. Longer reads heavier but delays the pop back to ride " +
             "height. Keep under ~0.5 so ground unstick never arms during the slam. Try 0.25 to 0.5.")]
    [Min(0.05f)]
    public float hardLandingSuppressDuration = 0.35f;
}
