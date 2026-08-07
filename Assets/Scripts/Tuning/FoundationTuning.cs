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
    [Tooltip("How far above the ground the chassis floats at rest, measured from the hover points. " +
             "Gravity is fed forward per point, so this is the LITERAL resting height on flat ground " +
             "and on slopes: the springs no longer have to compress to hold the craft up, and the old " +
             "roughly 1m sag below this value is gone.")]
    public float hoverHeight = 7f;

    [Tooltip("How far the hover rays look down for ground. Must stay comfortably above Hover Height so " +
             "the chassis can drop before losing lift.\n" +
             "This also defines GROUNDED, which is the part that bites. Between Hover Height and this " +
             "value the springs produce no lift at all, but the craft still counts as grounded: leveling " +
             "torque and drag keep applying and air control stays off. So this is really 'how far can it " +
             "rise before it stops being a ground vehicle'. Keep the gap to Hover Height tight, around " +
             "2 to 3 metres, or leaving the ground stops reading cleanly.")]
    public float sensorRange = 9.5f;

    [Tooltip("Stiffness of the hover spring, PER HOVER POINT. Higher values make the chassis correct height " +
             "faster and feel firmer; push too high and the vehicle starts bouncing.\n" +
             "Force is applied as mass-independent acceleration and every hover point contributes its own, " +
             "so the stiffness the chassis actually feels is this value times the number of hover points. " +
             "At 4 points and 16 that is 64 m/s^2 per metre of compression. This is why the number looks small.\n" +
             "It no longer sets resting height. Gravity is fed forward per point, so the springs hold the " +
             "craft at Hover Height regardless of what this is set to, and this controls only how hard the " +
             "chassis resists being pushed off that height. Tune it purely for that: lower is easier to " +
             "shove around with weapons and collisions, higher is more composed.")]
    public float liftStrength = 16f;

    [Tooltip("Damping on the hover spring, PER HOVER POINT. Absorbs bounce after landings and slope transitions.\n" +
             "This and Lift Strength together set how the chassis settles. Damping ratio is " +
             "(points x this) / (2 x sqrt(points x Lift Strength)). Below 1 the craft overshoots and bobs; " +
             "at 1 it settles without ever crossing the target.\n" +
             "At 4 points, Lift Strength 16 and this at 2.2 the ratio is 0.55: about 13% overshoot and " +
             "0.9s to settle. That is the intended sedan feel, loose enough to visibly rock when something " +
             "hits it, quick enough to stay drivable. Drop toward 1.6 for more wallow, push toward 3.2 to " +
             "remove the bounce entirely. Retune whenever Lift Strength changes, since the ratio depends " +
             "on both.")]
    public float liftDamping = 2.2f;

    // Slope lift compensation used to live here. Removed: the gravity feedforward in
    // ApplyHoverForces is scaled by the ground normal, so it already supports exactly the
    // slope-adjusted share of the weight (G*cos0) and holds ride height on any incline
    // without compressing the spring. Verified at 7.000m on a 30 degree slope with the
    // old multiplier already neutralised to 1.0, where the spring-only model sagged 0.53m.

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
    [Tooltip("Tilt angle that counts as flipped, and the angle at which recovery ARMS. " +
             "90 is on its side, 180 is fully upside down. Try 70 to 100.")]
    [Range(10f, 180f)]
    public float flipRecoveryAngleThreshold = 80f;

    [Tooltip("Tilt angle at which the righting torque lets go. Deliberately LOWER than Flip Recovery " +
             "Angle Threshold: recovery arms at the threshold and holds until this.\n" +
             "Load-bearing, not polish. Releasing at the arm threshold instead left the craft parked in a " +
             "hover-supported equilibrium at about 78 degrees, where the one-sided spring lift from the two " +
             "hover points that still reach the ground exactly balances Leveling Torque Strength. It sat " +
             "there, drifted back over the threshold, re-armed, and repeated forever. Righting has to hand " +
             "over well clear of that band, not at its edge.\n" +
             "Raising this toward the arm threshold reintroduces the stall. Lowering it holds player " +
             "control lockout for longer, since the craft counts as downed until this angle. Try 30 to 45.")]
    [Range(0f, 80f)]
    public float flipRecoveryReleaseAngle = 35f;

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
             "Prevents recovery from firing while the chassis is still tumbling.\n" +
             "Set this by how still a STUCK craft actually is, not by how still you imagine it is. " +
             "The timer resets the instant this is exceeded, so a hull that comes to rest on a curved " +
             "face and micro-rocks will keep resetting and never recover, which looks identical to " +
             "recovery being broken. 0.5 was too tight for this chassis; 2 is the current guess and " +
             "is unconfirmed. Try 1 to 3.")]
    [Min(0f)]
    public float flipRecoverySpeedThreshold = 2f;

    // -------------------------------------------------------------------------
    // 🌎 Gravity
    // -------------------------------------------------------------------------
    [Header("🌎 Gravity")]
    [Tooltip("Extra gravity applied at all times. Pulls the chassis down harder onto the hover springs for a heavier feel. 0 disables.")]
    [Range(0f, 5f)]
    public float extraGravityMultiplier = 0f;

    // A symmetric extraAirGravity used to sit here, applying to the rise and the fall
    // equally. Removed: it only ever shortened the whole arc, which is the same trade as
    // raising gravity outright, and the anti-float job it existed for is done better and
    // asymmetrically by extraFallGravity below.

    [Tooltip("Extra downward pull applied ONLY while airborne and descending. " +
             "This is the anti-float knob.\n" +
             "Raising gravity globally kills float, but it also flattens jump arcs and " +
             "forces big jump impulses to win the height back. Putting the weight on the " +
             "descent only lets the rise stay generous, which is where air tricks live, " +
             "while the landing still reads as decisive. Rise time goes up, fall time " +
             "comes down, total hangtime barely moves.\n" +
             "At 13 with the default gravity tuning, a max-charge jump rises for 1.02s and " +
             "falls for 0.88s. Try 8 to 20.\n" +
             "IMPORTANT: this breaks the rule that a jump lands at the speed it launched " +
             "with, because the fall is now faster than the rise. Hard Landing Min Speed " +
             "has to be derived from the descent, not from Jump Impulse Max. Recheck it " +
             "whenever you move this.")]
    [Range(0f, 40f)]
    public float extraFallGravity = 13f;

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
             "Jumps write velocity directly, so landing speed is computable rather than guesswork:\n" +
             "  launch = sqrt(Jump Impulse Max^2 + Air Jump Impulse^2)   [drop the second term for a plain jump]\n" +
             "  landing = launch x sqrt(fall gravity / rise gravity)\n" +
             "Because Extra Fall Gravity makes the descent heavier than the ascent, a jump no longer lands " +
             "at the speed it launched with. It lands FASTER, by that square root.\n" +
             "At the current tuning (rise 39.24, fall 52.24, so the ratio is 1.154) a max-charge jump lands " +
             "at 46.2 m/s and a stacked air jump at 54.4. This sitting at 58 clears both with margin.\n" +
             "Those two figures are the speed at RIDE HEIGHT. The check actually samples on the " +
             "airborne-to-grounded edge, which happens at Sensor Range, so the measured value is a few " +
             "percent lower again (about 43 and 52 here). The formula therefore over-estimates slightly, " +
             "which is the safe direction. It also means raising Sensor Range makes hard landings " +
             "LESS likely, because the edge fires higher up where the craft is still slower.\n" +
             "Recheck it whenever you move either jump impulse, Extra Fall Gravity, OR Sensor Range.")]
    [Min(0f)]
    public float hardLandingMinSpeed = 58f;

    [Tooltip("Downward speed at which the crash reads at full severity. Roughly a 60m drop at default " +
             "gravity tuning. Try 65 to 80.")]
    [Min(0f)]
    public float hardLandingMaxSpeed = 85f;

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
