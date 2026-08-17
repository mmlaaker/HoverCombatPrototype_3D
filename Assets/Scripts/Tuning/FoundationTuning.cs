using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Foundation. Pure designer-facing
/// values. No scene refs, no runtime state.
///
/// TOOLTIP CONVENTION, set 2026-08-10 at the owner's request. A tooltip is read
/// mid-tune with a hand on the slider, so it answers three things in order:
///
///   1. What does this do, in one plain sentence.
///   2. What happens if I raise it or lower it.
///   3. What else moves when I move this.
///
/// It is NOT a changelog. No dates, no "this used to", no measurements from a
/// specific session, no derivations. Anything that is genuinely a number rather
/// than a feeling belongs in the inspector's derived readouts, which recompute
/// themselves and therefore cannot go stale the way a written figure does.
/// Project history lives in CLAUDE.md, and the reasoning behind a design lives
/// in the consuming module's doc comment.
/// </summary>
[Serializable]
public class FoundationTuning
{
    // -------------------------------------------------------------------------
    // 🚀 Hover Lift
    // -------------------------------------------------------------------------
    [Header("🚀 Hover Lift")]
    [Tooltip("Resting height above the ground, measured from the hover points.\n\n" +
             "This is the literal resting height, on flat ground and on slopes alike. Raise it to " +
             "clear taller obstacles and read more like a hovercraft; lower it to feel planted and " +
             "car-like.\n\n" +
             "Moves with: Sensor Range, which has to stay comfortably above it.")]
    public float hoverHeight = 7f;

    [Tooltip("How far the hover rays look down for ground.\n\n" +
             "The gap between this and Hover Height is your FLOAT BAND: how high you can rise and " +
             "still count as driving. Inside it you keep steering, jump charge and drift entry, so " +
             "the band is also what decides whether a drift survives a bump.\n\n" +
             "Too tight and leaving the ground stops reading cleanly. Too wide and you keep driving " +
             "while visibly airborne. The readout below shows the band; aim for 2 to 3.\n\n" +
             "This does NOT decide when you count as flying. Support Margin does.")]
    public float sensorRange = 9.5f;

    [Tooltip("How far above resting height you rise before the game treats you as flying rather " +
             "than driving.\n\n" +
             "A fade, not a switch: drag and leveling fade out while fall gravity and air control " +
             "fade in, so nothing pops.\n\n" +
             "Tighter reads as a crisp split between driving and flying, but triggers on bump " +
             "crests. Wider is gentler over rough ground, but small jumps feel half committed. " +
             "Try 0.5 to 1.5.")]
    [Min(0.01f)]
    public float supportMargin = 0.75f;

    [Tooltip("How much clear ground you need BELOW you before the stick becomes attitude control.\n\n" +
             "This is the price of entry for tricks. Keep it above the tap jump apex. Drift and air " +
             "control share a button, so if a tap clears this, holding drift through a small hop " +
             "hands over full authority, and since the left stick is throttle on the ground and " +
             "pitch in the air, simply not letting go of forward then commands a hard nose-down and " +
             "plants you.\n\n" +
             "Measured as room below you rather than height gained, which is deliberate: a hop on " +
             "flat ground grants nothing, the same hop off a ledge arms as the ground drops away, " +
             "and a cliff dive arms at once.\n\n" +
             "To make tricks easier to start, lower this or raise Jump Impulse Min. The readout " +
             "below shows what charge fraction actually clears it.")]
    [Min(0f)]
    public float airControlMinClearance = 8f;

    [Tooltip("Lets the chassis squat to fit under low ceilings instead of being crushed against them.\n\n" +
             "Turn this off and any ceiling lower than the craft needs stops being a nuisance and " +
             "becomes a soft-lock: the springs have no force ceiling, so the craft gets pinned and " +
             "friction kills all drive. Level geometry would then need a hard minimum clearance " +
             "instead.")]
    public bool enableCeilingDuck = true;

    [Tooltip("Room the craft wants above its hover points before it starts ducking.\n\n" +
             "Has to cover the hull height above the hover points plus whatever daylight you want.\n\n" +
             "Too low and the roof scrapes. Too high and you duck under tunnels that would have " +
             "cleared fine, giving up ride height for nothing. Try 2.5 to 4.")]
    [Min(0f)]
    public float ceilingClearance = 3f;

    [Tooltip("How low ducking is allowed to push the resting height.\n\n" +
             "A floor on the squat, so a genuinely impassable gap parks the craft low instead of " +
             "reaching for a negative height. Gaps tighter than this plus Ceiling Clearance still " +
             "pin, which is correct: that geometry should read as a wall. Try 1 to 2.")]
    [Min(0.1f)]
    public float minDuckHoverHeight = 1.5f;

    [Tooltip("How hard the hover spring resists being pushed off its resting height, per hover point.\n\n" +
             "Low is easy to shove around with weapons and collisions. High is composed and hard to " +
             "move. This is the only thing it does: gravity is fed forward separately, so the craft " +
             "holds Hover Height whatever you set here.\n\n" +
             "The number looks small because every hover point contributes its own, so the craft " +
             "feels this times the point count.\n\n" +
             "Moves with: Lift Damping. Bounce depends on the RATIO between the two, so raising " +
             "this alone makes the craft bouncier. Raise both together to get firmer without more " +
             "bounce.")]
    public float liftStrength = 16f;

    [Tooltip("How quickly hover bounce dies out after a landing or a hit, per hover point.\n\n" +
             "This is the bounce knob. Raise it to remove wallow, lower it to let the craft visibly " +
             "rock when something hits it.\n\n" +
             "What matters is the ratio against Lift Strength rather than this number alone. The " +
             "readout below turns the pair into an overshoot percentage and a settle time, and " +
             "shows where bouncing stops entirely. Retune this whenever Lift Strength moves.")]
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
    [Tooltip("How aggressively the chassis rotates to match the ground under it.\n\n" +
             "Higher follows terrain faster and reads as planted, but can wobble. Pair it with " +
             "Pitch Roll Damping to settle.\n\n" +
             "Must stay BELOW Flip Recovery Torque, or a flipped craft can never right itself. " +
             "Start around 8 to 15.")]
    [Range(0f, 50f)]
    public float levelingTorqueStrength = 12f;

    [Tooltip("Resistance to tilting on pitch and roll. The wobble killer.\n\n" +
             "Raise it alongside Leveling Torque Strength to stop the ride oscillating over bumps.\n\n" +
             "It does two other jobs worth knowing about: it is the only thing damping aim pitch, " +
             "so raise it if the nose oscillates while aiming, and it sets how fast a flipped craft " +
             "rights itself, since righting speed is Flip Recovery Torque divided by this.")]
    [Range(0f, 30f)]
    public float pitchRollDamping = 8f;

    // -------------------------------------------------------------------------
    // 🎯 Aim Pitch Tracking
    // -------------------------------------------------------------------------
    [Header("🎯 Aim Pitch Tracking")]
    [Tooltip("How hard the nose is driven toward your aim angle in strafe mode.\n\n" +
             "Higher is snappy and shooter-like, lower is heavy and deliberate.\n\n" +
             "Pitch axis ONLY, which is the point of having it: cranking this for snappy aim does " +
             "not stiffen the ride over bumps, because ground following stays on Leveling Torque " +
             "Strength.\n\n" +
             "If the nose overshoots and wobbles, raise Aim Pitch Damping rather than lowering this.")]
    [Range(0f, 300f)]
    public float aimPitchTrackingStrength = 150f;

    [Tooltip("Extra settling on the nose while aiming, on top of Pitch Roll Damping.\n\n" +
             "Lets aim feel be tuned without changing ride stiffness. Raise it if a flick lands and " +
             "then wobbles.\n\n" +
             "It also slows the approach slightly, so raise Aim Pitch Tracking Strength to " +
             "compensate if aim starts feeling heavy.")]
    [Range(0f, 30f)]
    public float aimPitchDamping = 8f;

    // -------------------------------------------------------------------------
    // 🛩 Air Control
    // -------------------------------------------------------------------------
    [Header("🛩 Air Control")]
    [Tooltip("How hard the craft flips nose over tail under air control. Stick up is nose down.\n\n" +
             "Flip speed is this divided by Air Control Damping; the readout below turns that into " +
             "seconds for a full rotation.\n\n" +
             "Keep it BELOW Air Roll Torque. A car-shaped craft should flip slower than it rolls.\n\n" +
             "Aim at the readout rather than at this number. Somewhere around 1.2 to 1.5 s for a " +
             "full flip reads deliberate and stays countable in the air. The raw value here only " +
             "means something next to Air Control Damping, so the two rescale together.")]
    [Range(0f, 60f)]
    public float airPitchTorque = 14f;

    [Tooltip("How hard the craft barrel rolls under air control. Stick right is roll right.\n\n" +
             "Roll speed is this divided by Air Control Damping; the readout below turns that into " +
             "seconds for a full rotation.\n\n" +
             "This is the fastest rotation in the game and the one most tricks are built on.\n\n" +
             "Aim at the readout rather than at this number. Around 0.7 s for a full barrel roll " +
             "reads snappy without becoming a blur. The raw value here only means something next " +
             "to Air Control Damping, so the two rescale together.")]
    [Range(0f, 60f)]
    public float airRollTorque = 24f;

    [Tooltip("How hard rotation is resisted in the air. Sets both the spin ceiling and how cleanly " +
             "you stop.\n\n" +
             "Lower spins faster but coasts past where you released the stick. Higher stops on a " +
             "dime but needs more torque to reach the same speed.\n\n" +
             "This divides BOTH air torques, so moving it rescales roll and flip together. To keep " +
             "the spin speeds you have and just tighten the stop, raise this and both torques by " +
             "the same factor.\n\n" +
             "There is no useful range for this on its own, because only the ratio against the " +
             "torques means anything. Tune it by the stop time in the readout, then put the spin " +
             "speeds back with the torques.")]
    [Range(0f, 30f)]
    public float airControlDamping = 4f;

    // -------------------------------------------------------------------------
    // 📌 Ground Unstick
    // -------------------------------------------------------------------------
    [Header("📌 Ground Unstick")]
    [Tooltip("How long the craft can sit belly-down and upright before the unstick nudge fires.\n\n" +
             "Keep it short. This is a physics correction, not a gameplay moment. Try 0.1 to 0.3.")]
    [Min(0f)]
    public float unstickRecoveryDelay = 0.2f;

    [Tooltip("How steep a surface can be and still count as FLOOR for unstick. Anything steeper is " +
             "treated as a wall and never triggers it.\n\n" +
             "This filter is load-bearing. Without it, driving into a wall reads as being stuck and " +
             "fires a repeating push straight back off it, which cancels full throttle outright and, " +
             "against another vehicle, hands out free knockback that no weapon paid for.\n\n" +
             "If craft still shove off ramps and bowl walls, lower it. 90 makes every surface count. " +
             "Try 50 to 70.")]
    [Range(0f, 90f)]
    public float unstickMaxSurfaceAngle = 60f;

    [Tooltip("Unstick only arms while you are barely moving up or down.\n\n" +
             "A genuinely stuck craft has settled; a belly scrape at speed has not. Without this, " +
             "ramp grinds and bottoming out collect lift pulses on top of springs that are already " +
             "working. Lower it if scrapes still pulse.\n\n" +
             "Horizontal speed is deliberately not checked, so unstick still works right after a " +
             "landing. Try 0.5 to 2.")]
    [Min(0f)]
    public float unstickMaxVerticalSpeed = 1f;

    [Tooltip("How hard the craft is popped up when it is stuck. Front-loaded, fading out over the " +
             "lift window.\n\n" +
             "Enough to break contact, not enough to read as a jump. Tune it by the pop height in " +
             "the readout below rather than by this number, since Unstick Lift Duration changes the " +
             "result too. Try 40 to 80.")]
    [Min(0f)]
    public float unstickLiftForce = 60f;

    [Tooltip("How long the unstick pop lasts as it fades out.\n\n" +
             "Longer reads as a gentle shove, shorter as a snap. It changes the pop height as well, " +
             "so check the readout below after moving it. Try 0.1 to 0.25.")]
    [Min(0.01f)]
    public float unstickLiftDuration = 0.15f;

    // -------------------------------------------------------------------------
    // 🔄 Flip Recovery
    // -------------------------------------------------------------------------
    [Header("🔄 Flip Recovery")]
    [Tooltip("How far over you have to tip before you lose control. 90 is on its side, 180 is fully " +
             "upside down.\n\n" +
             "This is the DOWNED angle, not the rescue angle. Touch the ground past it and control " +
             "is taken away immediately, at any speed. Recovery arms separately, at Flip Recovery " +
             "Arm Angle below.\n\n" +
             "Lower and steep terrain starts taking control off you mid-drive. Try 70 to 100.")]
    [Range(10f, 180f)]
    public float flipRecoveryAngleThreshold = 80f;

    [Tooltip("How far over you have to tip before righting will arm and rescue you.\n\n" +
             "Must sit BELOW the resting balance point, which measures around 78 degrees on this " +
             "chassis: past roughly 70 the hover lift from the two points still touching cancels " +
             "the leveling torque, and the craft settles there. If arming starts ABOVE that balance " +
             "point, a craft that comes to rest inside the gap is stuck forever -- too tipped to " +
             "drive out, not tipped enough to be rescued. That is a real defect this value exists " +
             "to close, caught on 2026-08-08 with the craft parked at 79-point-something against an " +
             "arm angle of 80.\n\n" +
             "Deliberately SEPARATE from the downed threshold above. Sharing one value was the " +
             "obvious fix and it is unsafe: the downed check has no speed gate, so a single number " +
             "low enough to rescue you would also strip control the instant you brushed a steep " +
             "bank. Arming is gated on a full Flip Recovery Delay under Flip Recovery Speed " +
             "Threshold, so it can only ever catch a craft that is genuinely stopped.\n\n" +
             "Keep it at or below the downed angle. Try 65 to 75.")]
    [Range(10f, 180f)]
    public float flipRecoveryArmAngle = 70f;

    [Tooltip("How far upright the craft has to get before the righting torque lets go.\n\n" +
             "Must stay well BELOW Flip Recovery Angle Threshold, and this is not polish. Release " +
             "too near the arm angle and the craft parks in a balance point around 78 degrees, " +
             "where hover lift from the two points still touching cancels the leveling torque. It " +
             "sits there, tips back over, re-arms, and repeats forever.\n\n" +
             "You stay locked out of control until this angle, so lowering it costs you control " +
             "time. Try 30 to 45.")]
    [Range(0f, 80f)]
    public float flipRecoveryReleaseAngle = 35f;

    [Tooltip("How long you lie there before righting starts.\n\n" +
             "This is what the player actually feels as the cost of flipping, because the righting " +
             "itself is near instant. Raise it to make flipping hurt. Try 0.75 to 1.5.")]
    [Min(0f)]
    public float flipRecoveryDelay = 1.0f;

    [Tooltip("How hard the craft is rolled back upright.\n\n" +
             "Must beat Leveling Torque Strength, or righting stalls part way over. Constant rather " +
             "than spring-like, so it does not ease off as the craft comes up.\n\n" +
             "Righting speed is this divided by Pitch Roll Damping, and it is already near instant. " +
             "If you want flipping to feel worse, raise Flip Recovery Delay instead. Try 30 to 70.")]
    [Range(0f, 100f)]
    public float flipRecoveryTorque = 60f;

    [Tooltip("How still the craft has to be before righting is allowed to start. Stops recovery " +
             "firing mid-tumble.\n\n" +
             "Set it by how still a STUCK craft actually is, not by how still you imagine it is. " +
             "The timer resets the instant this is exceeded, so a hull resting on a curved face and " +
             "micro-rocking will keep resetting and never recover, which looks exactly like " +
             "recovery being broken. Try 1 to 3.")]
    [Min(0f)]
    public float flipRecoverySpeedThreshold = 2f;

    // -------------------------------------------------------------------------
    // 🌎 Gravity
    // -------------------------------------------------------------------------
    [Header("🌎 Gravity")]
    [Tooltip("Extra gravity all the time, as a multiple of normal gravity added on top of it. " +
             "Pulls the craft down harder onto its springs for a heavier feel.\n\n" +
             "The most load-bearing number in the profile, and it does not look like it. Every jump " +
             "height, airtime and landing speed is derived from it, and it also scales how firmly " +
             "the springs hold ride height.\n\n" +
             "Moving it invalidates at minimum: Hard Landing Min Speed, both jump impulses, and the " +
             "Extra Fall Gravity balance. Expect every readout in this profile to move with it.")]
    [Range(0f, 5f)]
    public float extraGravityMultiplier = 0f;

    // A symmetric extraAirGravity used to sit here, applying to the rise and the fall
    // equally. Removed: it only ever shortened the whole arc, which is the same trade as
    // raising gravity outright, and the anti-float job it existed for is done better and
    // asymmetrically by extraFallGravity below.

    [Tooltip("Extra downward pull applied only while falling. The anti-float knob.\n\n" +
             "Raising gravity overall kills float, but it flattens the rise too and forces bigger " +
             "jump impulses to win the height back. Putting the weight on the descent alone keeps " +
             "the rise generous, which is where tricks live, while the landing still reads as " +
             "decisive. Rise time goes up, fall time comes down, total airtime barely moves.\n\n" +
             "DO NOT REACH FOR THIS TO SHORTEN HANG TIME. It is a weak lever on airtime and a " +
             "strong one on descent weight: 30 to 40 is 14% heavier on the way down but only ~3% " +
             "less time in the air, because the rise runs on separate gravity and the fall " +
             "shortens by a square root. If the complaint is 'too long in the air', this knob " +
             "will disappoint you at every value.\n\n" +
             "TWO CEILINGS, both measured rather than guessed:\n" +
             "- 43 is where a flat-ground charge jump starts tripping Hard Landing Min Speed. A " +
             "charge jump must never hard-land (Standing Decision), so that is a hard cap. The " +
             "slider stops at 40 to keep a margin.\n" +
             "- Every +5 here costs about 25ms of the window between finishing two barrel rolls " +
             "and touching down. At the shipped 35 that window is ~0.09s.\n\n" +
             "The catch: a jump now lands FASTER than it launched. Recheck Hard Landing Min Speed " +
             "whenever you move this. Shipped 35 and JUDGED GOOD on the condition that two barrel " +
             "rolls still land, so test that before accepting any new value. History in " +
             "TuningLog.md.")]
    [Range(0f, 40f)]
    public float extraFallGravity = 13f;

    // -------------------------------------------------------------------------
    // 💥 Hard Landing
    // -------------------------------------------------------------------------
    [Header("💥 Hard Landing")]
    [Tooltip("Falls faster than Hard Landing Min Speed make the hover springs give way, so the " +
             "craft slams the ground before popping back to ride height.\n\n" +
             "Feel only: no damage, no control loss.")]
    public bool enableHardLanding = true;

    [Tooltip("How fast you have to be falling for a landing to slam.\n\n" +
             "Keep it above the fastest speed an ordinary jump can land at, or every jump slams and " +
             "the effect stops meaning anything. The readout below works out both ordinary landing " +
             "speeds and shows you the margin.\n\n" +
             "Moves with: both jump impulses, Extra Fall Gravity, and Sensor Range. That last one " +
             "is the surprise. A longer sensor range catches the landing higher up, where you are " +
             "still slower, so it makes hard landings LESS likely.")]
    [Min(0f)]
    public float hardLandingMinSpeed = 58f;

    [Tooltip("The falling speed that reads at full severity. Anything faster is capped here.\n\n" +
             "The gap between this and Hard Landing Min Speed is your expressive range. Narrow it " +
             "and almost every hard landing looks maximal; widen it and severity scales gradually " +
             "with how badly you misjudged the drop.\n\n" +
             "Set it relative to the threshold rather than on its own. Roughly 20 to 40 above Hard " +
             "Landing Min Speed keeps the scaling readable, and it has to move whenever that " +
             "threshold does.")]
    [Min(0f)]
    public float hardLandingMaxSpeed = 85f;

    [Tooltip("How much of the hover spring gives way at full severity.\n\n" +
             "1 cuts the springs out completely so the craft free-falls onto its collider. 0.5 is a " +
             "heavy sag. Try 0.8 to 1.")]
    [Range(0f, 1f)]
    public float hardLandingSuppressStrength = 0.9f;

    [Tooltip("How long the springs stay weak after a slam. Weakest on the first frame, recovering " +
             "from there.\n\n" +
             "Longer reads heavier but delays the pop back to ride height. Keep it under about 0.5 " +
             "so ground unstick never arms during the slam. Try 0.25 to 0.5.")]
    [Min(0.05f)]
    public float hardLandingSuppressDuration = 0.35f;
}
