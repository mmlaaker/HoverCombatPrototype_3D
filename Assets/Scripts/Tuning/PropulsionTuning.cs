using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Propulsion. Pure designer-facing
/// values. No scene refs, no runtime state.
///
/// See FoundationTuning's doc comment for the tooltip convention these follow:
/// what it does, what raising or lowering feels like, what else moves with it.
/// No history, no derivations, no session measurements. Live numbers belong in
/// the inspector's derived readouts, which recompute and cannot go stale.
/// </summary>
[Serializable]
public class PropulsionTuning
{
    // -------------------------------------------------------------------------
    // 🚀 Drive
    // -------------------------------------------------------------------------
    [Header("🚀 Drive")]
    [Tooltip("How hard the craft accelerates forward at full throttle, before boost.\n\n" +
             "This is the pickup knob. Raise it to leap off the line, lower it for weight and " +
             "inertia. Top Speed decides where you end up; this decides how fast you get there.\n\n" +
             "Cannot be 0.")]
    [Min(0.01f)]
    public float maxForwardAccel = 25f;

    [Tooltip("Forward top speed before boost.\n\n" +
             "Also the reference the strafe and reverse caps are measured against, so moving it " +
             "quietly rescales both. Cannot be 0.")]
    [Min(0.01f)]
    public float topSpeed = 40f;

    [Tooltip("The SHAPE of the acceleration: how much of Max Forward Accel you get at each " +
             "fraction of the top speed currently in force.\n\n" +
             "Flat at 1.0 is no shape at all — full thrust until the ceiling cuts it off, which " +
             "reaches top speed in under a second and is what one playtester called far too quick. " +
             "Falling to the right buys a long tail: the launch is unchanged and the last stretch " +
             "has to be earned.\n\n" +
             "WHICH KEY TO DRAG DEPENDS ON THE COMPLAINT, and they are not interchangeable. " +
             "The RIGHT-HAND end only buys back the last quarter of the range: raising it five-fold " +
             "moves time-to-top-speed from about 2.4s to 1.7s and leaves time-to-40 and time-to-60 " +
             "where they were. If the craft feels SLUGGISH rather than slow to top out, that is the " +
             "MIDDLE keys, and lifting the whole shape toward 1.0 is the move.\n\n" +
             "Keep the right-hand end above zero or top speed stops being reachable at all.\n\n" +
             "Read against a FRACTION of the current ceiling, not against metres per second, so " +
             "boost and strafe get the same shape measured against their own top speeds.\n\n" +
             "Applies to forward drive in both modes and to airborne boost. NOT to reverse, so " +
             "braking is untouched.")]
    public AnimationCurve accelCurve = DefaultAccelCurve();

    [Tooltip("How hard the craft accelerates backwards, and also THE BRAKE.\n\n" +
             "Pulling reverse while still moving forward slows you at this rate, so set it by how " +
             "fast you want to STOP rather than how fast you want to reverse.\n\n" +
             "Deliberately not blended by strafe, so braking feels identical in both modes.")]
    public float maxReverseAccel = 15f;

    [Tooltip("Reverse top speed in DRIVE MODE ONLY.\n\n" +
             "Strafe mode ignores this. The cap blends toward Strafe Top Speed as strafe comes in, " +
             "so at full strafe forward, reverse and sideways all share one ceiling and the craft " +
             "reads as a single omnidirectional speed. Raising this therefore speeds up drive-mode " +
             "reverse without touching strafe at all, which is the reason the two are separate.\n\n" +
             "Boost scales it the same way it scales the forward cap, so boosting in reverse really " +
             "does go faster.")]
    [Min(0.01f)]
    public float reverseTopSpeed = 20f;

    // -------------------------------------------------------------------------
    // 🔄 Turning
    // -------------------------------------------------------------------------
    [Header("🔄 Turning")]
    [Tooltip("How fast the craft rotates at full turn input. Higher is twitchier and more arcade.\n\n" +
             "Moves with: Yaw Damping, which decides how cleanly that rotation stops.")]
    public float yawAccel = 8f;

    [Tooltip("The widest gap drift will open between where you POINT and where you are MOVING.\n\n" +
             "This knob decides what a drift is FOR. Weapons fire along the nose, so a held slide " +
             "is the only way to aim off your line of travel at full speed. Strafe buys you the " +
             "same freedom for a third of your top speed; drift buys it for acceleration instead.\n\n" +
             "Turning authority fades as the slide approaches this angle, so a drift settles " +
             "somewhere you can hold and shoot from instead of spinning for as long as you hold the " +
             "stick. Only the widening direction is limited: steering back toward your line always " +
             "has full authority, so a drift can never strand you sideways.\n\n" +
             "Pick it by what you want the manoeuvre to BE. Around 25 to 35 is a lean through a " +
             "corner that barely changes what you can shoot at. Around 45 to 60 is a committed " +
             "sideways slide you aim out of, and it is a different move.\n\n" +
             "Authority fades as you approach, so this is an asymptote rather than a wall: the " +
             "last few degrees arrive slowly and you will usually settle just under it. Expect to " +
             "settle around 55 to 60 percent of this number.\n\n" +
             "IT ALSO SETS YOUR CORNER RADIUS, which is not obvious from the name and is the most " +
             "useful thing about it. The fade throttles your turn rate, so a LOW value chokes " +
             "rotation and throws the corner wide, while a high one lets the nose keep coming " +
             "round. It is the only knob that tightens the corner and widens the slide at the same " +
             "time; the other two trade one against the other. Measured at Drift Lateral Damp 1.5: " +
             "60 gives a 33m corner at a 42 degree slide, 90 gives a 23m corner at 52 degrees.")]
    [Min(1f)]
    public float maxDriftAngle = 30f;

    [Tooltip("How firmly turning settles, as resistance proportional to how fast you are already " +
             "rotating.\n\n" +
             "Raise it to kill wobble and have the nose stop where you left it. Lower it for a " +
             "looser, floatier turn. Raise Yaw Accel alongside it to keep the same turn speed.")]
    [Range(0f, 20f)]
    public float yawDamping = 6f;

    [Tooltip("How much of your steering you keep as you speed up, read against a fraction of " +
             "Top Speed.\n\n" +
             "UNLIKE ACCEL CURVE, THE VALUE IS THE OUTCOME. Turn rate scales straight off this, so " +
             "0.5 at the right-hand end means exactly half the turn rate: a 180 that takes 1.7s " +
             "when slow takes 3.4s at top speed, and the corner comes out twice as wide. There is " +
             "no arithmetic between what you type and what you feel.\n\n" +
             "Flat at 1.0 is no fade, which is what a playtest called too tight: the nose comes " +
             "round just as fast at 80 as at 5, and the craft ends up sliding as much as 70 degrees " +
             "off its own nose before speed bleeds away.\n\n" +
             "Where the curve LEAVES 1.0 is the speed at which turning starts costing you. Where it " +
             "ENDS is how much you keep. Add a key between them to bend how it gets there.\n\n" +
             "Read against ACTUAL speed, not speed relative to boost, so boosting past Top Speed " +
             "holds the right-hand value rather than earning back authority.\n\n" +
             "DRIVE MODE ONLY. Drift, strafe aim and airborne turning all keep full authority.")]
    public AnimationCurve yawSpeedFade = DefaultYawFadeCurve();

    [Tooltip("How much of your remaining steering you keep at the BOOSTED top speed, on top of " +
             "Yaw Speed Fade.\n\n" +
             "Yaw Speed Fade only describes speeds up to Top Speed, so without this the whole band " +
             "boost adds is flat: measured 53 deg/s at 104 m/s, identical to 80 m/s. This keeps the " +
             "fade falling across that band instead.\n\n" +
             "It MULTIPLIES the curve's right-hand value, so 0.5 on the curve and 0.85 here means " +
             "0.425 of your steering flat out. 1.0 disables it and restores the flat band.\n\n" +
             "DRIVEN BY SPEED, NOT BY THE BUTTON, which is the thing to know. Boosting out of a " +
             "corner at half top speed costs you nothing, and letting go at 95 m/s does not hand " +
             "the steering back until you have actually slowed down. Nothing here punishes pressing " +
             "boost; it prices the speed boost buys.")]
    [Range(0f, 1f)]
    public float boostYawMultiplier = 0.85f;

    [Tooltip("How much steering you keep in the air. 0 is none, 1 is full ground response.\n\n" +
             "This is ordinary turning, separate from the air control trick system.")]
    [Range(0f, 1f)]
    public float airTurnMultiplier = 0.5f;

    // -------------------------------------------------------------------------
    // ⚡ Boost
    // -------------------------------------------------------------------------
    [Header("⚡ Boost")]
    [Tooltip("Master switch for boost.")]
    public bool enableBoost = true;

    [Tooltip("How much harder the craft accelerates while boosting.\n\n" +
             "This is the part you FEEL on engage. Boost Speed Multiplier decides where you end up; " +
             "this decides how violently you get there.")]
    [Range(1f, 3f)]
    public float boostAccelMultiplier = 1.75f;

    [Tooltip("How much higher the top speed climbs while boosting.\n\n" +
             "Scales the forward, reverse and strafe caps together.")]
    [Range(1f, 3f)]
    public float boostSpeedMultiplier = 1.5f;

    [Tooltip("How long boost takes to ramp in and out. Shorter is punchier.\n\n" +
             "Worth knowing: the CAMERA reads this same ramp. Shortening it sharpens the field of " +
             "view kick and the pull-back for free, without touching any camera setting.")]
    [Min(0.01f)]
    public float boostBlendSeconds = 0.35f;

    [Tooltip("Energy burned per second of continuous boost.\n\n" +
             "How long a boost lasts is a tug of war between this and the Energy section: this " +
             "sets the burn, Regen Rate and Regen Delay set how soon you can go again. The readout " +
             "below shows the whole cycle.\n\n" +
             "To keep boost a tempo decision rather than a permanent state, keep the recovery time " +
             "at or above the burn time.")]
    [Min(0f)]
    public float boostEnergyPerSecond = 20f;

    // -------------------------------------------------------------------------
    // 💨 Strafe Dodge
    // -------------------------------------------------------------------------
    [Header("💨 Strafe Dodge")]
    [Tooltip("Peak strength of the dodge burst. Front-loaded, fading to zero.\n\n" +
             "Tune this by the sideways speed it produces, not by the number itself. The readout " +
             "below shows that speed as a SHARE OF YOUR STRAFE CAP, which is the figure that " +
             "matters: 30 to 60% reads as a sharp sidestep, and at or above 100% it reads as a " +
             "teleport and gets hard to follow at speed.\n\n" +
             "Anything above the cap is not sustainable and bleeds back down through Strafe Lateral " +
             "Cap Strength.")]
    [Min(0f)]
    public float dodgeForce = 120f;

    [Tooltip("How long the dodge burst lasts as it fades out.\n\n" +
             "Shorter is snappier, longer is more jet-like. It also changes how far you actually " +
             "travel, so to keep the same sidestep and simply make it faster, raise Dodge Force and " +
             "shorten this together. Try 0.15 to 0.35.")]
    [Min(0.01f)]
    public float dodgeDuration = 0.25f;

    [Tooltip("Flat energy cost per dodge.\n\n" +
             "Read it against Max Energy: that ratio is how many dodges a full tank buys, and how " +
             "much boost each dodge costs you.")]
    [Min(0f)]
    public float dodgeEnergyCost = 20f;

    [Tooltip("Seconds between dodges. Prevents spam.\n\n" +
             "Whichever bites first, this or the energy cost, is what actually limits dodging.")]
    [Min(0f)]
    public float dodgeCooldown = 0.5f;

    // -------------------------------------------------------------------------
    // 🦘 Jump
    // -------------------------------------------------------------------------
    [Header("🦘 Jump")]
    [Tooltip("Master switch for jump.")]
    public bool enableJump = true;

    [Tooltip("Jump strength on a quick tap.\n\n" +
             "Sets the floor for what a hop can do. Keep the apex it produces BELOW Air Control Min " +
             "Clearance, or a tap grants attitude authority and hopping through a drift flips you. " +
             "The readout below shows both figures together.")]
    [Min(0f)]
    public float jumpImpulseMin = 4f;

    [Tooltip("Jump strength on a fully charged hold.\n\n" +
             "The readout below turns this into apex height and airtime.\n\n" +
             "Moves with: Hard Landing Min Speed, which has to stay above the speed this lands at.")]
    [Min(0f)]
    public float jumpImpulseMax = 12f;

    [Tooltip("How long you hold to reach a full charge. The charge holds at full until you release.\n\n" +
             "Longer makes the big jump a real commitment. Shorter makes full height the default.")]
    [Min(0.05f)]
    public float jumpMaxChargeTime = 2f;

    [Tooltip("Cooldown after landing before another jump is allowed.")]
    [Min(0f)]
    public float jumpGroundedLockout = 0.2f;

    [Tooltip("Energy for a TAP jump, with no charge held.\n\n" +
             "The cost ramps from this to Jump Grounded Charged Energy Cost on the SAME charge " +
             "fraction that sets the impulse, so height and price can never disagree.\n\n" +
             "Keep it cheap. A tap jump is used constantly to clear things on the ground, and " +
             "charging it for a full jump's worth of energy taxes ordinary driving.")]
    [Min(0f)]
    public float jumpGroundedEnergyCost = 10f;

    [Tooltip("Energy for a FULLY CHARGED grounded jump.\n\n" +
             "This is the one that buys ~20m of altitude and a whole trick window, so it should " +
             "read as a real commitment against a 100 pool. Set it equal to Jump Grounded Energy " +
             "Cost to go back to a flat price regardless of charge.")]
    [Min(0f)]
    public float jumpGroundedChargedEnergyCost = 25f;

    [Tooltip("Energy for an air jump.\n\n" +
             "Deliberately cheaper than a full charge: it is a hang-time extender and a mid-air " +
             "juke rather than a second full jump, and it competes directly with boost for the " +
             "same meter.")]
    [Min(0f)]
    public float jumpAirEnergyCost = 15f;

    [Tooltip("Air jump strength. Not charge-based.\n\n" +
             "Adds to whatever rise you have left rather than replacing it, so jumping again near " +
             "the top of a jump stacks into something higher than either alone.\n\n" +
             "That stacked jump produces the fastest ordinary landing in the game, and it is the " +
             "one Hard Landing Min Speed has to clear.\n\n" +
             "Read this together with Air Jump Fall Cancel. Raising this alone makes a jump at the " +
             "apex bigger without making a jump mid-fall feel any less dead.")]
    [Min(0f)]
    public float airJumpImpulse = 7f;

    [Tooltip("How much of the fall the air jump erases before it pushes. 0 is the old behaviour, " +
             "1 makes the jump deliver exactly Air Jump Impulse no matter how fast you were " +
             "dropping.\n\n" +
             "WHAT THIS FIXES: the impulse ADDS speed, so falling at 30 and jumping for 20 leaves " +
             "you still falling at 10 and the button feels dead. The same press at the top of an " +
             "arc gives the full 20. Without this, how strong the ability feels depends on WHEN " +
             "you press it, not on what it is worth.\n\n" +
             "Only velocity fighting the jump is cancelled, so this never removes rise you " +
             "already had and never breaks the apex stack described above.\n\n" +
             "TRADE: toward 1 the air jump becomes a reliable save from any fall, which may be too " +
             "forgiving for a trick game; toward 0 it stays situational and rewards timing. " +
             "Start around 0.6 and move it if recoveries feel free.")]
    [Range(0f, 1f)]
    public float airJumpFallCancel = 0.6f;

    // -------------------------------------------------------------------------
    // 🧲 Drag
    // -------------------------------------------------------------------------
    [Header("🧲 Drag")]
    [Tooltip("How hard the craft resists sliding sideways when you did not ask it to. Sets how " +
             "tightly it tracks its nose and how fast a leftover slide dies out.\n\n" +
             "Your intended strafe movement is excluded from this, so it never fights strafe input " +
             "or caps strafe speed. 0 is fully slippery sideways.")]
    [Range(0f, 50f)]
    public float lateralDamp = 2f;

    [Tooltip("How fast you bleed off speed when you let go of the throttle.\n\n" +
             "Only applies near zero throttle, so it never fights the drive. 0 means you coast " +
             "forever.")]
    [Range(0f, 50f)]
    public float forwardDamp = 2f;

    // -------------------------------------------------------------------------
    // 🌀 Drift
    // -------------------------------------------------------------------------
    [Header("🌀 Drift")]
    [Tooltip("Sideways grip while fully drifting. This is how much the craft RESISTS sliding.\n\n" +
             "KEEP IT BELOW Lateral Damp. Above it, pressing drift gives you MORE grip than normal " +
             "driving, the slide disappears and drift becomes a grip-assist button. That was tried " +
             "at 4 against a Lateral Damp of 1 and the slide collapsed from 52 to 29 degrees.\n\n" +
             "Raising it tightens the corner and narrows the slide together, so it trades one " +
             "against the other. Measured at Drift Yaw Multiplier 2.5 / Max Drift Angle 90: " +
             "1 gives a 27m corner at a 58 degree slide, 1.5 gives 23m at 52 degrees, 2 gives " +
             "20m at 47. To tighten the corner WITHOUT losing slide, raise Max Drift Angle instead.")]
    [Range(0f, 50f)]
    public float driftLateralDamp = 0f;

    [Tooltip("Forward bleed while fully drifting.\n\n" +
             "Usually close to Forward Damp, since a drift changes how you slide sideways rather " +
             "than how you coast. Lower it to carry more speed out of a drift.")]
    [Range(0f, 50f)]
    public float driftForwardDamp = 2f;

    [Tooltip("How much faster the nose rotates than your momentum while drifting.\n\n" +
             "This is what actually opens the slide angle: above 1 the nose gets ahead of where you " +
             "are going. Max Drift Angle decides where it stops.\n\n" +
             "Raising it tightens the corner AND costs speed, because a faster-rotating nose points " +
             "more of your thrust away from where you are actually going. Unlike Drift Lateral " +
             "Damp, it does not cost you slide angle, so it is the safer of the two to reach for.")]
    [Range(1f, 3f)]
    public float driftYawMultiplier = 1.4f;

    [Tooltip("How long you can hold a drift before it starts costing you speed.\n\n" +
             "Short drifts are FREE, which is what keeps drift usable as a quick aiming tool " +
             "mid-fight. Only sustained drifts pay. Set this above the length of a normal corner.")]
    [Min(0f)]
    public float driftSustainSeconds = 1.5f;

    [Tooltip("How long, after the free window above, a held drift takes to decay from your normal " +
             "top speed down to Drift Sustained Top Speed.\n\n" +
             "This is the whole cost of a long drift. Shorter is a harsher penalty.\n\n" +
             "It is ALSO the drift's total length: when this ramp finishes the drift ends itself, " +
             "even with the button still held. So a drift lasts Drift Sustain Seconds plus this, " +
             "and that total is the window your slide angle has to open in. Worth knowing if you " +
             "run a very low Drift Lateral Damp, where the slide is often still widening when the " +
             "clock runs out.")]
    [Min(0.01f)]
    public float driftBleedSeconds = 2.5f;

    [Tooltip("Where a long drift eventually leaves you, and the point at which the drift ENDS " +
             "itself and hands normal grip and steering back.\n\n" +
             "Deliberately set BELOW Min Drift Speed, so holding a drift to the end drops you under " +
             "the speed you needed to start one: you cannot simply re-enter, you have to rebuild " +
             "first. That is what makes a long drift a decision instead of a default. Raising it " +
             "above Min Drift Speed breaks that and lets a spent drift re-arm instantly.\n\n" +
             "Because corner radius is speed divided by turn rate, this also sets how tight a fully " +
             "bled drift gets: the drift TIGHTENS as it bleeds. At the shipped drift knobs, 51 m/s " +
             "gives a 26m corner and 44 m/s gives 23m, at a steady 52 degree slide throughout.")]
    [Min(0f)]
    public float driftSustainedTopSpeed = 45f;

    [Tooltip("How far you have to turn before drift engages. Keeps gentle steering from tripping " +
             "it. Try 0.3 to 0.5.")]
    [Range(0f, 1f)]
    public float driftTurnThreshold = 0.4f;

    [Tooltip("How fast you must be going to START a drift. Once drifting, speed is no longer " +
             "checked: you own the drift until you release the button.\n\n" +
             "Meant to MATCH Strafe Top Speed, so that outrunning strafe is exactly what earns you " +
             "a drift and the two modes split the speed range cleanly between them. Nothing in code " +
             "enforces that, so move this whenever you move Strafe Top Speed.")]
    [Min(0f)]
    public float minDriftSpeed = 20f;

    [Tooltip("How long drift takes to ramp in and out. Faster is snappier. Try 0.1 to 0.25.")]
    [Min(0.01f)]
    public float driftBlendSeconds = 0.15f;

    [Tooltip("A small upward kick when a drift STARTS.\n\n" +
             "The punctuation on the entry. Without it the transition is a silent crossfade of " +
             "numbers and the drift has no moment of commitment.\n\n" +
             "Keep it tiny, and understand why. A drift only survives while you still count as " +
             "grounded, so a hop past the float band (Sensor Range minus Hover Height) CANCELS the " +
             "drift it just started. It also must not reach Air Control Min Clearance, or entering " +
             "a drift would hand over attitude control.\n\n" +
             "Free side effect worth having: the hop genuinely unweights the craft, so drag and " +
             "leveling ease off for a moment. Set 0 to disable.")]
    [Min(0f)]
    public float driftHopImpulse = 5.5f;

    [Tooltip("How far the craft leans at full drift.\n\n" +
             "Exaggerate slightly. Readability matters at speed. Try 15 to 25.")]
    [Range(0f, 45f)]
    public float maxBankAngle = 20f;

    [Tooltip("A subtle lean on ordinary turns, for a constant carving look.\n\n" +
             "Stacks with drift bank, so keep it low. Try 3 to 5.")]
    [Range(0f, 15f)]
    public float passiveBankAngle = 4f;

    [Tooltip("How fast the lean catches up to its target angle. Try 6 to 10.")]
    [Range(1f, 20f)]
    public float bankLerpSpeed = 8f;

    // -------------------------------------------------------------------------
    // 🛩 Air Control
    // -------------------------------------------------------------------------
    [Header("🛩 Air Control")]
    [Tooltip("Master switch for air control. Airborne and holding drift, the left stick pitches " +
             "(up is nose down) and rolls, while the right stick still yaws.\n\n" +
             "How strong and how snappy lives in Foundation, under Air Control.")]
    public bool enableAirControl = true;

    [Tooltip("How long air control fades in and out on drift press, release, takeoff and landing.\n\n" +
             "Intended to MATCH Drift Blend Seconds, so that hopping out of a drift hands over " +
             "cleanly instead of one system arriving before the other. Nothing in code enforces " +
             "that; the inspector warns when they disagree.\n\n" +
             "Near zero, attitude control snaps on the instant you leave the ground, which is " +
             "sharper for tricks but means a bump can hand you full authority with no warning.")]
    [Min(0.01f)]
    public float airControlBlendSeconds = 0.15f;

    // -------------------------------------------------------------------------
    // 🎯 Strafe Mode
    // -------------------------------------------------------------------------
    [Header("🎯 Strafe Mode")]
    [Tooltip("Master switch for strafe / aim mode (Left Trigger).")]
    public bool enableStrafe = true;

    [Tooltip("Top speed in strafe mode, in every direction at once.\n\n" +
             "This is the trade strafe asks for: you give up speed to aim freely. It is the real " +
             "lateral ceiling, and Lateral Damp does not fight it, so raising this needs no other " +
             "compensation.\n\n" +
             "One gap worth knowing before you tune around it. SIDEWAYS, entry speed above this " +
             "bleeds off on its own. FORWARD it does not: drive stops pushing here, but the forward " +
             "over-speed bleed does not start until full Top Speed, and forward drag only applies " +
             "near zero throttle. Hold forward in strafe anywhere between the two speeds and " +
             "nothing acts on you at all, so you coast there as long as you like. Widening the gap " +
             "between them widens that band.\n\n" +
             "Moves with: Min Drift Speed, which is meant to match it.")]
    [Min(1f)]
    public float strafeTopSpeed = 20f;

    [Tooltip("How fast you reach strafe speed, in any direction.\n\n" +
             "Deliberately lower than forward accel: strafe is manoeuvring, not charging.")]
    [Min(0f)]
    public float strafeAccel = 15f;

    [Tooltip("How far the nose can tilt up or down while aiming. This is your vertical aim range.\n\n" +
             "How the nose FEELS getting there lives in Foundation, under Aim Pitch Tracking.")]
    [Range(5f, 45f)]
    public float strafePitchLimit = 15f;

    [Tooltip("Aim speed in degrees per second at full stick.\n\n" +
             "How fast you cross the range set by Strafe Pitch Limit, so the two together decide " +
             "how long a full sweep from floor to ceiling takes. Raising Strafe Pitch Limit without " +
             "raising this makes aiming feel slower even though nothing about the speed changed.\n\n" +
             "Roughly 60 to 150. Low is deliberate and easy to hold steady; high is quick to " +
             "reacquire but harder to settle on a target.")]
    [Range(10f, 300f)]
    public float strafePitchSensitivity = 90f;

    [Tooltip("How long strafe mode takes to fade in and out on the trigger.")]
    [Min(0.05f)]
    public float strafeModeBlendSeconds = 0.2f;

    [Tooltip("How fast over-cap sideways speed bleeds back down.\n\n" +
             "Only acts ABOVE Strafe Top Speed, so it never fights normal strafing. What it really " +
             "tunes is how a DODGE feels: gentle values let a dodge fired at the cap read as an " +
             "additive surge that glides back down, high values crush it in a tenth of a second. " +
             "Try 2 to 6.")]
    [Range(0f, 120f)]
    public float strafeLateralCapStrength = 3f;

    [Tooltip("A lane change while driving. Left stick sideways leans the craft off its line " +
             "without turning it, for lining up a shot or slipping past something.\n\n" +
             "THE NUMBER IS THE SIDEWAYS SPEED YOU SETTLE AT, at full stick and full speed. It is " +
             "scaled by how fast you are going, so the lean is the same few degrees off your line " +
             "at any speed and is EXACTLY NOTHING standing still. That is the point: a stationary " +
             "craft sliding sideways is strafing without the trigger, which is the one thing this " +
             "must never become.\n\n" +
             "Judge it as a lean, not a speed. Against a Top Speed of 80: 5 is about 3.6 degrees " +
             "off your line, 8 is 5.7, 12 is 8.5. Past roughly 20 it starts reading as weak " +
             "strafing and holding the trigger stops being worth it.\n\n" +
             "It builds over about a second rather than snapping across, because the same sideways " +
             "drag that limits it is what it has to push through. Lateral Damp therefore sets how " +
             "QUICKLY the lean arrives, while this sets how FAR it goes.\n\n" +
             "DRIVE MODE, ON THE GROUND, FORWARD ONLY. It fades out as the craft leaves the ground " +
             "exactly as mid-air roll fades in, so the two can never both have the stick. Strafe " +
             "and drift both keep their own behaviour untouched.")]
    [Range(0f, 25f)]
    public float driveLateralPush = 8f;

    /// <summary>
    /// The shipped shape for accelCurve: full thrust off the line, tapering to a floor
    /// near the ceiling. Only ever reached by a field that has never been serialised --
    /// an existing asset keeps whatever curve it already holds, so changing this does
    /// NOT change any craft already tuned (Measuring.md trap 21).
    ///
    /// The floor at the right-hand end is deliberate and is not decoration: a curve that
    /// reaches zero makes top speed asymptotic, so the craft approaches the number in the
    /// inspector forever and never arrives at it.
    /// </summary>
    private static AnimationCurve DefaultAccelCurve()
    {
        var c = new AnimationCurve(
            new Keyframe(0f,    1f),
            new Keyframe(0.15f, 1f),
            new Keyframe(0.40f, 0.70f),
            new Keyframe(0.70f, 0.32f),
            new Keyframe(1f,    0.12f));

        for (int i = 0; i < c.length; i++) c.SmoothTangents(i, 0f);

        // Flatten the entry segment by hand. SmoothTangents gives the second key a
        // downward slope on BOTH sides, and a Hermite segment between two equal values
        // with one sloped end bulges above them: it peaked at 1.013, so the craft
        // briefly accelerated harder than Max Forward Accel claims to be its maximum.
        var k0 = c[0]; k0.inTangent = 0f; k0.outTangent = 0f; c.MoveKey(0, k0);
        var k1 = c[1]; k1.inTangent = 0f;                     c.MoveKey(1, k1);

        return c;
    }

    /// <summary>
    /// The shipped shape for yawSpeedFade: full steering out to half of top speed, then
    /// down to half authority at top speed. Owner's spec, and it needs no solving --
    /// unlike accelCurve, the value here IS the turn rate multiplier.
    ///
    /// Only ever reached by a field that has never been serialised; an existing asset
    /// keeps whatever curve it already holds (Measuring.md trap 21).
    /// </summary>
    private static AnimationCurve DefaultYawFadeCurve()
    {
        var c = new AnimationCurve(
            new Keyframe(0f,   1f),
            new Keyframe(0.5f, 1f),
            new Keyframe(1f,   0.5f));

        for (int i = 0; i < c.length; i++) c.SmoothTangents(i, 0f);

        // Same entry flattening as DefaultAccelCurve, for the same reason: a Hermite
        // segment between two equal values with one sloped end bulges above them, which
        // would hand out MORE steering in the mid-range than the craft has when parked.
        var k0 = c[0]; k0.inTangent = 0f; k0.outTangent = 0f; c.MoveKey(0, k0);
        var k1 = c[1]; k1.inTangent = 0f;                     c.MoveKey(1, k1);

        return c;
    }
}
