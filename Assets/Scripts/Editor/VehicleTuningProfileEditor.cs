using UnityEditor;
using UnityEngine;

/// <summary>
/// VehicleTuningProfileEditor v1.1
/// -------------------------------
/// Shows what the profile's numbers actually PRODUCE, so tuning stops requiring arithmetic between
/// every tweak, and warns on the relationships that are documented as "maintain by hand" and that
/// nothing in code enforces.
///
/// v1.1 adds session baseline marking through TuningBaseline: fields moved since the baseline draw
/// bold with their old value beside them, and right-clicking one reverts it. This is a
/// ScriptableObject, so Unity's own prefab override bolding has no parent to diff against and does
/// not apply. See that file for why the baseline is a session snapshot rather than the class
/// defaults. Undo was never missing and is not added here: ApplyModifiedProperties has always
/// registered it.
///
/// Every formula here was verified against the code that consumes the value, not against the docs.
/// Three results are worth knowing before editing this file:
///
///   1. Dodge and unstick delta-v are NOT force x duration / 2. ApplyDodgeForce and
///      ApplyUnstickForce both sample the taper BEFORE decrementing the timer, so the first tick
///      runs at full magnitude and the last at one timestep's worth rather than zero. The discrete
///      sum is force x (duration + fixedDeltaTime) / 2, about 6.7% higher at the project's 100Hz.
///      Both tooltips were corrected to match in the same pass.
///   2. The hover spring is applied PER POINT and is not divided by the point count. Only the
///      gravity feedforward is divided (HoverController_Foundation.cs:542). So the stiffness the
///      chassis feels is N x liftStrength, which is where the "64 x compression" figure comes from.
///   3. The strafeAccel / lateralDamp relationship is NOT an invariant any more. The comment at
///      HoverController_Propulsion.cs:877-882 describes that bug in the past tense; ApplyDrag now
///      excludes intended strafe velocity, so drag contributes exactly zero at the cap and the cap
///      is reachable regardless of that ratio. Do not add a warning for it.
///
/// Sections are drawn by ITERATING each nested container's children rather than by naming every
/// property, which WeaponDefinitionEditor does. That editor has ~30 fields; this profile has ~90,
/// and a hand-written list would silently drop any field added to a *Tuning class later.
///
/// Section headings come free from the [Header] attributes on each group's first field, because
/// PropertyField draws a field's decorators along with it and we draw the LEAF property rather than
/// the container. Do not add explicit LabelField headings; see WeaponDefinitionEditor's doc comment
/// for what happened the last time someone did.
/// </summary>
[CustomEditor(typeof(VehicleTuningProfile))]
[CanEditMultipleObjects]
public class VehicleTuningProfileEditor : Editor
{
    /// <summary>
    /// Hull height above the hover points on HoverCar_Prototype, measured 2026-08-07.
    /// Not derivable from the asset: the profile is shared and carries no geometry.
    /// </summary>
    private const float HullHeightAboveHoverPoints = 2.22f;

    /// <summary>
    /// Nose-to-tail length of HoverCar_Prototype, from the collider union in CLAUDE.md.
    ///
    /// Every distance in the readouts is also printed in these, and the reason is a note from the
    /// owner: metres are already abstract to an American, and the arena is not built at real-world
    /// scale, so a figure in metres carries almost no intuition. The craft's own length is the one
    /// ruler on screen at all times, and it stays meaningful no matter what the world scale is.
    /// </summary>
    private const float CraftLength = 6.88f;

    private const int DefaultHoverPointCount = 4;

    private const string ShowHintsPref = "VehicleTuningProfileEditor.ShowHints";

    /// <summary>
    /// Whether the readouts print their tuning hints. The numbers are always shown; this only
    /// controls the prose under them, which is worth switching off once it has been read.
    /// </summary>
    private static bool ShowHints
    {
        get => EditorPrefs.GetBool(ShowHintsPref, true);
        set => EditorPrefs.SetBool(ShowHintsPref, value);
    }

    private static readonly string[] Sections =
        { "foundation", "propulsion", "energy", "health", "shield", "emp" };

    private int    hoverPointCount  = DefaultHoverPointCount;
    private string hoverPointSource = "assumed";
    private bool   singleTarget;

    private void OnEnable()
    {
        RefreshHoverPointCount();
        EditorApplication.hierarchyChanged += RefreshHoverPointCount;
    }

    private void OnDisable()
    {
        EditorApplication.hierarchyChanged -= RefreshHoverPointCount;
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        // Derived values and invariant checks are meaningless across a mixed selection: every
        // formula reads several fields and there is no single answer. Fields still edit normally.
        singleTarget = targets.Length == 1;

        // Passing a null target on a mixed selection deactivates the baseline rather than picking
        // one object's values and reporting them as though they described the whole selection.
        TuningBaseline.Begin(serializedObject, singleTarget ? target : null);
        TuningBaseline.DrawBar();

        if (singleTarget)
            ShowHints = EditorGUILayout.ToggleLeft(
                new GUIContent("Show tuning hints",
                    "The prose under each readout: what to change to get a given result, and what " +
                    "else moves with it. Switch it off once you know them and keep the numbers."),
                ShowHints);

        foreach (string section in Sections)
            DrawSection(section);

        if (singleTarget)
            DrawWarnings();

        serializedObject.ApplyModifiedProperties();
    }

    // -------------------------------------------------------------------------
    // Drawing
    // -------------------------------------------------------------------------

    private void DrawSection(string sectionName)
    {
        SerializedProperty section = serializedObject.FindProperty(sectionName);

        if (section == null)
            return;

        SerializedProperty it  = section.Copy();
        SerializedProperty end = section.GetEndProperty();
        bool enterChildren = true;

        while (it.NextVisible(enterChildren) && !SerializedProperty.EqualContents(it, end))
        {
            enterChildren = false;
            TuningBaseline.PropertyField(it);

            if (singleTarget)
                DrawDerivedAfter(it.propertyPath);
        }
    }

    /// <summary>
    /// Anchors each derived readout to the last field that feeds it, so the consequence appears
    /// while your hand is still on the slider. Keyed on propertyPath rather than name so the
    /// anchors stay unambiguous if a leaf name is ever reused across two sections.
    /// </summary>
    private void DrawDerivedAfter(string propertyPath)
    {
        switch (propertyPath)
        {
            case "foundation.sensorRange":          DerivedGroundedBand();  break;
            case "foundation.ceilingClearance":     DerivedCeilingDuck();   break;
            case "foundation.liftDamping":          DerivedHoverSpring();   break;
            case "foundation.airControlDamping":    DerivedAirControl();    break;
            case "foundation.airControlMinClearance": DerivedAirControlGate(); break;
            case "foundation.flipRecoveryTorque":   DerivedRighting();      break;
            case "foundation.unstickLiftDuration":  DerivedUnstick();       break;
            case "foundation.extraFallGravity":     DerivedGravity();       break;
            case "foundation.hardLandingMinSpeed":  DerivedHardLanding();   break;
            case "propulsion.dodgeDuration":        DerivedDodge();         break;
            case "propulsion.airJumpImpulse":       DerivedJump();          break;
            case "propulsion.reverseTopSpeed":      DerivedReverseBand();   break;
            case "propulsion.bankLerpSpeed":        DerivedDrift();         break;
            case "propulsion.boostEnergyPerSecond": DerivedBoost();         break;
        }
    }

    // -------------------------------------------------------------------------
    // Derived readouts
    // -------------------------------------------------------------------------

    private void DerivedGroundedBand()
    {
        float band = Get("foundation.sensorRange") - Get("foundation.hoverHeight");

        DerivedBox(
            "Float band",            Dist(band),
            "Counts as flying above", Dist(Get("foundation.supportMargin")),
            "", "Two different heights, and they do different jobs. Support Margin is where drag, "
              + "leveling and air control hand over. The float band is how high you can rise and "
              + "still DRIVE: steering, jump charge and drift entry all survive inside it, so it "
              + "also decides whether a drift lives through a bump or a hop.\n\n"
              + "Want drifts to survive rougher ground: widen the band with Sensor Range. Want a "
              + "crisper split between driving and flying: tighten Support Margin instead. "
              + "Widening the band too far means you keep driving while visibly in the air.");
    }

    private void DerivedCeilingDuck()
    {
        float hoverHeight = Get("foundation.hoverHeight");
        float clearance   = Get("foundation.ceilingClearance");
        float minDuck     = Get("foundation.minDuckHoverHeight");

        if (!GetBool("foundation.enableCeilingDuck"))
        {
            DerivedBox(
                "Ceiling duck", "OFF",
                "Anything tighter than", Dist(hoverHeight + HullHeightAboveHoverPoints),
                "", "With ducking off, that figure is a hard minimum for level geometry. Tunnels "
                  + "tighter than it do not slow you down, they PIN you: the hover springs have no "
                  + "force ceiling, so the craft is crushed against the roof and friction kills all "
                  + "drive. Either keep every ceiling above that number or turn ducking back on.");
            return;
        }

        DerivedBox(
            "Ducking starts under", Dist(hoverHeight + clearance),
            "Squats down to",       $"the gap minus {clearance:F1} m",
            "Reads as a wall under", Dist(minDuck + clearance),
            "", "Ceiling Clearance is the whole feel of this. Too low and the roof scrapes, since it "
              + $"has to cover the hull above the hover points ({HullHeightAboveHoverPoints:F2} m on "
              + "the prototype). Too high and you duck under tunnels that would have cleared fine, "
              + "giving up ride height for nothing.\n\n"
              + "Min Duck Hover Height sets where squatting gives up and the gap simply becomes "
              + "impassable. Raise it if crawling under tight geometry feels like a soft-lock; lower "
              + "it if you want more places to be squeezable.");
    }

    private void DerivedHoverSpring()
    {
        int   n        = hoverPointCount;
        float strength = Get("foundation.liftStrength");
        float damping  = Get("foundation.liftDamping");

        float spring = n * strength;

        if (spring <= 0f)
        {
            DerivedBox("Hover points", $"{n}  ({hoverPointSource})",
                       "Spring", "0: the craft makes no lift at all and will sit on the ground");
            return;
        }

        float omega = Mathf.Sqrt(spring);
        float zeta  = (n * damping) / (2f * omega);

        // Standard second-order step response. Overshoot only exists below critical damping.
        string overshoot = zeta < 1f
            ? $"{Mathf.Exp(-Mathf.PI * zeta / Mathf.Sqrt(1f - zeta * zeta)) * 100f:F0}% overshoot"
            : "no overshoot";

        string settle = zeta > 0f ? $"settles in {4f / (zeta * omega):F2} s" : "never settles";

        DerivedBox(
            "Hover points",  $"{n}  ({hoverPointSource})",
            "Bounce",        $"{overshoot}, {settle}",
            "Damping ratio", $"{zeta:F2}   (1.00 = no bounce at all)",
            "", "The ratio is what you are really tuning, not either field alone. Below about 0.3 "
              + "the craft wallows and bobs; near 1.0 it feels dead and slow to reach ride height. "
              + "Around 0.5 to 0.7 it visibly rocks when something hits it and still drives.\n\n"
              + "Less bounce, same firmness: raise Lift Damping. Firmer against hits and shoves "
              + "WITHOUT more bounce: raise Lift Strength and Lift Damping together, because moving "
              + "Lift Strength alone drops this ratio and makes the craft bouncier.");
    }

    private void DerivedAirControl()
    {
        float damping = Get("foundation.airControlDamping");

        if (damping <= 0f)
        {
            DerivedBox("Air control", "damping is 0: spins accelerate forever and never settle");
            return;
        }

        float roll  = Get("foundation.airRollTorque")  / damping;
        float pitch = Get("foundation.airPitchTorque") / damping;

        string ordering = roll > pitch
            ? "Rolls faster than it flips, which is right for a car-shaped craft."
            : "FLIPS FASTER THAN IT ROLLS. Usually backwards for a car-shaped craft: drop Air "
            + "Pitch Torque below Air Roll Torque.";

        DerivedBox(
            "Barrel roll",     PerTurn(roll),
            "Front flip",      PerTurn(pitch),
            "Stop on release", $"{3f / damping:F2} s",
            "", ordering,
            "", "Both spin speeds are their torque divided by Air Control Damping, which is the "
              + "whole relationship: the torques move one rotation at a time, damping moves both at "
              + "once and trades spin speed against how cleanly you stop.\n\n"
              + "Faster tricks: raise Air Roll Torque or Air Pitch Torque. Stop dead where you "
              + "released instead of coasting past: raise Air Control Damping, then raise BOTH "
              + "torques by the same factor to get your spin speeds back.\n\n"
              + "These are full-stick figures. Everything scales down as air control fades in, so "
              + "a rotation started the instant you leave the ground is slower than this.");
    }

    private void DerivedRighting()
    {
        float damping = Get("foundation.pitchRollDamping");

        if (damping <= 0f)
        {
            DerivedBox("Righting", "Pitch Roll Damping is 0: righting never settles to a steady rate");
            return;
        }

        float rate      = Get("foundation.flipRecoveryTorque") / damping;
        float toRelease = (180f - Get("foundation.flipRecoveryReleaseAngle")) * Mathf.Deg2Rad;
        float delay     = Get("foundation.flipRecoveryDelay");

        DerivedBox(
            "Upside down to upright", $"{toRelease / rate:F2} s",
            "You lie there first",    $"{delay:F2} s",
            "Total off the pace",     $"{delay + toRelease / rate:F2} s",
            "", "Almost all of what a flip costs you is the DELAY, not the righting. The righting "
              + "itself is near instant and always will be, because the torque has to beat Leveling "
              + "Torque Strength to work at all.\n\n"
              + "Want flipping to hurt more: raise Flip Recovery Delay. Raising Flip Recovery Torque "
              + "instead only shortens the part the player was not really feeling.");
    }

    private void DerivedUnstick()
    {
        float v    = UnstickDeltaV;
        float rise = RiseGravity;

        string pop = rise > 0f ? Dist(v * v / (2f * rise)) : "gravity is 0: it never comes down";

        DerivedBox(
            "Pop height", pop,
            "", "This should read as breaking contact, not as a jump. Compare it against the float "
              + "band above: if the pop clears that band it will cancel a drift every time it "
              + "fires, and if it ever reaches Air Control Min Clearance it hands over attitude "
              + "control for getting stuck.\n\n"
              + "Bigger pop: raise Unstick Lift Force. Same pop but snappier: raise the force and "
              + "shorten Unstick Lift Duration together, since both change the result.\n\n"
              + "If it fires when it should not, the fix is one of the two gates rather than the "
              + "strength. Shoving off ramps and bowl walls means Unstick Max Surface Angle is too "
              + "high; pulsing during a scrape at speed means Unstick Max Vertical Speed is.");
    }

    private void DerivedGravity()
    {
        float rise = RiseGravity;
        float fall = FallGravity;

        string weight = rise > 0f ? $"{fall / rise:F2}x heavier than going up" : "gravity is 0";

        DerivedBox(
            "Coming down",   weight,
            "You land at",   $"{LandingRatio:F2}x the speed you launched",
            "", "That second number is the one that catches people out. Because the fall is heavier "
              + "than the rise, a jump does NOT land at the speed it left with, so Hard Landing Min "
              + "Speed cannot be read off Jump Impulse Max. The hard landing readout below does "
              + "that arithmetic for you.\n\n"
              + "Less floaty at the top of a jump without losing height: raise Extra Fall Gravity. "
              + "Heavier everywhere, including how firmly the craft is pinned to its ride height: "
              + "raise Extra Gravity Multiplier, then recheck both jump impulses and the hard "
              + "landing threshold, because all of them are derived from it.");
    }

    private void DerivedHardLanding()
    {
        if (!GetBool("foundation.enableHardLanding"))
        {
            DerivedBox("Hard landing", "disabled");
            return;
        }

        float threshold = Get("foundation.hardLandingMinSpeed");
        float maxSpeed  = Get("foundation.hardLandingMaxSpeed");
        float plain     = PlainLandingSpeed;
        float stacked   = StackedLandingSpeed;
        float margin    = threshold - stacked;

        string verdict = margin > 0f
            ? $"clears the worst ordinary jump by {margin:F1}"
            : $"BELOW the worst ordinary jump by {-margin:F1}: normal jumps will slam";

        DerivedBox(
            "Max charge jump lands at", $"{plain:F1}",
            "Stacked air jump lands at", $"{stacked:F1}",
            "Slam threshold",            $"{threshold:F1}   ({verdict})",
            "Full severity at",          $"{maxSpeed:F1}",
            "", "The threshold has to sit above the stacked jump, or the effect fires on ordinary "
              + "play and stops meaning anything. The gap up to full severity is your expressive "
              + "range: narrow it and every hard landing looks maximal, widen it and severity "
              + "scales with how badly you misjudged the drop.\n\n"
              + "This threshold moves on its own whenever you touch either jump impulse, Extra Fall "
              + "Gravity, or Sensor Range. The last is the counter-intuitive one: a LONGER sensor "
              + "range makes hard landings less likely, because it catches the landing higher up "
              + "where you are still slower.\n\n"
              + "These landing speeds are measured at ride height and the real check fires slightly "
              + "higher, so they read a few percent pessimistic. That is the safe direction.");
    }

    private void DerivedDodge()
    {
        float strafeCap = Get("propulsion.strafeTopSpeed");
        float deltaV    = DodgeDeltaV;
        float share     = strafeCap > 0f ? deltaV / strafeCap * 100f : 0f;

        string reads = strafeCap <= 0f ? "strafe cap is 0"
                     : share < 30f     ? "reads as a nudge"
                     : share <= 60f    ? "reads as a sharp sidestep"
                     : share < 100f    ? "reads as a strong lunge"
                                       : "reads as a TELEPORT, hard to follow at speed";

        float energy   = Get("propulsion.dodgeEnergyCost");
        float maxPool  = Get("energy.maxEnergy");
        float cooldown = Get("propulsion.dodgeCooldown");
        float regen    = Get("energy.regenRate");

        // Two independent limits on repeat dodging: the cooldown, and how fast regen can pay for
        // the next one. Whichever wants a longer gap is the one the player actually feels.
        float energyGap = regen > 0f ? energy / regen : Mathf.Infinity;

        string limiter = energy <= 0f
            ? $"the {cooldown:F2} s cooldown (dodges are free)"
            : energyGap > cooldown
                ? $"ENERGY, one every {energyGap:F2} s sustained"
                : $"the {cooldown:F2} s cooldown";

        DerivedBox(
            "Sidestep speed",  strafeCap > 0f ? $"{share:F0}% of strafe cap   ({reads})" : reads,
            "Dodges per tank", energy > 0f ? $"{Mathf.Floor(maxPool / energy):F0}" : "unlimited",
            "Repeat rate set by", limiter,
            "", "Read the sidestep as a SHARE of the strafe cap, never as a raw number. Anything "
              + "over 100% is not sustainable: it is an additive burst that bleeds back down "
              + "through Strafe Lateral Cap Strength, which is what makes a dodge read as a surge "
              + "rather than a speed change.\n\n"
              + "Same distance but snappier: raise Dodge Force and shorten Dodge Duration together. "
              + "Bigger sidestep: raise Dodge Force alone. Make dodging a resource decision rather "
              + "than a reflex: raise Dodge Energy Cost, since that competes directly with boost.");
    }

    private void DerivedJump()
    {
        if (!GetBool("propulsion.enableJump"))
        {
            DerivedBox("Jump", "disabled");
            return;
        }

        float rise = RiseGravity;
        float fall = FallGravity;

        if (rise <= 0f || fall <= 0f)
        {
            DerivedBox("Jump", "gravity is 0: the craft never comes back down");
            return;
        }

        float tapLaunch     = Get("propulsion.jumpImpulseMin");
        float plainLaunch   = Get("propulsion.jumpImpulseMax");
        float stackedLaunch = StackedLaunchSpeed;

        float tapApex     = tapLaunch * tapLaunch / (2f * rise);
        float plainApex   = plainLaunch * plainLaunch / (2f * rise);
        float stackedApex = stackedLaunch * stackedLaunch / (2f * rise);

        DerivedBox(
            "Tap jump",         $"{Dist(tapApex)},  {tapLaunch / rise + tapLaunch * LandingRatio / fall:F2} s airborne",
            "Max charge",       $"{Dist(plainApex)},  {plainLaunch / rise + PlainLandingSpeed / fall:F2} s airborne",
            "Stacked air jump", $"{Dist(stackedApex)},  {stackedLaunch / rise + StackedLandingSpeed / fall:F2} s airborne",
            "Charge buys you",  plainApex > 0f ? $"{plainApex / Mathf.Max(0.01f, tapApex):F1}x the height of a tap" : "nothing",
            "", "An air jump ADDS to whatever rise you have left rather than replacing it, which is "
              + "why the stacked figure is not simply the two heights added together. Time it near "
              + "the top of a jump and you get most of both.\n\n"
              + "More hangtime at the same height: lower Extra Fall Gravity. More height at the "
              + "same hangtime: raise Jump Impulse Max and Extra Fall Gravity together. Make the "
              + "charge feel worth holding: widen the gap between the two impulses.\n\n"
              + "Keep the TAP apex below Air Control Min Clearance, or hopping while holding drift "
              + "hands over attitude control and plants you.");
    }

    /// <summary>
    /// Replaces the old DerivedStrafeBand, which printed a "forward coast band" from the strafe
    /// cap up to Top Speed. That band was real once and is not any more: Propulsion v1.2 pointed
    /// ApplyOverSpeedBleed at the same BlendedTopSpeed that ApplyDrive clamps to, so the bleed now
    /// engages at the strafe-blended cap and there is no speed at which nothing acts. The readout
    /// outlived the defect it described and was reporting a fixed bug as a live one.
    ///
    /// Reverse took its slot because reverse is the axis that is now invisible. Its cap moves with
    /// BOTH boost and strafe blend and appears nowhere in the inspector, which is the same
    /// invisibility that motivated the Propulsion v1.4 in-play readout.
    /// </summary>
    /// <summary>
    /// Translates the air control clearance floor into the thing a designer actually
    /// decides: how much of a jump charge it costs to earn attitude authority.
    ///
    /// The whole point of the floor is that a tap jump must not reach it, so the tap
    /// apex is printed next to the threshold rather than left to be worked out. Both
    /// failure directions are called out below, because either one silently turns the
    /// feature off in a way that looks like it is working.
    /// </summary>
    private void DerivedAirControlGate()
    {
        if (!GetBool("propulsion.enableAirControl"))
        {
            DerivedBox("Air control", "disabled");
            return;
        }

        float rise = RiseGravity;

        if (rise <= 0f)
        {
            DerivedBox("Air control gate", "gravity is 0: apex is undefined");
            return;
        }

        float clearance = Get("foundation.airControlMinClearance");
        float minImp    = Get("propulsion.jumpImpulseMin");
        float maxImp    = Get("propulsion.jumpImpulseMax");

        float tapApex = minImp * minImp / (2f * rise);
        float maxApex = maxImp * maxImp / (2f * rise);
        float reqImp  = Mathf.Sqrt(2f * rise * clearance);

        string cost;
        if (reqImp <= minImp)
            cost = "an uncharged TAP already clears it, so tricks are always armed";
        else if (reqImp > maxImp)
            cost = "NO jump can reach it: only ledges and ramps will ever arm tricks";
        else
        {
            float frac = (reqImp - minImp) / Mathf.Max(0.01f, maxImp - minImp);
            cost = $"{frac * 100f:F0}% charge   ({frac * Get("propulsion.jumpMaxChargeTime"):F2} s hold)";
        }

        DerivedBox(
            "Room needed below", Dist(clearance),
            "Costs you",         cost,
            "Tap jump reaches",  Dist(tapApex),
            "Full charge reaches", Dist(maxApex),
            "", "This is the price of admission for tricks, and it wants to sit BETWEEN those two "
              + "apexes. Below the tap and every hop arms attitude control, which means holding "
              + "drift through a bump reads your throttle as nose-down and plants you. Above the "
              + "full charge and jumping can never start a trick at all.\n\n"
              + "Make tricks easier to start: lower this, or raise Jump Impulse Min. Make them a "
              + "bigger commitment: raise this toward the full-charge apex.\n\n"
              + "It measures room BELOW you rather than height gained, so a hop on flat ground arms "
              + "nothing while the same hop off a ledge arms as the ground falls away. It is a "
              + "floor on WHEN you get authority, not on what you can do with it: past that point "
              + "your airtime still decides whether a rotation finishes.");
    }

    private void DerivedReverseBand()
    {
        float driveCap  = Get("propulsion.reverseTopSpeed");
        float strafeCap = Get("propulsion.strafeTopSpeed");
        float speedMult = GetBool("propulsion.enableBoost")
            ? Get("propulsion.boostSpeedMultiplier")
            : 1f;

        float topSpeed = Get("propulsion.topSpeed");
        float brake    = Get("propulsion.maxReverseAccel");

        DerivedBox(
            "Reverse in drive",  $"{driveCap:F1}   (boosted {driveCap * speedMult:F1})",
            "Reverse in strafe", $"{strafeCap:F1}   (boosted {strafeCap * speedMult:F1})",
            "Full stop from top", brake > 0f ? $"{topSpeed / brake:F2} s" : "never: brake is 0",
            "", "Reverse blends toward Strafe Top Speed as you enter strafe, so at full strafe your "
              + "forward, reverse and sideways ceilings are all the same number and the craft reads "
              + "as one omnidirectional speed. Raising this field therefore speeds up drive-mode "
              + "reverse and leaves strafe untouched.\n\n"
              + "The stopping figure is the one worth watching: Max Reverse Accel doubles as the "
              + "BRAKE, so set it by how fast you want to stop and treat reverse speed as the side "
              + "effect. It is not strafe-blended, so braking feels the same in both modes.");
    }

    /// <summary>
    /// The drift block, and the only readout here that SOLVES for something rather than
    /// evaluating a formula.
    ///
    /// The shoulder angle is not a setting anywhere: it is the equilibrium between yaw opening the
    /// slide and lateral grip closing it, with maxDriftAngle capping it by fading yaw authority as
    /// the slide widens. Which of those two actually stops you is the single most useful thing to
    /// know when tuning a drift, and it is not visible in any field.
    /// </summary>
    private void DerivedDrift()
    {
        float maxAngle = Get("propulsion.maxDriftAngle");
        float yawDamp  = Get("propulsion.yawDamping");
        float latDamp  = Get("propulsion.driftLateralDamp");
        float openRate = yawDamp > 0f
            ? Get("propulsion.yawAccel") * Get("propulsion.driftYawMultiplier") / yawDamp
            : 0f;

        float settled = SettledDriftAngle(openRate, latDamp, maxAngle);

        // Entry gate, as a share of the speed range rather than a bare number.
        float topSpeed  = Get("propulsion.topSpeed");
        float minDrift  = Get("propulsion.minDriftSpeed");
        string gate = topSpeed > 0f
            ? $"{minDrift:F0}   (the top {Mathf.Max(0f, (topSpeed - minDrift) / topSpeed) * 100f:F0}% of your speed range)"
            : $"{minDrift:F0}";

        // The entry hop has to stay inside the float band or it cancels the drift it just started.
        float rise = RiseGravity;
        float hopV = Get("propulsion.driftHopImpulse");
        float band = Get("foundation.sensorRange") - Get("foundation.hoverHeight");
        float hop  = rise > 0f ? hopV * hopV / (2f * rise) : 0f;

        string hopLine = band > 0f
            ? (hop >= band
                ? $"{hop:F1} m   OVERSHOOTS the float band and cancels the drift"
                : $"{hop:F1} m   ({hop / band * 100f:F0}% of the float band, safe)")
            : $"{hop:F1} m";

        float normalYaw = yawDamp > 0f ? Get("propulsion.yawAccel") / yawDamp : 0f;

        string grip = Get("propulsion.lateralDamp") > 0f
            ? $"{latDamp / Get("propulsion.lateralDamp") * 100f:F0}% of normal"
            : "normal grip is 0, so drift changes nothing sideways";

        string whatStops = maxAngle <= 0f            ? "No ceiling set: the nose runs as long as you hold the stick."
                         : settled >= maxAngle * 0.9f ? "The CEILING is setting your angle. Turn authority runs out before "
                                                      + "grip pulls you back, so Max Drift Angle is the knob that matters."
                                                      : "GRIP is setting your angle. You settle well short of the ceiling, so "
                                                      + "Drift Lateral Damp is the knob that matters and raising Max Drift "
                                                      + "Angle will do nothing.";

        DerivedBox(
            "Drift starts above",  gate,
            "Settles around",      $"{settled:F0} deg   of a {maxAngle:F0} deg ceiling",
            "Sideways grip kept",  grip,
            "Nose swing at entry", $"{openRate * Mathf.Rad2Deg:F0} deg/s   (vs {normalYaw * Mathf.Rad2Deg:F0} normal)",
            "Entry hop",           hopLine,
            "", whatStops,
            "", "THE ANGLE is an equilibrium, not a setting. Drift Yaw Multiplier opens it, Drift "
              + "Lateral Damp closes it, and Max Drift Angle caps it by fading your turn authority "
              + "to nothing as the slide widens. Only the WIDENING direction is limited, so steering "
              + "back toward your line always has full authority and an overcooked entry can always "
              + "be driven out of.\n\n"
              + "THE PATH is mostly your throttle. Drive pushes along the NOSE, not along your line "
              + "of travel, so holding the stick forward is what bends the slide toward where you "
              + "are pointing. Drift Lateral Damp does the same thing passively and constantly.\n\n"
              + "That coupling is the trap: Drift Lateral Damp sets how straight the slide holds AND "
              + "how wide it settles, because it is one force doing both. Wider angle and a truer "
              + "line: raise Max Drift Angle, lower Drift Lateral Damp. Tighter, more controllable "
              + "arc: raise Drift Lateral Damp and let the angle come down with it.\n\n"
              + "Drift Forward Damp runs at FULL strength through a drift no matter your throttle, "
              + "unlike normal driving where it switches off above a light touch. So a drift adds "
              + "drag while you are on the gas and removes most of it while coasting.\n\n"
              + "Max Bank Angle, Passive Bank Angle and Bank Lerp Speed are COSMETIC. They rotate "
              + "the mesh and touch no physics, so exaggerate them freely for readability at speed.");
    }

    /// <summary>
    /// Where the slide settles: walk out from centre until the closing term catches the opening
    /// term. Yaw authority fades linearly to zero at maxDriftAngle; lateral grip pulls the velocity
    /// back under the nose at a rate proportional to sin(2*angle), which peaks at 45 degrees.
    ///
    /// Throttle is deliberately excluded. Drive pushes along the nose, which closes the angle
    /// further, so this reports the WIDEST the slide gets and a drift held under power sits inside
    /// it. That is the useful direction to be wrong in.
    /// </summary>
    private static float SettledDriftAngle(float openRate, float lateralDamp, float maxAngle)
    {
        if (maxAngle <= 0f || openRate <= 0f) return 0f;

        const int steps = 360;

        for (int i = 1; i <= steps; i++)
        {
            float deg    = maxAngle * i / steps;
            float budget = 1f - deg / maxAngle;
            float close  = lateralDamp * Mathf.Sin(2f * deg * Mathf.Deg2Rad) * 0.5f;

            if (openRate * budget <= close)
                return deg;
        }

        return maxAngle;
    }

    private void DerivedBoost()
    {
        if (!GetBool("propulsion.enableBoost"))
        {
            DerivedBox("Boost", "disabled");
            return;
        }

        float speedMult = Get("propulsion.boostSpeedMultiplier");
        float perSecond = Get("propulsion.boostEnergyPerSecond");
        float maxEnergy = Get("energy.maxEnergy");
        float regen     = Get("energy.regenRate");
        float regenWait = Get("energy.regenDelay");

        float burn    = perSecond > 0f ? maxEnergy / perSecond : Mathf.Infinity;
        float refill  = regen     > 0f ? maxEnergy / regen     : Mathf.Infinity;
        float recover = regenWait + refill;

        string duration = perSecond > 0f ? $"{burn:F1} s from full" : "unlimited: costs no energy";
        string back     = regen     > 0f ? $"{recover:F1} s to full again" : "never refills";

        string verdict = perSecond <= 0f  ? "Boost is free, so it is a permanent state rather than a choice."
                       : regen     <= 0f  ? "Energy never comes back, so a tank is all you get per life."
                       : recover >= burn  ? "Recovery costs more than the burn, so boost stays a decision."
                                          : "Recovery is FASTER than the burn, so boost is close to always available.";

        DerivedBox(
            "Boosted top speed",  $"{Get("propulsion.topSpeed") * speedMult:F1}   (x{speedMult:F2})",
            "Boosted strafe cap", $"{Get("propulsion.strafeTopSpeed") * speedMult:F1}",
            "Hold it for",        duration,
            "Then wait",          back,
            "", verdict,
            "", "Boost length is a tug of war across two sections: Boost Energy Per Second sets the "
              + "burn, and Energy's Regen Rate and Regen Delay set how soon you can go again. To "
              + "keep boost a tempo decision rather than a default state, keep the recovery at or "
              + "above the burn.\n\n"
              + "Punchier engage without changing any of that: shorten Boost Blend Seconds. The "
              + "camera reads the same ramp, so its kick and pull-back sharpen with it for free.");
    }

    // -------------------------------------------------------------------------
    // Warnings
    // -------------------------------------------------------------------------

    /// <summary>
    /// The relationships the tooltips describe as "maintain by hand". Each one has been out of step
    /// at some point, which is exactly why they are checked here rather than trusted.
    /// </summary>
    private void DrawWarnings()
    {
        bool any = false;

        // 1. Outpacing strafe is what is meant to earn the drift, so the two speeds define one
        //    boundary between the modes. Nothing in code ties them together.
        float minDrift  = Get("propulsion.minDriftSpeed");
        float strafeCap = Get("propulsion.strafeTopSpeed");

        if (!Mathf.Approximately(minDrift, strafeCap))
            Warn(ref any, $"minDriftSpeed ({minDrift:F1}) does not match strafeTopSpeed ({strafeCap:F1}). "
                        + "They are meant to be equal so that outpacing strafe is exactly what earns the "
                        + "drift and the two modes divide the speed range cleanly.", MessageType.Warning);

        // 1b. The air control floor exists to keep a hop out of trick territory, because
        //     drift and air control share a button and the left stick changes meaning the
        //     moment the craft leaves the ground. If an uncharged tap already clears the
        //     floor the feature is silently off, and it fails looking exactly like it works.
        if (GetBool("propulsion.enableAirControl") && RiseGravity > 0f)
        {
            float clearanceNeeded = Get("foundation.airControlMinClearance");
            float tapApexNow      = Get("propulsion.jumpImpulseMin") * Get("propulsion.jumpImpulseMin")
                                  / (2f * RiseGravity);
            float maxApexNow      = Get("propulsion.jumpImpulseMax") * Get("propulsion.jumpImpulseMax")
                                  / (2f * RiseGravity);

            if (tapApexNow >= clearanceNeeded)
                Warn(ref any, $"An uncharged tap jump apexes at {tapApexNow:F1} m, at or above "
                            + $"airControlMinClearance ({clearanceNeeded:F1} m), so a tap already grants "
                            + "air control. Holding drift through a hop then reads held throttle as full "
                            + "nose-down. Raise the clearance above the tap apex or lower jumpImpulseMin.",
                            MessageType.Warning);
            else if (maxApexNow < clearanceNeeded)
                Warn(ref any, $"Even a full-charge jump apexes at only {maxApexNow:F1} m, below "
                            + $"airControlMinClearance ({clearanceNeeded:F1} m), so no jump can ever grant "
                            + "air control. Only ledges and ramps will.", MessageType.Warning);
        }

        // 1c. Drift and air control share a button, so the two blends are a handover rather than
        //     two independent fades. If they disagree, one system arrives before the other and a
        //     hop out of a drift lands in a state that is neither.
        if (GetBool("propulsion.enableAirControl"))
        {
            float airBlend   = Get("propulsion.airControlBlendSeconds");
            float driftBlend = Get("propulsion.driftBlendSeconds");

            if (!Mathf.Approximately(airBlend, driftBlend))
                Warn(ref any, $"Air Control Blend Seconds ({airBlend:F2}) does not match Drift Blend "
                            + $"Seconds ({driftBlend:F2}). They are meant to be equal so that hopping "
                            + "out of a drift hands attitude control over cleanly instead of one fading "
                            + "while the other is still arriving. Near zero, air control also snaps on "
                            + "the instant you leave the ground, so a bump can grant full authority "
                            + "with no warning.", MessageType.Warning);
        }

        // 2. Depends on both jump impulses, extraFallGravity AND sensorRange. The sensorRange
        //    coupling is the non-obvious one: a wider sensor range fires the landing check higher
        //    up, where the craft is still slower, so it makes hard landings LESS likely.
        if (GetBool("foundation.enableHardLanding"))
        {
            float threshold = Get("foundation.hardLandingMinSpeed");
            float stacked   = StackedLandingSpeed;

            if (threshold <= stacked)
                Warn(ref any, $"hardLandingMinSpeed ({threshold:F1}) is at or below the computed stacked "
                            + $"jump landing speed ({stacked:F1}), so ordinary jumps will trigger hard "
                            + "landings. This figure moves with jumpImpulseMax, airJumpImpulse, "
                            + "extraFallGravity and sensorRange; recheck it after any of the four.",
                            MessageType.Warning);
        }

        // 3. sensorRange defines GROUNDED, not just sensing.
        float band = Get("foundation.sensorRange") - Get("foundation.hoverHeight");

        if (band <= 0f)
            Warn(ref any, $"sensorRange is at or below hoverHeight, leaving no grounded band ({band:F2} m). "
                        + "The craft loses lift before it stops counting as grounded.", MessageType.Error);
        else if (band < 2f || band > 3f)
            Warn(ref any, $"sensorRange - hoverHeight is {band:F2} m; roughly 2 to 3 is the working range. "
                        + "Below that, leaving the ground stops reading cleanly. Above it, the craft counts "
                        + "as grounded while visibly airborne.", MessageType.Warning);

        // 4. The detectable form of "retune liftDamping whenever liftStrength moves": the ratio
        //    depends on both, so only the result is worth checking.
        float spring = hoverPointCount * Get("foundation.liftStrength");

        if (spring > 0f)
        {
            float zeta = (hoverPointCount * Get("foundation.liftDamping")) / (2f * Mathf.Sqrt(spring));

            if (zeta < 0.3f)
                Warn(ref any, $"Hover damping ratio is {zeta:F2} at {hoverPointCount} hover points. Below "
                            + "about 0.3 the chassis overshoots hard and bobs. The ratio depends on both "
                            + "liftStrength and liftDamping, so retune liftDamping whenever liftStrength moves.",
                            MessageType.Warning);
            else if (zeta > 1.2f)
                Warn(ref any, $"Hover damping ratio is {zeta:F2} at {hoverPointCount} hover points. Well "
                            + "above critical, so the chassis will feel dead and slow to reach ride height.",
                            MessageType.Warning);
        }

        // 5. Geometry the asset cannot know about. See the constant's comment.
        float clearance = Get("foundation.ceilingClearance");

        if (GetBool("foundation.enableCeilingDuck") && clearance < HullHeightAboveHoverPoints)
            Warn(ref any, $"ceilingClearance ({clearance:F2}) is below the hull height above the hover "
                        + $"points ({HullHeightAboveHoverPoints:F2} m on HoverCar_Prototype), so the roof "
                        + "will scrape even while ducking.", MessageType.Warning);

        // 6. Advisory: HandleRecovery already Mathf.Min-clamps the pair, so the code cannot break.
        //    What returns is the ~78 degree hover-supported equilibrium stall.
        float releaseAngle = Get("foundation.flipRecoveryReleaseAngle");
        float armAngle     = Get("foundation.flipRecoveryAngleThreshold");

        if (armAngle > 0f && releaseAngle > armAngle * 0.7f)
            Warn(ref any, $"flipRecoveryReleaseAngle ({releaseAngle:F0}) is close to "
                        + $"flipRecoveryAngleThreshold ({armAngle:F0}). Righting must hand over well clear "
                        + "of the arm threshold, or the craft parks in the hover-supported equilibrium at "
                        + "about 78 degrees, drifts back over, re-arms and repeats.", MessageType.Warning);

        // 7. Not a fault, but every derived figure above changes meaning.
        if (Get("foundation.extraGravityMultiplier") <= 0f)
            Warn(ref any, "extraGravityMultiplier is 0, so the craft runs at plain Earth gravity. Every jump, "
                        + "hangtime and hard-landing figure in this profile is derived from it, and it also "
                        + "scales the hover spring's gravity feedforward. Recheck hardLandingMinSpeed, both "
                        + "jump impulses and extraFallGravity.", MessageType.Info);

        // 8. StrafeTopSpeedScaled divides by topSpeed, so the two are directly related.
        float topSpeed = Get("propulsion.topSpeed");

        if (strafeCap > topSpeed)
            Warn(ref any, $"strafeTopSpeed ({strafeCap:F1}) exceeds topSpeed ({topSpeed:F1}). The strafe cap "
                        + "is scaled against topSpeed, so strafing would outrun driving.", MessageType.Warning);

        // 9. Straight from the dodgeForce tooltip's own guidance.
        float dodge = DodgeDeltaV;

        if (strafeCap > 0f && dodge >= strafeCap)
            Warn(ref any, $"A dodge is worth {dodge / strafeCap * 100f:F0}% of the strafe cap. At or above "
                        + "100% it stops reading as a sidestep and starts reading as a teleport, which is "
                        + "hard to follow at speed and hard to shoot at. Lower Dodge Force, or raise Strafe "
                        + "Top Speed so the burst is a smaller share of it. 30 to 60% is the sharp-sidestep "
                        + "range.", MessageType.Warning);
    }

    // -------------------------------------------------------------------------
    // Shared derivations
    // -------------------------------------------------------------------------

    private float RiseGravity   => Physics.gravity.magnitude * (1f + Get("foundation.extraGravityMultiplier"));
    private float FallGravity   => RiseGravity + Get("foundation.extraFallGravity");
    private float LandingRatio  => Mathf.Sqrt(FallGravity / Mathf.Max(0.0001f, RiseGravity));

    private float PlainLandingSpeed   => Get("propulsion.jumpImpulseMax") * LandingRatio;
    private float StackedLandingSpeed => StackedLaunchSpeed * LandingRatio;

    /// <summary>
    /// Both impulses are ForceMode.VelocityChange along world up, and the air jump adds to whatever
    /// rise velocity remains, so the stacked launch is the quadrature sum rather than the plain sum.
    /// </summary>
    private float StackedLaunchSpeed
    {
        get
        {
            float grounded = Get("propulsion.jumpImpulseMax");
            float air      = Get("propulsion.airJumpImpulse");
            return Mathf.Sqrt(grounded * grounded + air * air);
        }
    }

    private float DodgeDeltaV =>
        Get("propulsion.dodgeForce") * (Get("propulsion.dodgeDuration") + Time.fixedDeltaTime) * 0.5f;

    private float UnstickDeltaV =>
        Get("foundation.unstickLiftForce") * (Get("foundation.unstickLiftDuration") + Time.fixedDeltaTime) * 0.5f;

    // -------------------------------------------------------------------------
    // Plumbing
    // -------------------------------------------------------------------------

    private float Get(string path)
    {
        SerializedProperty p = serializedObject.FindProperty(path);
        return p != null ? p.floatValue : 0f;
    }

    private bool GetBool(string path)
    {
        SerializedProperty p = serializedObject.FindProperty(path);
        return p != null && p.boolValue;
    }

    /// <summary>
    /// The hover point count is a Transform[] on HoverController_Foundation, not on this asset, so
    /// it cannot be read from the profile at all. Resolve it from a vehicle in the open scene that
    /// actually uses this profile, and always label the source so an assumed value never passes for
    /// a measured one.
    /// </summary>
    private void RefreshHoverPointCount()
    {
        hoverPointCount  = DefaultHoverPointCount;
        hoverPointSource = "assumed; no vehicle using this profile is open";

        if (target is not VehicleTuningProfile profile)
            return;

        var foundations = UnityEngine.Object.FindObjectsByType<HoverController_Foundation>(
            FindObjectsInactive.Include, FindObjectsSortMode.None);

        foreach (HoverController_Foundation foundation in foundations)
        {
            // profile and hoverPoints are both private [SerializeField], so SerializedObject is how
            // an editor reaches them without adding a runtime accessor to a module CLAUDE.md marks
            // "do not modify without explicit justification".
            var so = new SerializedObject(foundation);

            if (so.FindProperty("profile")?.objectReferenceValue != profile)
                continue;

            SerializedProperty points = so.FindProperty("hoverPoints");

            if (points == null || points.arraySize == 0)
                continue;

            hoverPointCount  = points.arraySize;
            hoverPointSource = foundation.gameObject.name;
            return;
        }
    }

    /// <summary>
    /// Dimmed label/value block. Arguments are label, value, label, value, and so on.
    ///
    /// An EMPTY label marks its value as a tuning HINT rather than a readout: it draws as a
    /// wrapped paragraph and disappears when hints are switched off. Hints answer "to get X,
    /// change Y and Z"; anything that is merely a caveat about what the numbers mean belongs in
    /// the labels themselves or in the field's tooltip, not here.
    /// </summary>
    private static void DerivedBox(params string[] labelValuePairs)
    {
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);

        using (new EditorGUI.DisabledScope(true))
        {
            for (int i = 0; i + 1 < labelValuePairs.Length; i += 2)
            {
                bool isHint = string.IsNullOrEmpty(labelValuePairs[i]);

                if (isHint && !ShowHints)
                    continue;

                if (isHint)
                    EditorGUILayout.LabelField(labelValuePairs[i + 1], EditorStyles.wordWrappedMiniLabel);
                else
                    EditorGUILayout.LabelField(labelValuePairs[i], labelValuePairs[i + 1]);
            }
        }

        EditorGUILayout.EndVertical();
    }

    /// <summary>
    /// A distance, in metres and in craft lengths. See the CraftLength constant for why the second
    /// figure is there: it is the one that survives the arena not being at real-world scale.
    /// </summary>
    private static string Dist(float metres)
    {
        float lengths = metres / CraftLength;

        // Below about a quarter of the craft, "0.1 craft lengths" rounds away the only digit that
        // carried meaning. A percentage keeps the comparison legible all the way down.
        string scaled = lengths >= 0.25f
            ? $"{lengths:F1} craft lengths"
            : $"{lengths * 100f:F0}% of a craft length";

        return $"{metres:F1} m   ({scaled})";
    }

    /// <summary>Seconds for one full 360 degree rotation at a steady rate given in rad/s.</summary>
    private static string PerTurn(float radiansPerSecond) =>
        radiansPerSecond > 0.0001f
            ? $"{2f * Mathf.PI / radiansPerSecond:F2} s per full turn"
            : "never completes a turn";

    private static void Warn(ref bool any, string message, MessageType severity)
    {
        if (!any)
        {
            EditorGUILayout.Space(8);
            any = true;
        }
        EditorGUILayout.HelpBox(message, severity);
    }
}
