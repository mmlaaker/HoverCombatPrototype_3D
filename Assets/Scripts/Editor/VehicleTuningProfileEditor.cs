using UnityEditor;
using UnityEngine;

/// <summary>
/// VehicleTuningProfileEditor v1.0
/// -------------------------------
/// Shows what the profile's numbers actually PRODUCE, so tuning stops requiring arithmetic between
/// every tweak, and warns on the relationships that are documented as "maintain by hand" and that
/// nothing in code enforces.
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

    private const int DefaultHoverPointCount = 4;

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
            EditorGUILayout.PropertyField(it, true);

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
            "Grounded band", $"{band:F2} m above ride height",
            "", "Springs make no lift here, but drag and leveling still apply and air control stays off.");
    }

    private void DerivedCeilingDuck()
    {
        float hoverHeight = Get("foundation.hoverHeight");
        float clearance   = Get("foundation.ceilingClearance");
        float minDuck     = Get("foundation.minDuckHoverHeight");

        if (!GetBool("foundation.enableCeilingDuck"))
        {
            DerivedBox("Ceiling duck", "disabled: geometry below "
                                     + $"{hoverHeight + HullHeightAboveHoverPoints:F2} m of clearance pins the chassis");
            return;
        }

        DerivedBox(
            "Duck engages below", $"{hoverHeight + clearance:F2} m floor-to-ceiling gap",
            "Ride height then", $"gap - {clearance:F2} m",
            "Pins again below", $"{minDuck + clearance:F2} m gap");
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
                       "Spring", "0: no lift at any compression");
            return;
        }

        float omega = Mathf.Sqrt(spring);
        float zeta  = (n * damping) / (2f * omega);

        // Standard second-order step response. Overshoot only exists below critical damping.
        string overshoot = zeta < 1f
            ? $"{Mathf.Exp(-Mathf.PI * zeta / Mathf.Sqrt(1f - zeta * zeta)) * 100f:F0}%"
            : "none";

        string settle = zeta > 0f ? $"{4f / (zeta * omega):F2} s" : "never";

        DerivedBox(
            "Hover points", $"{n}  ({hoverPointSource})",
            "Spring", $"{spring:F1} m/s² per metre of compression",
            "Damping ratio", $"{zeta:F2}   overshoot {overshoot}",
            "Settle to 2%", settle);
    }

    private void DerivedAirControl()
    {
        float damping = Get("foundation.airControlDamping");

        if (damping <= 0f)
        {
            DerivedBox("Air control", "damping is 0: rotation never reaches a steady rate");
            return;
        }

        float roll  = Get("foundation.airRollTorque")  / damping;
        float pitch = Get("foundation.airPitchTorque") / damping;

        DerivedBox(
            "Steady roll rate",  $"{roll:F2} rad/s   {roll * Mathf.Rad2Deg:F0} deg/s",
            "Steady pitch rate", $"{pitch:F2} rad/s   {pitch * Mathf.Rad2Deg:F0} deg/s",
            "Stop on release",   $"~{3f / damping:F2} s to 95%",
            "", "Full stick at full air-control weight. Both rates fall off with the blend.");
    }

    private void DerivedRighting()
    {
        float damping = Get("foundation.pitchRollDamping");

        if (damping <= 0f)
        {
            DerivedBox("Righting rate", "pitchRollDamping is 0: righting never reaches a steady rate");
            return;
        }

        float rate = Get("foundation.flipRecoveryTorque") / damping;

        DerivedBox(
            "Righting rate", $"{rate:F2} rad/s   {rate * Mathf.Rad2Deg:F0} deg/s",
            "", "Constant torque, not a spring, so it does not ease off as the craft comes up. "
              + "Flip Recovery Delay is what the player feels, not this.");
    }

    private void DerivedUnstick()
    {
        DerivedBox(
            "Unstick delta-v", $"{UnstickDeltaV:F2} m/s",
            "", $"force x (duration + {Time.fixedDeltaTime:F3} timestep) / 2. The taper is sampled "
              + "before the timer decrements, so the last tick is never zero.");
    }

    private void DerivedGravity()
    {
        DerivedBox(
            "Rise gravity", $"{RiseGravity:F2} m/s²",
            "Fall gravity", $"{FallGravity:F2} m/s²  (airborne and descending)",
            "Landing ratio", $"x{LandingRatio:F3} on launch speed",
            "", "Extra Gravity Multiplier also feeds the hover spring's gravity feedforward, so "
              + "moving it changes how firmly the craft is pinned, not its ride height.");
    }

    private void DerivedHardLanding()
    {
        if (!GetBool("foundation.enableHardLanding"))
        {
            DerivedBox("Hard landing", "disabled");
            return;
        }

        float threshold = Get("foundation.hardLandingMinSpeed");
        float plain     = PlainLandingSpeed;
        float stacked   = StackedLandingSpeed;
        float margin    = threshold - stacked;

        DerivedBox(
            "Plain jump lands at",   $"{plain:F1} m/s",
            "Stacked air jump",      $"{stacked:F1} m/s",
            "Margin above stacked",  $"{margin:F1} m/s",
            "", "Both figures are the speed at ride height. The check samples on the airborne-to-"
              + "grounded edge at Sensor Range, so the real value is a few percent lower and this "
              + "over-estimates, which is the safe direction.");
    }

    private void DerivedDodge()
    {
        float strafeCap = Get("propulsion.strafeTopSpeed");
        float deltaV    = DodgeDeltaV;

        string share = strafeCap > 0f ? $"{deltaV / strafeCap * 100f:F0}% of strafe cap" : "strafe cap is 0";

        DerivedBox(
            "Dodge delta-v", $"{deltaV:F2} m/s   ({share})",
            "", $"force x (duration + {Time.fixedDeltaTime:F3} timestep) / 2. Anything over the "
              + "strafe cap is additive and bleeds back down through Strafe Lateral Cap Strength.");
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

        float plainLaunch   = Get("propulsion.jumpImpulseMax");
        float stackedLaunch = StackedLaunchSpeed;

        DerivedBox(
            "Max-charge jump", $"apex {plainLaunch * plainLaunch / (2f * rise):F1} m, "
                             + $"up {plainLaunch / rise:F2} s, down {PlainLandingSpeed / fall:F2} s",
            "Stacked air jump", $"apex {stackedLaunch * stackedLaunch / (2f * rise):F1} m, "
                              + $"up {stackedLaunch / rise:F2} s, down {StackedLandingSpeed / fall:F2} s",
            "", "Both impulses are ForceMode.VelocityChange, so the field values are literally m/s "
              + "and an air jump adds to whatever rise velocity is left.");
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
        {
            cost = "reached by an uncharged tap";
        }
        else if (reqImp > maxImp)
        {
            cost = "UNREACHABLE by any jump";
        }
        else
        {
            float frac = (reqImp - minImp) / Mathf.Max(0.01f, maxImp - minImp);
            cost = $"{frac * 100f:F0}% charge  ({frac * Get("propulsion.jumpMaxChargeTime"):F2} s hold)";
        }

        DerivedBox(
            "Clearance needed",  $"{clearance:F1} m above ride height",
            "Costs",             cost,
            "Tap jump apex",     $"{tapApex:F1} m",
            "Max charge apex",   $"{maxApex:F1} m",
            "", "A floor on WHEN authority is granted, not on what can be done with it: above the "
              + "threshold, airtime still decides whether a rotation can be finished. Clearance is "
              + "measured to the ground below, so a hop off a ledge arms and the same hop on flat "
              + "ground does not.");
    }

    private void DerivedReverseBand()
    {
        float driveCap  = Get("propulsion.reverseTopSpeed");
        float strafeCap = Get("propulsion.strafeTopSpeed");
        float speedMult = GetBool("propulsion.enableBoost")
            ? Get("propulsion.boostSpeedMultiplier")
            : 1f;

        DerivedBox(
            "Drive reverse cap",  $"{driveCap:F1} m/s   (boosted {driveCap * speedMult:F1})",
            "Strafe reverse cap", $"{strafeCap:F1} m/s   (boosted {strafeCap * speedMult:F1})",
            "", "Reverse is strafe-blended: this field is the DRIVE cap and strafe mode converges "
              + "on Strafe Top Speed, so forward, reverse and lateral all share one ceiling at full "
              + "strafe. Braking is not shown here because it runs off Max Reverse Accel, which is "
              + "deliberately not blended and is therefore identical in both modes.");
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

        string duration = perSecond > 0f
            ? $"{maxEnergy / perSecond:F1} s from a full {maxEnergy:F0} pool"
            : "unlimited: costs no energy";

        DerivedBox(
            "Boosted top speed",  $"{Get("propulsion.topSpeed") * speedMult:F1} m/s",
            "Boosted strafe cap", $"{Get("propulsion.strafeTopSpeed") * speedMult:F1} m/s",
            "Continuous boost",   duration);
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
            Warn(ref any, $"Dodge delta-v is {dodge:F1} m/s against a strafe cap of {strafeCap:F1} "
                        + $"({dodge / strafeCap * 100f:F0}%). The dodgeForce tooltip's own guidance is that "
                        + "30 to 60% reads as a sharp sidestep and at or above 100% it reads as a teleport "
                        + "and gets hard to follow at speed.", MessageType.Warning);
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

    /// <summary>Dimmed label/value block. Arguments are label, value, label, value, ...</summary>
    private static void DerivedBox(params string[] labelValuePairs)
    {
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);

        using (new EditorGUI.DisabledScope(true))
        {
            for (int i = 0; i + 1 < labelValuePairs.Length; i += 2)
            {
                if (string.IsNullOrEmpty(labelValuePairs[i]))
                    EditorGUILayout.LabelField(labelValuePairs[i + 1], EditorStyles.wordWrappedMiniLabel);
                else
                    EditorGUILayout.LabelField(labelValuePairs[i], labelValuePairs[i + 1]);
            }
        }

        EditorGUILayout.EndVertical();
    }

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
