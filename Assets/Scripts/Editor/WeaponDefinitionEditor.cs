using UnityEditor;
using UnityEngine;

/// <summary>
/// WeaponDefinitionEditor v2.0
/// ---------------------------
/// Draws only the sections that the selected WeaponType and ProjectileMode actually read, so a
/// Shotgun does not show missile lock settings and a rocket does not show emitter settings.
/// Hidden fields keep their serialized values; this changes what you see, not what is stored.
///
/// Which sections each configuration reads:
///   Combat, Impact      always
///   Flight, Blast       Instantiated mode only
///   Homing              Missile type only, and only when it has a target (not Dumbfire)
///   Emitter             ParticleSystem mode only
///   Wind-up             Automatic type only
///   Lock                Missile type only
///
/// Section headings come free, and this survived the move to nested sections. They are [Header]
/// attributes on the first field of each group, and PropertyField draws a field's decorators
/// along with it, so a heading appears and disappears with the field that carries it. That still
/// holds when the field lives inside a nested [Serializable] class, because we draw the LEAF
/// property directly rather than the container.
///
/// Do not add explicit LabelField headings here. An earlier v2.0 draft assumed nesting broke the
/// trick and drew them by hand; the attributes rendered anyway and every heading appeared twice.
/// The only rule to respect is that each section's heading lives on whichever field is drawn
/// FIRST in that group, so reordering a draw block means moving the [Header] with it.
///
/// **ADDING A FIELD TO A Weapon*Tuning CLASS? IT WILL NOT APPEAR UNTIL YOU ADD IT HERE.** Every
/// property is looked up and drawn by name, so a new field serializes, holds its value and stays
/// completely invisible, with no error to say why. `combat.recoilVelocity` was added and lost this
/// way once already. The camera controller's inspector avoids the whole class of problem by
/// iterating generically, but that is not available here: this editor exists precisely to hide
/// sections a given WeaponType and ProjectileMode do not read, which requires naming them.
/// So the cost of the conditional display is that the field list is a maintenance burden.
/// </summary>
[CustomEditor(typeof(WeaponDefinition))]
[CanEditMultipleObjects]
public class WeaponDefinitionEditor : Editor
{
    // Identity
    private SerializedProperty type, displayName, projectileMode, projectilePrefab;
    // Sections
    private SerializedProperty damage, maxAmmo, startingAmmo, fireRate, recoilVelocity;
    private SerializedProperty impactForce, splashImpactForce, destabilizeFraction;
    private SerializedProperty speed, lifetime, armingDelay;
    private SerializedProperty turnRate, homingDelay, flareOffset, flareFlightFraction, flareDirection, weaveAmplitude, weaveFrequency;
    private SerializedProperty splashRadius, splashFalloff, propUpwardBias, blastLayers, explosionPrefab;
    private SerializedProperty emissionRate, burstCount, startSpeed, startSpeedMin, startLifetime,
                               coneAngle, coneRadius, emitterLayers;
    private SerializedProperty useWindUp, windUpDuration, windUpCurve, windDownDuration;
    private SerializedProperty missileFireMode, lockAcquireTime, lockRange, lockConeAngle, maxLocks, allowRepeatLocks, minLocksToFire, volleyLaunchInterval;

    private SerializedProperty Find(string path) => serializedObject.FindProperty(path);

    private void OnEnable()
    {
        type             = Find("type");
        displayName      = Find("displayName");
        projectileMode   = Find("projectileMode");
        projectilePrefab = Find("projectilePrefab");

        damage         = Find("combat.damage");
        maxAmmo        = Find("combat.maxAmmo");
        startingAmmo   = Find("combat.startingAmmo");
        fireRate       = Find("combat.fireRate");
        recoilVelocity = Find("combat.recoilVelocity");

        impactForce         = Find("impact.impactForce");
        splashImpactForce   = Find("impact.splashImpactForce");
        destabilizeFraction = Find("impact.destabilizeFraction");

        speed       = Find("flight.speed");
        lifetime    = Find("flight.lifetime");
        armingDelay = Find("flight.armingDelay");

        turnRate       = Find("homing.turnRate");
        homingDelay    = Find("homing.homingDelay");
        flareOffset    = Find("homing.flareOffset");
        flareFlightFraction = Find("homing.flareFlightFraction");
        flareDirection = Find("homing.flareDirection");
        weaveAmplitude = Find("homing.weaveAmplitude");
        weaveFrequency = Find("homing.weaveFrequency");

        splashRadius    = Find("blast.splashRadius");
        splashFalloff   = Find("blast.splashFalloff");
        propUpwardBias  = Find("blast.propUpwardBias");
        blastLayers     = Find("blast.damageLayers");
        explosionPrefab = Find("blast.explosionPrefab");

        emissionRate  = Find("emitter.emissionRate");
        burstCount    = Find("emitter.burstCount");
        startSpeed    = Find("emitter.startSpeed");
        startSpeedMin = Find("emitter.startSpeedMin");
        startLifetime = Find("emitter.startLifetime");
        coneAngle     = Find("emitter.coneAngle");
        coneRadius    = Find("emitter.coneRadius");
        emitterLayers = Find("emitter.damageLayers");

        useWindUp        = Find("windUp.useWindUp");
        windUpDuration   = Find("windUp.windUpDuration");
        windUpCurve      = Find("windUp.windUpCurve");
        windDownDuration = Find("windUp.windDownDuration");

        missileFireMode = Find("weaponLock.missileFireMode");
        lockAcquireTime = Find("weaponLock.lockAcquireTime");
        lockRange       = Find("weaponLock.lockRange");
        lockConeAngle   = Find("weaponLock.lockConeAngle");
        maxLocks        = Find("weaponLock.maxLocks");
        allowRepeatLocks = Find("weaponLock.allowRepeatLocks");
        minLocksToFire  = Find("weaponLock.minLocksToFire");
        volleyLaunchInterval = Find("weaponLock.volleyLaunchInterval");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        // Mixed selections can't resolve "which sections apply", so fall back to showing
        // everything rather than hiding a field the user is trying to edit.
        bool mixedType = type.hasMultipleDifferentValues;
        bool mixedMode = projectileMode.hasMultipleDifferentValues;

        var  weaponType  = (WeaponType)type.enumValueIndex;
        var  mode        = (ProjectileMode)projectileMode.enumValueIndex;
        bool isAutomatic = mixedType || weaponType == WeaponType.Automatic;
        bool isMissile   = mixedType || weaponType == WeaponType.Missile;
        bool isInstanced = mixedMode || mode == ProjectileMode.Instantiated;
        bool isParticle  = mixedMode || mode == ProjectileMode.ParticleSystem;

        EditorGUILayout.PropertyField(type);
        EditorGUILayout.PropertyField(displayName);
        EditorGUILayout.PropertyField(projectileMode);
        if (isInstanced) EditorGUILayout.PropertyField(projectilePrefab);

        EditorGUILayout.PropertyField(damage);
        EditorGUILayout.PropertyField(maxAmmo);
        // startingAmmo is ignored when maxAmmo is 0 (unlimited).
        if (maxAmmo.hasMultipleDifferentValues || maxAmmo.intValue > 0)
            EditorGUILayout.PropertyField(startingAmmo);
        EditorGUILayout.PropertyField(fireRate);
        // Always drawn: OnWeaponFired is raised by the projectile path AND the particle path, so
        // either kind of weapon can opt into camera recoil.
        EditorGUILayout.PropertyField(recoilVelocity);

        EditorGUILayout.PropertyField(impactForce);
        // Splash only exists on the projectile path; the particle path has no blast.
        if (isInstanced) EditorGUILayout.PropertyField(splashImpactForce);
        EditorGUILayout.PropertyField(destabilizeFraction);

        if (isInstanced)
        {
            EditorGUILayout.PropertyField(speed);
            EditorGUILayout.PropertyField(lifetime);
            EditorGUILayout.PropertyField(armingDelay);
        }

        // Homing is meaningless on a Dumbfire missile: nothing ever hands it a target, so every
        // value in the section is inert.
        if (isMissile)
        {
            bool mixedFireMode = missileFireMode.hasMultipleDifferentValues;
            var  fireMode      = (MissileFireMode)missileFireMode.enumValueIndex;
            bool guided        = mixedFireMode || fireMode != MissileFireMode.Dumbfire;

            if (guided)
            {
                EditorGUILayout.PropertyField(turnRate);
                EditorGUILayout.PropertyField(homingDelay);
                EditorGUILayout.PropertyField(flareOffset);
                if (flareOffset.hasMultipleDifferentValues || flareOffset.floatValue > 0f)
                {
                    EditorGUILayout.PropertyField(flareFlightFraction);
                    EditorGUILayout.PropertyField(flareDirection);
                }

                // Independent of the flare: the weave runs for the whole flight, so it is drawn
                // whether or not a flare is configured.
                EditorGUILayout.PropertyField(weaveAmplitude);
                if (weaveAmplitude.hasMultipleDifferentValues || weaveAmplitude.floatValue > 0f)
                    EditorGUILayout.PropertyField(weaveFrequency);
            }
        }

        if (isInstanced)
        {
            EditorGUILayout.PropertyField(splashRadius);
            EditorGUILayout.PropertyField(splashFalloff);
            EditorGUILayout.PropertyField(propUpwardBias);
            EditorGUILayout.PropertyField(blastLayers);
            EditorGUILayout.PropertyField(explosionPrefab);
        }

        if (isParticle)
        {
            EditorGUILayout.PropertyField(emissionRate);
            EditorGUILayout.PropertyField(burstCount);
            EditorGUILayout.PropertyField(startSpeed);
            EditorGUILayout.PropertyField(startSpeedMin);
            EditorGUILayout.PropertyField(startLifetime);
            EditorGUILayout.PropertyField(coneAngle);
            EditorGUILayout.PropertyField(coneRadius);
            EditorGUILayout.PropertyField(emitterLayers);
        }

        if (isAutomatic)
        {
            EditorGUILayout.PropertyField(useWindUp);
            if (useWindUp.hasMultipleDifferentValues || useWindUp.boolValue)
            {
                EditorGUILayout.PropertyField(windUpDuration);
                EditorGUILayout.PropertyField(windUpCurve);
                EditorGUILayout.PropertyField(windDownDuration);
            }
        }

        if (isMissile)
        {
            EditorGUILayout.PropertyField(missileFireMode);

            bool mixedFireMode = missileFireMode.hasMultipleDifferentValues;
            var  fireMode      = (MissileFireMode)missileFireMode.enumValueIndex;
            bool scans         = mixedFireMode || fireMode != MissileFireMode.Dumbfire;
            bool hardLock      = mixedFireMode || fireMode == MissileFireMode.HardLock;

            if (hardLock)
            {
                EditorGUILayout.PropertyField(lockAcquireTime);
                EditorGUILayout.PropertyField(maxLocks);
                // Only meaningful once a volley can outnumber the targets in front of you.
                if (maxLocks.hasMultipleDifferentValues || maxLocks.intValue > 1)
                {
                    EditorGUILayout.PropertyField(allowRepeatLocks);
                    EditorGUILayout.PropertyField(minLocksToFire);
                    EditorGUILayout.PropertyField(volleyLaunchInterval);
                }
            }
            if (scans)
            {
                EditorGUILayout.PropertyField(lockRange);
                EditorGUILayout.PropertyField(lockConeAngle);
            }
        }

        DrawDerived(weaponType, mode, mixedType, mixedMode);
        DrawWarnings(weaponType, mode, mixedType, mixedMode);

        serializedObject.ApplyModifiedProperties();
    }

    // =====================================================================================
    // Derived values
    // =====================================================================================
    /// <summary>
    /// Every number here is a PRODUCT of fields above, recomputed on each repaint.
    /// <para>
    /// This block exists because five separate defects in one session were all the same shape:
    /// a value moved and silently changed a quantity that multiplied by it, with nothing
    /// reporting the change. topSpeed invalidated the force model. topSpeed invalidated missile
    /// speed. Emission rate silently rescaled DPS. Missile speed tripled the homing turn radius.
    /// Missile speed tripled the arming distance and killed the rocket jump outright.
    /// </para><para>
    /// Documentation was tried first and is what failed: the impactForce tooltip stated the
    /// coupling correctly, went stale when topSpeed moved 60 -> 105, and then actively caused
    /// the mis-authoring it was written to prevent. A doc is a COPY of a fact and drifts from it.
    /// These are computed from the live values, so they cannot drift, and they sit where the
    /// authoring decision is actually made.
    /// </para><para>
    /// Vehicle figures are read from the VehicleTuningProfile asset and the craft prefab rather
    /// than hardcoded, for exactly the same reason. If either cannot be found this block says so
    /// instead of quietly substituting a constant -- a wrong number here would be worse than none.
    /// </para>
    /// </summary>
    private void DrawDerived(WeaponType weaponType, ProjectileMode mode, bool mixedType, bool mixedMode)
    {
        if (mixedType || mixedMode) return;   // products of mixed values are meaningless

        ResolveVehicleContext(out float topSpeed, out float boosted, out float hoverHeight,
                              out float craftMass, out string contextNote);

        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("Derived — computed live, not authored", EditorStyles.boldLabel);

        using (new EditorGUI.IndentLevelScope())
        {
            if (contextNote != null)
                EditorGUILayout.HelpBox(contextNote, MessageType.Warning);

            if (mode == ProjectileMode.ParticleSystem)
            {
                // Per-particle values, so everything scales with emission rate. Barrel count
                // lives on the vehicle prefab's slot, not here, so this is stated per barrel.
                float rate = emissionRate.floatValue;
                float dps  = damage.floatValue * rate;
                float push = impactForce.floatValue * rate;

                Row("Rounds per second", $"{rate:F0} per barrel");
                Row("Damage per second", $"{dps:F1} per barrel   (damage {damage.floatValue} x rate)");
                if (craftMass > 0f)
                    Row("Push per second held", $"{push / craftMass:F1} m/s per barrel"
                        + (topSpeed > 0f ? $"   ({push / craftMass / topSpeed:P0} of top speed per second)" : ""));

                if (burstCount.intValue > 0)
                {
                    Row("Damage per burst", $"{damage.floatValue * burstCount.intValue:F0} across {burstCount.intValue} pellets");
                    if (craftMass > 0f)
                        Row("Push per full burst", $"{impactForce.floatValue * burstCount.intValue / craftMass:F0} m/s if every pellet lands");
                }

                float pSpeed = startSpeed.floatValue;
                Row("Effective range", $"{pSpeed * startLifetime.floatValue:F0} m   (speed x lifetime)");
                if (rate > 0f)
                    Row("Tracer spacing", $"{pSpeed / rate:F1} m apart   (wide spacing reads as a slow gun)");
                if (boosted > 0f)
                    Row("Particle speed", $"{pSpeed:F0} m/s = {pSpeed / boosted:F2}x a boosted craft");
            }
            else
            {
                if (craftMass > 0f)
                {
                    float dv = impactForce.floatValue / craftMass;
                    Row("Knockback", $"{dv:F0} m/s"
                        + (topSpeed > 0f ? $" = {dv / topSpeed:F2}x top speed  ({(dv > topSpeed ? "REMOVAL" : "disruption")})" : ""));
                    Row("Splash knockback", $"{splashImpactForce.floatValue / craftMass:F0} m/s at the blast centre");
                }

                float armDist = speed.floatValue * armingDelay.floatValue;
                Row("Arming distance", $"{armDist:F1} m   (speed x arming delay)"
                    + (hoverHeight > 0f ? $"   craft hovers at {hoverHeight:F1} m" : ""));
                Row("Max range", $"{speed.floatValue * lifetime.floatValue:F0} m   (speed x lifetime)");

                if (boosted > 0f)
                    Row("Speed", $"{speed.floatValue:F0} m/s = {speed.floatValue / boosted:F2}x a boosted craft"
                        + $"   (closes on a runner at {speed.floatValue - boosted:F0} m/s)");

                if (weaponType == WeaponType.Missile && turnRate.floatValue > 0f)
                {
                    float radius = speed.floatValue / (turnRate.floatValue * Mathf.Deg2Rad);

                    // Stated against lock range, because that is the number it has to be judged
                    // against. A turn radius is not wide or tight on its own — it is wide or
                    // tight relative to the distance the missile is fired from.
                    string vsLock = lockRange.floatValue > 0f
                        ? $"   = {radius / lockRange.floatValue:F2}x lock range ({lockRange.floatValue:F0} m)"
                        : "";

                    Row("Turn radius", $"{radius:F0} m"
                        + "   (speed / turn rate — raising speed WIDENS this)" + vsLock);
                }

                // The flare has to finish inside the flight or the missile arrives still aiming
                // wide. Flight time is range / speed, so tripling speed silently broke this on
                // both homing missiles once already.
                if (weaponType == WeaponType.Missile && flareFlightFraction.floatValue > 0f
                    && flareOffset.floatValue > 0f && speed.floatValue > 0f)
                {
                    // Stated as a departure angle and a share of the trip, because those are the
                    // two things the flare actually controls and neither depends on range. The
                    // old readout gave "resolves after N metres", which was only meaningful while
                    // the duration was authored in seconds.
                    float departure = Mathf.Atan(flareOffset.floatValue) * Mathf.Rad2Deg;
                    Row("Flare shape", $"leaves at {departure:F0} deg, settled by "
                        + $"{flareFlightFraction.floatValue * 100f:F0}% of the way in"
                        + "   (identical at every range)");

                    float lockR = lockRange.floatValue;
                    if (lockR > 0f)
                        Row("Flare at max lock range", $"{lockR / speed.floatValue * flareFlightFraction.floatValue:F2} s "
                            + $"of a {lockR / speed.floatValue:F2} s flight");
                }

                // The squiggle only shows if the nose can keep up with it. One half-swing takes
                // half a period, and the heading has to cover the angle to the offset point in
                // that time — too fast and it averages out into a straight line.
                if (weaponType == WeaponType.Missile && weaveAmplitude.floatValue > 0f
                    && weaveFrequency.floatValue > 0f && turnRate.floatValue > 0f)
                {
                    float halfSwing   = 0.5f / weaveFrequency.floatValue;
                    float swingDeg    = Mathf.Atan(weaveAmplitude.floatValue) * Mathf.Rad2Deg * 2f;
                    float degAvailable = turnRate.floatValue * halfSwing;

                    Row("Weave swing", $"+/-{swingDeg:F0} deg of aim, {weaveFrequency.floatValue:F1} per second");
                    Row("  nose can follow", $"{degAvailable:F0} deg per half swing"
                        + (degAvailable < swingDeg
                            ? "   — TOO FAST, it averages out to straight. Lower the frequency."
                            : "   — enough to trace it"));
                }
            }
        }
    }

    private static void Row(string label, string value) =>
        EditorGUILayout.LabelField(label, value);

    /// <summary>
    /// Pulls live vehicle figures rather than baking them in. Returns a note instead of a
    /// silent fallback when something is missing, because a stale constant here would
    /// reproduce the exact failure this whole block exists to prevent.
    /// </summary>
    private static void ResolveVehicleContext(out float topSpeed, out float boosted,
                                              out float hoverHeight, out float craftMass,
                                              out string note)
    {
        topSpeed = boosted = hoverHeight = craftMass = 0f;
        note = null;

        var profileGuids = AssetDatabase.FindAssets("t:VehicleTuningProfile");
        if (profileGuids.Length > 0)
        {
            var profile = AssetDatabase.LoadAssetAtPath<VehicleTuningProfile>(
                AssetDatabase.GUIDToAssetPath(profileGuids[0]));
            if (profile != null)
            {
                topSpeed    = profile.propulsion.topSpeed;
                boosted     = topSpeed * profile.propulsion.boostSpeedMultiplier;
                hoverHeight = profile.foundation.hoverHeight;
            }
        }

        // Mass lives on the prefab's Rigidbody, not in any profile.
        foreach (var guid in AssetDatabase.FindAssets("t:Prefab"))
        {
            var go = AssetDatabase.LoadAssetAtPath<GameObject>(AssetDatabase.GUIDToAssetPath(guid));
            if (go == null || go.GetComponent<HoverController_Foundation>() == null) continue;
            var rb = go.GetComponent<Rigidbody>();
            if (rb != null) { craftMass = rb.mass; break; }
        }

        if (topSpeed <= 0f && craftMass <= 0f)
            note = "No VehicleTuningProfile or craft prefab found, so the comparisons below are omitted "
                 + "rather than guessed.";
        else if (topSpeed <= 0f)
            note = "No VehicleTuningProfile found. Speed and knockback comparisons are omitted.";
        else if (craftMass <= 0f)
            note = "No craft prefab with a Rigidbody found. Knockback in m/s is omitted.";
    }

    /// <summary>
    /// Flags configurations that compile and serialize fine but do nothing, or do something
    /// the field tooltips explicitly warn against.
    /// </summary>
    private void DrawWarnings(WeaponType weaponType, ProjectileMode mode, bool mixedType, bool mixedMode)
    {
        bool any = false;

        if (!mixedMode && mode == ProjectileMode.Instantiated
            && !projectilePrefab.hasMultipleDifferentValues && projectilePrefab.objectReferenceValue == null)
            Warn(ref any, "Instantiated mode with no projectilePrefab. Firing spawns nothing.", MessageType.Error);

        if (!damage.hasMultipleDifferentValues && damage.floatValue <= 0f)
            Warn(ref any, "damage is 0. This weapon applies knockback but never reduces health.", MessageType.Warning);

        if (!maxAmmo.hasMultipleDifferentValues && !startingAmmo.hasMultipleDifferentValues
            && maxAmmo.intValue > 0 && startingAmmo.intValue > maxAmmo.intValue)
            Warn(ref any, "startingAmmo exceeds maxAmmo and will be clamped.", MessageType.Warning);

        // Camera recoil lands once per shot, so its right value is a function of fire rate. The
        // same number that thumps on a missile becomes a permanent tremor on anything automatic.
        if (!recoilVelocity.hasMultipleDifferentValues && !fireRate.hasMultipleDifferentValues
            && recoilVelocity.floatValue > 0.2f && fireRate.floatValue > 10f)
            Warn(ref any, $"Recoil Velocity {recoilVelocity.floatValue:F2} at {fireRate.floatValue:F0} "
                        + "shots per second is a kick every few frames, which reads as a continuous "
                        + "tremor rather than as impact. Keep it under 0.2 above 10/sec, or leave it "
                        + "at 0 and sell the weapon through muzzle VFX.", MessageType.Warning);

        // The destabilizeFraction tooltip's own guidance: per-bullet torque accumulates every frame.
        if (!mixedType && !mixedMode && weaponType == WeaponType.Automatic
            && mode == ProjectileMode.ParticleSystem
            && !destabilizeFraction.hasMultipleDifferentValues && destabilizeFraction.floatValue > 0f)
            Warn(ref any, "destabilizeFraction above 0 on a sustained-fire weapon. Per-bullet torque "
                        + "accumulates every frame and spins targets uncontrollably. Intended for "
                        + "burst weapons.", MessageType.Warning);

        // Projectiles read their tuning through IProjectileDefinitionCarrier. A prefab that does
        // not implement it (nor the legacy carriers) ignores this entire asset.
        if (!mixedMode && mode == ProjectileMode.Instantiated
            && !projectilePrefab.hasMultipleDifferentValues
            && projectilePrefab.objectReferenceValue is GameObject prefab
            && prefab.GetComponent<IProjectileDefinitionCarrier>() == null
            && prefab.GetComponent<IProjectileImpactCarrier>() == null)
            Warn(ref any, $"'{prefab.name}' implements neither IProjectileDefinitionCarrier nor the "
                        + "legacy IProjectileImpactCarrier, so nothing on this asset ever reaches it.",
                        MessageType.Warning);

        // A flare aims at a point beside the target and slides onto it. With no steering authority
        // the missile can leave, but it can never come back.
        if (!mixedType && weaponType == WeaponType.Missile
            && !flareOffset.hasMultipleDifferentValues && flareOffset.floatValue > 0f
            && !turnRate.hasMultipleDifferentValues && turnRate.floatValue <= 0f)
            Warn(ref any, "Flare Offset is set but Turn Rate is 0, so the missile will swing wide and "
                        + "never converge. Raise Turn Rate or clear the flare.", MessageType.Warning);

        if (!mixedType && weaponType == WeaponType.Missile
            && !homingDelay.hasMultipleDifferentValues && !lifetime.hasMultipleDifferentValues
            && homingDelay.floatValue >= lifetime.floatValue)
            Warn(ref any, "Homing Delay is longer than Lifetime. The missile expires before it is "
                        + "allowed to steer, so it can only ever fly straight.", MessageType.Warning);

        // The tunnelling class of bug: a projectile that moves further than its own collider in one
        // physics step relies entirely on the sweep in ProjectileSweep to catch anything.
        if (!mixedMode && mode == ProjectileMode.Instantiated
            && !speed.hasMultipleDifferentValues
            && projectilePrefab.objectReferenceValue is GameObject p2)
        {
            var col = p2.GetComponentInChildren<Collider>();
            if (col != null)
            {
                float perStep = speed.floatValue * Time.fixedDeltaTime;
                float extent  = Mathf.Min(col.bounds.extents.x, Mathf.Min(col.bounds.extents.y, col.bounds.extents.z));
                if (extent > 0f && perStep > extent)
                    Warn(ref any, $"At speed {speed.floatValue:F0} this projectile moves {perStep:F2}m per "
                               + $"physics step against a collider only {extent:F2}m across, so Unity's own "
                               + "contact callbacks will not fire. Detection relies entirely on the swept "
                               + "test in ProjectileSweep. That is by design, but do not remove the sweep.",
                               MessageType.Info);
            }
        }

        if (!mixedType && !mixedMode)
            DrawCouplingWarnings(ref any, weaponType, mode);
    }

    /// <summary>
    /// Invariants that were each discovered by measurement rather than by reading the code,
    /// after the value that broke them had already shipped into the assets. Each one describes
    /// a configuration that compiles, serialises and reads as a DESIGN problem in play, which
    /// is what makes them expensive: you go and tune the wrong thing.
    /// </summary>
    private void DrawCouplingWarnings(ref bool any, WeaponType weaponType, ProjectileMode mode)
    {
        ResolveVehicleContext(out float topSpeed, out float boosted, out float hoverHeight,
                              out float craftMass, out _);

        if (mode == ProjectileMode.Instantiated)
        {
            // Killed the rocket jump. Arming distance is speed x delay, so tripling a
            // projectile's speed triples the range inside which it refuses to detonate.
            // Nothing flags it: the weapon still fires and still explodes, just never near you.
            float armDist = speed.floatValue * armingDelay.floatValue;
            if (hoverHeight > 0f && armDist > hoverHeight)
                Warn(ref any, $"Arming distance is {armDist:F1}m (speed {speed.floatValue:F0} x arming delay "
                            + $"{armingDelay.floatValue}), which is further than the craft hovers above the "
                            + $"ground ({hoverHeight:F1}m). This projectile CANNOT detonate below its own "
                            + "firer, so rocket jumps and any close-quarters use are dead. Measured: at 10m "
                            + "arming the self-shove was 1.9 m/s; at 3.5m it was 34. Lower the arming delay "
                            + "as you raise speed.", MessageType.Warning);

            // The 5.5 rule: projectiles are faster than vehicles.
            if (boosted > 0f && speed.floatValue > 0f && speed.floatValue < boosted)
                Warn(ref any, $"Speed {speed.floatValue:F0} is BELOW a boosted craft ({boosted:F0} m/s), so a "
                            + "target running in a straight line simply outruns this. The design rule is that "
                            + "speed catches and turn rate is what skill evades.", MessageType.Warning);

            // Raising speed silently widens the turning circle, which reads as "homing is broken".
            if (weaponType == WeaponType.Missile && turnRate.floatValue > 0f && speed.floatValue > 0f)
            {
                float radius = speed.floatValue / (turnRate.floatValue * Mathf.Deg2Rad);

                // Judged against LOCK RANGE, not against a fixed number of metres. The old test
                // was "radius > 80m", and it stayed silent on the Soft Homing missile at a 56m
                // circle — which measured 0% hits on a crossing target because 56m is over half
                // of its 100m lock range. A turn radius is only meaningful next to the distance
                // the missile is actually fired from, so compare the two.
                float lockR = lockRange.floatValue;
                if (lockR > 0f && radius > lockR * 0.5f)
                    Warn(ref any, $"Turn radius is {radius:F0}m but lock range is {lockR:F0}m, so the missile's "
                                + $"turning circle is {radius / lockR:F2}x the distance it gets fired from. It "
                                + "cannot correct inside its own engagement: measured, a missile in this state "
                                + "misses a crossing target outright rather than curving onto it. Raise turn "
                                + "rate, lower speed, or shorten lock range so the weapon only offers shots it "
                                + "can service.", MessageType.Warning);
                else if (lockR <= 0f && radius > 80f)
                    Warn(ref any, $"Turn radius is {radius:F0}m (speed / turn rate). That circle is wider than "
                                + "most engagements, so this missile will miss anything that is not flying "
                                + "straight at it. Raising speed widens this proportionally — raise turn rate "
                                + "with it, or accept it as the evasion dial deliberately.", MessageType.Warning);
            }

            // Self-spin scales with splash x selfImpactScale x destabilizeFraction. Measured
            // straight down: 0.30 tilted the firer 50 degrees, 0.45 flipped and downed it.
            if (destabilizeFraction.floatValue >= 0.4f && splashImpactForce.floatValue > 0f)
                Warn(ref any, $"destabilizeFraction {destabilizeFraction.floatValue:F2} is at or past the "
                            + "measured point where a rocket jump flips the player who fired it (0.45 flipped "
                            + "and downed the firer; 0.30 reached 50 degrees and recovered). Note this ceiling "
                            + "MOVES with splashImpactForce and selfImpactScale — re-measure if you raise "
                            + "either.", MessageType.Warning);
        }

        if (mode == ProjectileMode.ParticleSystem)
        {
            // Damage and force are PER PARTICLE, so changing rate silently rescales both.
            if (boosted > 0f && startSpeed.floatValue > 0f && startSpeed.floatValue < boosted)
                Warn(ref any, $"Particle speed {startSpeed.floatValue:F0} is below a boosted craft "
                            + $"({boosted:F0} m/s), so a boosting target outruns these rounds.",
                            MessageType.Warning);

            if (emissionRate.floatValue > 0f && startSpeed.floatValue > 0f)
            {
                float spacing = startSpeed.floatValue / emissionRate.floatValue;
                if (spacing > 40f)
                    Warn(ref any, $"Tracers land {spacing:F0}m apart (speed / emission rate), which reads as a "
                                + "sparse dotted line rather than a stream. Note raising SPEED makes this "
                                + "worse, not better — rate is the dial that fills the gap.", MessageType.Info);
            }
        }

        if (weaponType == WeaponType.Missile && mode == ProjectileMode.Instantiated)
        {
            // The old warning here checked that the flare resolved inside the flight, and it is
            // gone because that failure is no longer expressible: the flare is authored as a
            // SHARE of the flight, so it scales with every shot instead of being a fixed number
            // of seconds that a close shot could outrun. What is left is the one thing a share
            // can still get wrong — asking for so much of the trip that nothing is left to
            // converge in.
            if (flareOffset.floatValue > 0f && flareFlightFraction.floatValue > 0.65f)
                Warn(ref any, $"The flare occupies {flareFlightFraction.floatValue * 100f:F0}% of the "
                            + "flight, which leaves too little of it to converge. The missile arrives "
                            + "still turning onto the target and misses regardless of range. Measured, "
                            + "0.6 is the most that still lands reliably.", MessageType.Warning);

            // Alternate produces exactly two roll angles, so a volley larger than two collapses
            // into two overlapping stacks and reads as having fired two missiles.
            if ((MissileFlareMode)flareDirection.enumValueIndex == MissileFlareMode.Alternate
                && flareOffset.floatValue > 0f && maxLocks.intValue > 2)
                Warn(ref any, $"Flare Direction is Alternate on a {maxLocks.intValue}-missile volley. "
                            + "Alternate only ever swings left or right, so the salvo flies as two "
                            + "overlapping stacks and looks like two missiles. Use Random to fan "
                            + "them properly.", MessageType.Warning);

            // A weave the nose cannot follow averages out to a straight line: the dial appears
            // to do nothing at all, which reads as the feature being broken.
            if (weaveAmplitude.floatValue > 0f && weaveFrequency.floatValue > 0f && turnRate.floatValue > 0f)
            {
                float swingDeg     = Mathf.Atan(weaveAmplitude.floatValue) * Mathf.Rad2Deg * 2f;
                float degAvailable = turnRate.floatValue * (0.5f / weaveFrequency.floatValue);
                if (degAvailable < swingDeg)
                    Warn(ref any, $"The weave asks for {swingDeg:F0} deg of aim swing but the nose can "
                                + $"only cover {degAvailable:F0} deg in half a cycle at Turn Rate "
                                + $"{turnRate.floatValue:F0}. The missile cannot follow its own squiggle, "
                                + "so it averages out into a straight line and this looks like it does "
                                + "nothing. Lower Weave Frequency or raise Turn Rate.", MessageType.Warning);
            }

            // Weave and flare both work by moving the AIM POINT, which only exists when the
            // missile has something to aim at.
            if ((weaveAmplitude.floatValue > 0f || flareOffset.floatValue > 0f)
                && turnRate.floatValue <= 0f)
                Warn(ref any, "Flare and Weave both steer the missile toward a shifted aim point, and "
                            + "this missile has Turn Rate 0 so it never steers at all. Both are inert "
                            + "here.", MessageType.Warning);

            // Lifetime is a duration, so range moves whenever speed does.
            if (speed.floatValue > 0f && lifetime.floatValue > 0f)
            {
                float maxRange = speed.floatValue * lifetime.floatValue;
                if (maxRange > 800f)
                    Warn(ref any, $"Maximum range is {maxRange:F0}m (speed x lifetime). That is usually "
                                + "an accident rather than a choice: lifetime is a DURATION, so range "
                                + "grows every time speed does. Around 2s is a few hundred metres at "
                                + "these speeds.", MessageType.Info);
            }

            // Straight flight spent not steering, subtracted from an already short flight.
            if (homingDelay.floatValue > 0f && speed.floatValue > 0f)
            {
                float dumbDistance = homingDelay.floatValue * speed.floatValue;
                if (dumbDistance > 40f)
                    Warn(ref any, $"Homing Delay is {dumbDistance:F0}m of straight flight at this speed. "
                                + "Any target closer than that is engaged by what is effectively a "
                                + "dumbfire, because the missile arrives before it ever steers.",
                                MessageType.Warning);
            }
        }

        // A volley you can never assemble reads as a weapon that simply refuses to fire.
        if (weaponType == WeaponType.Missile
            && (MissileFireMode)missileFireMode.enumValueIndex == MissileFireMode.HardLock
            && minLocksToFire.intValue > maxLocks.intValue)
            Warn(ref any, $"minLocksToFire ({minLocksToFire.intValue}) is above maxLocks "
                        + $"({maxLocks.intValue}). It is clamped at runtime so this behaves as "
                        + "'always require a full volley', but the two values disagree and the "
                        + "next person to read them will not know which was intended.",
                        MessageType.Warning);

        // The hold IS the weapon's cost, and it is the product of these two rather than either.
        if (weaponType == WeaponType.Missile
            && (MissileFireMode)missileFireMode.enumValueIndex == MissileFireMode.HardLock
            && maxLocks.intValue > 1)
        {
            float fullHold = lockAcquireTime.floatValue * maxLocks.intValue;
            if (fullHold > 3f)
                Warn(ref any, $"A full volley takes {fullHold:F1}s of holding "
                            + $"({lockAcquireTime.floatValue}s x {maxLocks.intValue} locks), which is "
                            + "longer than most engagements last. Lock Acquire Time is per lock, not "
                            + "per volley.", MessageType.Warning);
        }

        // Reads as "the weapon does nothing" while looking correctly configured.
        if (weaponType == WeaponType.Automatic && fireRate.floatValue > 0f && fireRate.floatValue < 0.5f)
            Warn(ref any, $"fireRate {fireRate.floatValue} means one shot event every "
                        + $"{1f / fireRate.floatValue:F0} seconds. For a ParticleSystem weapon the emitters "
                        + "still play, so this is invisible until ammo or recoil is switched on — then the "
                        + "weapon stops working with no obvious cause.", MessageType.Warning);
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
