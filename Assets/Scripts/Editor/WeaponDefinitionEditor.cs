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
    private SerializedProperty turnRate, homingDelay, flareOffset, flareDuration, flareDirection;
    private SerializedProperty splashRadius, splashFalloff, blastLayers, explosionPrefab;
    private SerializedProperty emissionRate, burstCount, startSpeed, startSpeedMin, startLifetime,
                               coneAngle, coneRadius, emitterLayers;
    private SerializedProperty useWindUp, windUpDuration, windUpCurve, windDownDuration;
    private SerializedProperty missileFireMode, lockAcquireTime, lockRange, lockConeAngle;

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
        flareDuration  = Find("homing.flareDuration");
        flareDirection = Find("homing.flareDirection");

        splashRadius    = Find("blast.splashRadius");
        splashFalloff   = Find("blast.splashFalloff");
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
                    EditorGUILayout.PropertyField(flareDuration);
                    EditorGUILayout.PropertyField(flareDirection);
                }
            }
        }

        if (isInstanced)
        {
            EditorGUILayout.PropertyField(splashRadius);
            EditorGUILayout.PropertyField(splashFalloff);
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

            if (hardLock) EditorGUILayout.PropertyField(lockAcquireTime);
            if (scans)
            {
                EditorGUILayout.PropertyField(lockRange);
                EditorGUILayout.PropertyField(lockConeAngle);
            }
        }

        DrawWarnings(weaponType, mode, mixedType, mixedMode);

        serializedObject.ApplyModifiedProperties();
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
