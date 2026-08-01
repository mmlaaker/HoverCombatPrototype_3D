using UnityEditor;
using UnityEngine;

/// <summary>
/// WeaponDefinitionEditor v1.0
/// ---------------------------
/// Draws only the fields that the selected WeaponType and ProjectileMode actually read.
/// Wind-up is Automatic only, lock settings are Missile only, and splashImpactForce is
/// Instantiated only. Hidden fields keep their serialized values; this changes what you see,
/// not what is stored.
///
/// Which fields each mode reads (verified against the runtime, not the tooltips):
///   damage             Both paths. SetDamage on the projectile, direct read for particles.
///   impactForce        Both paths. Per particle for ParticleSystem, per direct hit for
///                      Instantiated, so the sane range differs by orders of magnitude.
///   splashImpactForce  Instantiated only, and only if the prefab has splash.
///   destabilizeFraction Both paths, via the shared WeaponImpact helper.
/// </summary>
[CustomEditor(typeof(WeaponDefinition))]
[CanEditMultipleObjects]
public class WeaponDefinitionEditor : Editor
{
    private SerializedProperty type, displayName;
    private SerializedProperty projectileMode, projectilePrefab;
    private SerializedProperty damage, maxAmmo, startingAmmo, fireRate;
    private SerializedProperty impactForce, splashImpactForce, destabilizeFraction;
    private SerializedProperty useWindUp, windUpDuration, windUpCurve, windDownDuration;
    private SerializedProperty missileFireMode, lockAcquireTime, lockRange, lockConeAngle;

    private void OnEnable()
    {
        type                = serializedObject.FindProperty(nameof(WeaponDefinition.type));
        displayName         = serializedObject.FindProperty(nameof(WeaponDefinition.displayName));
        projectileMode      = serializedObject.FindProperty(nameof(WeaponDefinition.projectileMode));
        projectilePrefab    = serializedObject.FindProperty(nameof(WeaponDefinition.projectilePrefab));
        damage              = serializedObject.FindProperty(nameof(WeaponDefinition.damage));
        maxAmmo             = serializedObject.FindProperty(nameof(WeaponDefinition.maxAmmo));
        startingAmmo        = serializedObject.FindProperty(nameof(WeaponDefinition.startingAmmo));
        fireRate            = serializedObject.FindProperty(nameof(WeaponDefinition.fireRate));
        impactForce         = serializedObject.FindProperty(nameof(WeaponDefinition.impactForce));
        splashImpactForce   = serializedObject.FindProperty(nameof(WeaponDefinition.splashImpactForce));
        destabilizeFraction = serializedObject.FindProperty(nameof(WeaponDefinition.destabilizeFraction));
        useWindUp           = serializedObject.FindProperty(nameof(WeaponDefinition.useWindUp));
        windUpDuration      = serializedObject.FindProperty(nameof(WeaponDefinition.windUpDuration));
        windUpCurve         = serializedObject.FindProperty(nameof(WeaponDefinition.windUpCurve));
        windDownDuration    = serializedObject.FindProperty(nameof(WeaponDefinition.windDownDuration));
        missileFireMode     = serializedObject.FindProperty(nameof(WeaponDefinition.missileFireMode));
        lockAcquireTime     = serializedObject.FindProperty(nameof(WeaponDefinition.lockAcquireTime));
        lockRange           = serializedObject.FindProperty(nameof(WeaponDefinition.lockRange));
        lockConeAngle       = serializedObject.FindProperty(nameof(WeaponDefinition.lockConeAngle));
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        // Mixed selections can't resolve "which fields apply", so fall back to showing
        // everything rather than hiding a field the user is trying to edit.
        bool mixedType = type.hasMultipleDifferentValues;
        bool mixedMode = projectileMode.hasMultipleDifferentValues;

        var  weaponType = (WeaponType)type.enumValueIndex;
        var  mode       = (ProjectileMode)projectileMode.enumValueIndex;
        bool isAutomatic  = mixedType || weaponType == WeaponType.Automatic;
        bool isMissile    = mixedType || weaponType == WeaponType.Missile;
        bool isInstanced  = mixedMode || mode == ProjectileMode.Instantiated;

        // Section headings come from the [Header] attributes on WeaponDefinition: PropertyField
        // draws a field's decorators with it, so a heading disappears along with the field that
        // carries it. Wind-up and Missile headers sit on the first field of their group, which is
        // why hiding those groups hides their headings too.
        EditorGUILayout.PropertyField(type);
        EditorGUILayout.PropertyField(displayName);

        EditorGUILayout.PropertyField(projectileMode);
        if (isInstanced)
            EditorGUILayout.PropertyField(projectilePrefab);

        EditorGUILayout.PropertyField(damage);
        EditorGUILayout.PropertyField(maxAmmo);
        // startingAmmo is ignored when maxAmmo is 0 (unlimited).
        if (maxAmmo.hasMultipleDifferentValues || maxAmmo.intValue > 0)
            EditorGUILayout.PropertyField(startingAmmo);
        EditorGUILayout.PropertyField(fireRate);
        EditorGUILayout.PropertyField(impactForce);
        // Splash only exists on the projectile path; the particle path has no blast.
        if (isInstanced)
            EditorGUILayout.PropertyField(splashImpactForce);
        EditorGUILayout.PropertyField(destabilizeFraction);

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
                EditorGUILayout.PropertyField(lockAcquireTime);
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

        // The destabilizeFraction tooltip's own guidance: per-bullet torque accumulates every frame.
        if (!mixedType && !mixedMode && weaponType == WeaponType.Automatic
            && mode == ProjectileMode.ParticleSystem
            && !destabilizeFraction.hasMultipleDifferentValues && destabilizeFraction.floatValue > 0f)
            Warn(ref any, "destabilizeFraction above 0 on a sustained-fire weapon. Per-bullet torque "
                        + "accumulates every frame and spins targets uncontrollably. Intended for "
                        + "burst weapons.", MessageType.Warning);

        // Knockback reaches an Instantiated projectile only through IProjectileImpactCarrier.
        // A prefab that doesn't implement it silently ignores every value in this section.
        if (!mixedMode && mode == ProjectileMode.Instantiated
            && !projectilePrefab.hasMultipleDifferentValues
            && projectilePrefab.objectReferenceValue is GameObject prefab
            && prefab.GetComponent<IProjectileImpactCarrier>() == null
            && (impactForce.floatValue > 0f || splashImpactForce.floatValue > 0f))
            Warn(ref any, $"'{prefab.name}' does not implement IProjectileImpactCarrier, so the impact "
                        + "values above never reach it. Implement the interface on the prefab, or set "
                        + "the forces to 0 to make that explicit.", MessageType.Warning);
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
