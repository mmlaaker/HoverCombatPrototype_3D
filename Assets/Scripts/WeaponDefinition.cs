using UnityEngine;

/// <summary>
/// WeaponDefinition v2.0
/// ----------------------
/// Shared data asset describing a single weapon. One asset is referenced by every vehicle that
/// carries this weapon, so changing a value here updates all of them at once.
/// Create via: Assets → Create → Weapons → Weapon Definition
///
/// v2.0: THIS IS NOW THE ONLY PLACE WEAPON TUNING LIVES.
///
/// Tuning used to be spread across this asset, the projectile prefabs, and the particle emitters,
/// and the emitter values in particular sat four override layers deep (VFX_*.prefab, into the
/// vehicle prefab, into a prefab variant, into the scene instance). Answering "what emission rate
/// does the Machine Gun actually use" took three files, and the player and AI craft had silently
/// drifted apart. All of it now lives here, in sections mirroring VehicleTuningProfile.
///
/// How values reach the things that use them:
///   • Projectiles hold a reference to this asset and read it LIVE, exactly like the hover
///     controllers read VehicleTuningProfile. Nothing is copied, so nothing can drift, and
///     editing a value during play mode affects projectiles already in the air.
///   • Particle emitters cannot work that way, because Unity's particle simulation reads its
///     settings off the ParticleSystem component rather than off this asset. So
///     ParticleWeaponCollision WRITES the emitter section into the emitter: at runtime in Awake,
///     and in the editor via OnValidate below so the inspector never shows a stale number.
///
/// Deliberately NOT here, and none of these are oversights:
///   • Scene references (muzzle transforms, emitter lists, the VFX mount) -- WeaponSlot, because
///     they are per-vehicle objects rather than values.
///   • Runtime state (current ammo, cooldown, wind-up progress) -- WeaponSlot.
///   • Debug flags (drawDebug, logSplash, debugSettings) -- per instance, not tuning.
///   • Emitter GameObject layer -- overwritten at runtime by VehicleLayerAssigner anyway.
/// The one thing that looks per-vehicle but is not: collision masks. Author the full set of
/// layers in the emitter section and the firer's own layer is stripped automatically, so the
/// player and the AI each end up unable to shoot themselves from one shared value.
///
/// Naming convention: WD_MachineGuns, WD_Rail, WD_HardLockMissile, WD_Shotgun, etc.
/// </summary>
[CreateAssetMenu(fileName = "WD_NewWeapon", menuName = "Weapons/Weapon Definition")]
public class WeaponDefinition : ScriptableObject
{
    // -------------------------------------------------------------------------
    // 🔫 Identity
    // -------------------------------------------------------------------------
    [Header("🔫 Identity")]
    [Tooltip("How the weapon fires. Sets which input it listens to and which firing logic runs it.\n" +
             "SingleShot: one shot per trigger pull.\n" +
             "Automatic: keeps firing while the trigger is held.\n" +
             "Missile: guided or unguided projectile. See the Lock section.\n" +
             "Mine: drops a deployable at the muzzle with no launch speed.")]
    public WeaponType type = WeaponType.SingleShot;

    [Tooltip("Name shown on the HUD.")]
    public string displayName = "Weapon";

    [Tooltip("How a shot is delivered.\n" +
             "Instantiated: spawns the prefab below at every muzzle point, one per shot. Use for " +
             "rockets and mines, anything that flies on its own physics. Reads the Flight, Homing " +
             "and Blast sections.\n" +
             "ParticleSystem: switches pre-placed particle emitters on and off instead of spawning " +
             "anything. Use for guns that fire a lot of small bullets. Reads the Emitter section; " +
             "muzzle points are ignored.")]
    public ProjectileMode projectileMode = ProjectileMode.Instantiated;

    [Tooltip("What gets spawned when you fire. Required for Instantiated mode, unused otherwise.\n" +
             "The prefab is handed this whole asset the moment it spawns and reads its tuning from " +
             "here, so there is nothing to tune on the prefab itself.")]
    public GameObject projectilePrefab;

    // -------------------------------------------------------------------------
    // Tuning sections. Same shape as VehicleTuningProfile: field-initialized so a freshly
    // created asset has sane defaults rather than a pile of zeroes.
    // -------------------------------------------------------------------------
    public WeaponCombatTuning  combat  = new WeaponCombatTuning();
    public WeaponImpactTuning  impact  = new WeaponImpactTuning();
    public WeaponFlightTuning  flight  = new WeaponFlightTuning();
    public WeaponHomingTuning  homing  = new WeaponHomingTuning();
    public WeaponBlastTuning   blast   = new WeaponBlastTuning();
    public WeaponEmitterTuning emitter = new WeaponEmitterTuning();
    public WeaponWindUpTuning  windUp  = new WeaponWindUpTuning();
    public WeaponLockTuning    weaponLock = new WeaponLockTuning();

#if UNITY_EDITOR
    /// <summary>
    /// Editor-time sync, so the emitter section is not a set of numbers that only mean something
    /// once you press play. Projectiles do not need this because they read the asset live; particle
    /// systems do, because Unity reads their settings off the component.
    ///
    /// This is the project's only OnValidate. The hover controllers deliberately have none, since
    /// nothing caches VehicleTuningProfile and there is nothing to push.
    /// </summary>
    private void OnValidate()
    {
        // Delayed: OnValidate can fire during serialization and asset import, where touching other
        // objects is unsafe. delayCall runs it once the editor is back in a normal state.
        UnityEditor.EditorApplication.delayCall += SyncEmittersInEditor;
    }

    private void SyncEmittersInEditor()
    {
        // The asset can be deleted between OnValidate and the delayed call.
        if (this == null) return;

        foreach (var pwc in FindObjectsByType<ParticleWeaponCollision>(
                     FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (pwc == null || pwc.Definition != this) continue;
            pwc.ApplyDefinitionToEmitter();
            UnityEditor.EditorUtility.SetDirty(pwc);
        }
    }
#endif
}
