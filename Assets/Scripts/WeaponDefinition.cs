using UnityEngine;

/// <summary>
/// WeaponDefinition v1.0
/// ----------------------
/// Shared data asset describing a single weapon type.
/// Create via: Assets → Create → Weapons → Weapon Definition
///
/// One asset is referenced by all vehicles that carry this weapon.
/// Changing a value here updates every vehicle simultaneously.
///
/// Does NOT contain:
///   • Runtime state (ammo, cooldown) — lives in WeaponSlot
///   • Vehicle-specific scene references (muzzle transforms, particle emitters) — lives in WeaponSlot
///
/// Naming convention: WD_MachinGun, WD_Rail, WD_Missile_HardLock, WD_Shotgun, etc.
/// </summary>
[CreateAssetMenu(fileName = "WD_NewWeapon", menuName = "Weapons/Weapon Definition")]
public class WeaponDefinition : ScriptableObject
{
    // -------------------------------------------------------------------------
    // 🔫 Identity
    // -------------------------------------------------------------------------
    [Header("🔫 Identity")]
    [Tooltip("Firing behavior class. Determines which input events and state machines are active.")]
    public WeaponType type = WeaponType.SingleShot;

    [Tooltip("Display name shown in UI.")]
    public string displayName = "Weapon";

    // -------------------------------------------------------------------------
    // 💥 Projectile Mode
    // -------------------------------------------------------------------------
    [Header("💥 Projectile Mode")]
    [Tooltip("Instantiated: spawns projectilePrefab at each muzzle point per fire event.\n" +
             "ParticleSystem: drives pre-placed emitters via Play()/Stop(). " +
             "Muzzle points are unused in this mode.")]
    public ProjectileMode projectileMode = ProjectileMode.Instantiated;

    [Tooltip("Prefab spawned at each muzzle point on fire.\n" +
             "Only used when projectileMode is Instantiated. Must not be null in that mode.\n" +
             "Implement IProjectileDamageCarrier on the prefab to receive damage from this definition.")]
    public GameObject projectilePrefab;

    // -------------------------------------------------------------------------
    // ⚔️ Combat
    // -------------------------------------------------------------------------
    [Header("⚔️ Combat")]
    [Tooltip("Damage applied per hit.\n" +
             "For Instantiated weapons: passed to the projectile via IProjectileDamageCarrier.\n" +
             "For ParticleSystem weapons: read directly by the ParticleWeaponCollision script " +
             "on the emitter GameObject.")]
    [Min(0f)]
    public float damage = 25f;

    [Tooltip("Maximum ammo capacity. Set to 0 for unlimited ammo.")]
    [Min(0)]
    public int maxAmmo = 30;

    [Tooltip("Starting ammo on spawn. Ignored when maxAmmo is 0.")]
    [Min(0)]
    public int startingAmmo = 30;

    [Tooltip("Shots per second. Controls the minimum interval between fire events.\n" +
             "For ParticleSystem weapons: controls ammo consumption cadence only. " +
             "Bullet visual rate is set by the ParticleSystem's Emission Rate.")]
    [Min(0.01f)]
    public float fireRate = 10f;

    [Tooltip("Impulse force applied to the hit Rigidbody along the projectile's travel direction. " +
             "Used by ParticleWeaponCollision on emitter GameObjects and by projectile hit handlers. " +
             "Set to 0 for no knockback.")]
    [Min(0f)]
    public float impactForce = 50f;

    // -------------------------------------------------------------------------
    // 🌀 Wind-up (Automatic only)
    // -------------------------------------------------------------------------
    [Header("🌀 Wind-up (Automatic only)")]
    [Tooltip("Enable wind-up scaling. When false, fire rate is constant from the first frame.\n" +
             "Disable for basic machine guns. Enable for Minigun spin-up feel.")]
    public bool useWindUp = false;

    [Tooltip("Seconds to reach full wind-up from cold. Only used when useWindUp is true.")]
    [Min(0.01f)]
    public float windUpDuration = 1.5f;

    [Tooltip("Maps wind-up progress (0..1) to fire rate scale (0..1).\n" +
             "Only used when useWindUp is true.")]
    public AnimationCurve windUpCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);

    [Tooltip("Seconds for wind-up to fully decay after fire is released.\n" +
             "Set to 0 for instant reset. Only used when useWindUp is true.")]
    [Min(0f)]
    public float windDownDuration = 0.5f;

    // -------------------------------------------------------------------------
    // 🎯 Missile Lock (Missile only)
    // -------------------------------------------------------------------------
    [Header("🎯 Missile Lock (Missile only)")]
    [Tooltip("When false: dumbfire mode — fires immediately on FirePressed, no lock required.\n" +
             "When true: FireHeld accumulates lock; FirePressed commits on confirmed lock.")]
    public bool useMissileLock = true;

    [Tooltip("Seconds of sustained FireHeld over a target to confirm lock.\n" +
             "Only used when useMissileLock is true.")]
    [Min(0.01f)]
    public float lockAcquireTime = 1.5f;

    [Tooltip("Maximum lock-on scan range in metres.\n" +
             "Only used when useMissileLock is true.")]
    [Min(1f)]
    public float lockRange = 80f;

    [Tooltip("Half-angle of the lock-on cone in degrees. Target must stay inside this cone.\n" +
             "Only used when useMissileLock is true.")]
    [Range(1f, 45f)]
    public float lockConeAngle = 15f;
}
