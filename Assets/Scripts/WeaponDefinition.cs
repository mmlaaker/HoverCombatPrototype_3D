using UnityEngine;

/// <summary>
/// WeaponDefinition v1.1
/// ----------------------
/// Shared data asset describing a single weapon type.
/// Create via: Assets → Create → Weapons → Weapon Definition
///
/// One asset is referenced by all vehicles that carry this weapon.
/// Changing a value here updates every vehicle simultaneously.
///
/// Does NOT contain:
///   • Runtime state (ammo, cooldown) -- lives in WeaponSlot
///   • Vehicle-specific scene references (muzzle transforms, particle emitters) -- lives in WeaponSlot
///   • Blast geometry (splashRadius, splashFalloff, damageLayers) -- lives on the projectile
///     prefab, because it has to match the explosion VFX. See splashImpactForce for why that
///     split is expected to be temporary.
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
    [Tooltip("How the weapon fires. Sets which input it listens to and which firing logic runs it.\n" +
             "SingleShot: one shot per trigger pull.\n" +
             "Automatic: keeps firing while the trigger is held.\n" +
             "Missile: guided or unguided projectile. See Missile Fire Mode below.\n" +
             "Mine: drops a deployable at the muzzle with no launch speed.")]
    public WeaponType type = WeaponType.SingleShot;

    [Tooltip("Name shown on the HUD.")]
    public string displayName = "Weapon";

    // -------------------------------------------------------------------------
    // 💥 Projectile Mode
    // -------------------------------------------------------------------------
    [Header("💥 Projectile Mode")]
    [Tooltip("How a shot is delivered.\n" +
             "Instantiated: spawns the prefab below at every muzzle point, one per shot. Use for " +
             "rockets and mines, anything that flies on its own physics.\n" +
             "ParticleSystem: switches pre-placed particle emitters on and off instead of spawning " +
             "anything. Use for guns that fire a lot of small bullets. Muzzle points are ignored.")]
    public ProjectileMode projectileMode = ProjectileMode.Instantiated;

    [Tooltip("What gets spawned when you fire. Required for Instantiated mode, unused otherwise.\n" +
             "The prefab is handed its damage and knockback by this asset the moment it spawns, so " +
             "tune those here and not on the prefab.")]
    public GameObject projectilePrefab;

    // -------------------------------------------------------------------------
    // ⚔️ Combat
    // -------------------------------------------------------------------------
    [Header("⚔️ Combat")]
    [Tooltip("Health removed per hit.\n" +
             "Watch out: for ParticleSystem weapons this counts PER PARTICLE, so a shotgun that " +
             "fires 40 pellets can take off 40x this in a single blast.")]
    [Min(0f)]
    public float damage = 25f;

    [Tooltip("Ammo capacity. Set to 0 for unlimited, which is what the Machine Gun uses.")]
    [Min(0)]
    public int maxAmmo = 30;

    [Tooltip("How much ammo you spawn with. Ignored when maxAmmo is 0.")]
    [Min(0)]
    public int startingAmmo = 30;

    [Tooltip("Shots per second.\n" +
             "For ParticleSystem weapons this only paces ammo drain and the gap between bursts. How " +
             "fast bullets actually appear is the emitter's own Emission rate, so turning this up " +
             "will not make the gun look faster.")]
    [Min(0.01f)]
    public float fireRate = 10f;

    [Tooltip("How hard a direct hit shoves the target, along the direction the shot was travelling. " +
             "0 for no knockback at all.\n" +
             "The two delivery modes need wildly different numbers here, so never copy a value from " +
             "one to the other:\n" +
             "ParticleSystem counts PER PARTICLE, so a 40 pellet blast lands 40x this. Think " +
             "hundreds, up to a few thousand.\n" +
             "Instantiated applies it once, on the hit. Think tens of thousands before a rocket " +
             "visibly punts a vehicle.")]
    [Min(0f)]
    public float impactForce = 50f;

    [Tooltip("How hard the explosion shoves everything caught in it. Pushes outward from the blast " +
             "centre and weakens with distance.\n" +
             "Rockets only. Weapons that do not explode ignore this.\n" +
             "NOTE: the SIZE of the blast is not set here. Radius, falloff and which layers get " +
             "caught all live on the projectile prefab, because they have to line up with the " +
             "explosion VFX. Those VFX are strictly placeholder right now, so this split is " +
             "temporary. Once the real explosions land, blast geometry should move onto this asset " +
             "with everything else and the prefab should go back to being pure delivery.")]
    [Min(0f)]
    public float splashImpactForce = 0f;

    [Tooltip("How much of a hit lands where it actually connected, instead of being applied dead " +
             "centre.\n" +
             "At 0 every hit is a clean shove: the target slides away and never spins, no matter how " +
             "hard you hit it or where. Turn it up and hit location starts to matter. Clip a flank " +
             "and the vehicle rotates, catch it square and it just gets shoved.\n" +
             "This does not change how FAR anything gets knocked back, only how much it tumbles, so " +
             "you can tune spin and shove separately.\n" +
             "Where to start:\n" +
             "0 for anything you hold down. Spin from each bullet stacks frame after frame and sends " +
             "vehicles into a spin nobody can recover from.\n" +
             "0.15 for rockets. A glancing hit wobbles and recovers, a solid off-centre hit tips past " +
             "the point of no return and commits to a flip.\n" +
             "1 for the Shotgun. 40 pellets hitting 40 different spots do the work on their own: a " +
             "centred blast has its impulses cancel out into a hard shove, while one that catches a " +
             "flank concentrates them and spins the target hard.\n" +
             "Past about 0.3 the worst off-centre hits run into Unity's spin speed ceiling and start " +
             "looking identical to each other, so cranking it higher buys you nothing.")]
    [Range(0f, 1f)]
    public float destabilizeFraction = 0f;

    // -------------------------------------------------------------------------
    // 🌀 Wind-up (Automatic only)
    // -------------------------------------------------------------------------
    [Header("🌀 Wind-up (Automatic only)")]
    [Tooltip("Whether the gun has to spin up before it reaches full speed. Off for a plain machine " +
             "gun that fires at full rate instantly, on for the Minigun.")]
    public bool useWindUp = false;

    [Tooltip("Seconds to go from cold to fully spun up.")]
    [Min(0.01f)]
    public float windUpDuration = 1.5f;

    [Tooltip("Shape of the spin-up. Left edge is cold, right edge is fully spun. Height is the share " +
             "of full fire rate at that moment.\n" +
             "A straight line ramps evenly. A curve that stays low then rises late gives a slow " +
             "growl that suddenly opens up.")]
    public AnimationCurve windUpCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);

    [Tooltip("Seconds to spin back down to cold after you let go of fire.\n" +
             "0 snaps back instantly, which throws away the Minigun feel: tapping fire would give " +
             "full rate every time.")]
    [Min(0f)]
    public float windDownDuration = 0.5f;

    // -------------------------------------------------------------------------
    // 🎯 Missile Fire Mode (Missile only)
    // -------------------------------------------------------------------------
    [Header("🎯 Missile Fire Mode (Missile only)")]
    [Tooltip("Dumbfire: launches the instant you press. Flies straight, tracks nothing.\n" +
             "SoftHoming: also launches instantly, then grabs whoever is in the cone and chases " +
             "them. No waiting around, which is what makes it the harassment tool.\n" +
             "HardLock: hold fire to build a lock, let go to launch. Slow to set up, but it will " +
             "not miss.")]
    public MissileFireMode missileFireMode = MissileFireMode.HardLock;

    [Tooltip("How long you have to keep the target inside the cone before the lock confirms. " +
             "HardLock only.")]
    [Min(0.01f)]
    public float lockAcquireTime = 1.5f;

    [Tooltip("How far out the weapon can see targets, in metres. Used by SoftHoming and HardLock.")]
    [Min(1f)]
    public float lockRange = 80f;

    [Tooltip("How wide the targeting cone is, measured from the centre outward, in degrees. Small " +
             "means you have to be pointed almost straight at them; large is forgiving but will " +
             "happily grab the wrong target in a crowd. Used by SoftHoming and HardLock.")]
    [Range(1f, 45f)]
    public float lockConeAngle = 15f;
}
