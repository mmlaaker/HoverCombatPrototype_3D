/// <summary>
/// Determines how a weapon spawns its visual effect and routes damage.
/// Used by WeaponDefinition and HoverController_Weapons.
/// </summary>
public enum ProjectileMode
{
    /// <summary>
    /// Spawns projectilePrefab at each muzzle point in WeaponSlot.muzzlePoints on fire.
    /// Damage is passed to the prefab via IProjectileDamageCarrier if implemented.
    /// Use for Rail, Missile, Shotgun, Mine, Ricochet Disk.
    /// </summary>
    Instantiated,

    /// <summary>
    /// Drives pre-placed ParticleSystem emitters in WeaponSlot.particleEmitters
    /// via Play() / Stop(). Muzzle points are not used in this mode.
    /// Damage is handled by a collision script on the emitter GameObject.
    /// Use for machine guns, laser beams, or any sustained spray effect.
    /// </summary>
    ParticleSystem
}
