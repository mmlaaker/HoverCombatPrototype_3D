/// <summary>
/// IProjectileImpactCarrier v1.0
/// ------------------------------
/// Lets HoverController_Weapons hand knockback values to a spawned projectile, the same way
/// IProjectileDamageCarrier hands it damage.
///
/// Exists so impact force is authored once on the WeaponDefinition instead of per-prefab.
/// Before this, WeaponDefinition.impactForce was read only by ParticleWeaponCollision while
/// the projectile path carried its own serialized copy on RocketProjectile, so the same
/// inspector field meant two different things depending on projectileMode, and the values
/// sitting on the missile definitions were dead data.
///
/// Usage:
///   spawned.GetComponent&lt;IProjectileImpactCarrier&gt;()?
///          .SetImpact(def.impactForce, def.splashImpactForce, def.destabilizeFraction);
///
/// Contract:
///   • Called once immediately after Instantiate, before the projectile's first Update or
///     FixedUpdate, and before it can collide.
///   • Projectiles without splash ignore splashForce.
///   • Optional. GetComponent returns null gracefully for prefabs that handle their own
///     knockback.
/// </summary>
public interface IProjectileImpactCarrier
{
    /// <summary>
    /// Receives knockback values from the spawning weapon.
    /// </summary>
    /// <param name="directHitForce">Impulse applied to a directly hit Rigidbody.</param>
    /// <param name="splashForce">Impulse applied to Rigidbodies caught in splash, before falloff.</param>
    /// <param name="destabilizeFraction">0..1 share of each impulse applied at the contact point.</param>
    void SetImpact(float directHitForce, float splashForce, float destabilizeFraction);
}
