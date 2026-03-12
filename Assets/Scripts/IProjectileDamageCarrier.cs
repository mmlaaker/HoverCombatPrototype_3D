/// <summary>
/// IProjectileDamageCarrier v1.0
/// ------------------------------
/// Thin interface that allows HoverController_Weapons to pass damage values
/// to spawned projectiles without coupling to any specific projectile implementation.
///
/// Implement this on any projectile MonoBehaviour that needs to receive damage
/// from the weapon that spawned it.
///
/// Usage:
///   var carrier = spawnedProjectile.GetComponent&lt;IProjectileDamageCarrier&gt;();
///   carrier?.SetDamage(weaponDefinition.damage);
///
/// Contract:
///   • SetDamage is called once immediately after Instantiate, before the
///     projectile's first Update or FixedUpdate.
///   • The projectile is responsible for storing and applying this value.
///   • Not all projectiles need to implement this — GetComponent returns null
///     gracefully if the projectile handles damage differently.
/// </summary>
public interface IProjectileDamageCarrier
{
    /// <summary>
    /// Receives the damage value from the spawning weapon.
    /// Called once immediately after the projectile is instantiated.
    /// </summary>
    /// <param name="damage">Damage amount defined by the weapon's WeaponDefinition.</param>
    void SetDamage(float damage);
}
