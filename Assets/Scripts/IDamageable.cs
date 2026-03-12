/// <summary>
/// IDamageable
/// -----------
/// Implement on any GameObject that can receive damage.
/// MachineGunParticleCollision and projectile hit handlers call TakeDamage
/// via GetComponentInParent — place this on the vehicle root or the component
/// that owns the health pool.
/// </summary>
public interface IDamageable
{
    /// <summary>
    /// Apply incoming damage to this object's health pool.
    /// </summary>
    /// <param name="amount">Raw damage value from the weapon definition.</param>
    void TakeDamage(float amount);
}
