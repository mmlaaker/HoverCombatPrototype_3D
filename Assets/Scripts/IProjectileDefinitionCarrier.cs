/// <summary>
/// IProjectileDefinitionCarrier v1.0
/// ---------------------------------
/// Implemented by a spawned projectile that reads its tuning from the firing WeaponDefinition.
/// HoverController_Weapons.FireAllMuzzles hands the asset over immediately after Instantiate and
/// before the projectile's first tick.
///
/// This supersedes the older IProjectileDamageCarrier / IProjectileImpactCarrier pair for any
/// projectile that adopts it. Those pushed a handful of individual floats, which meant every new
/// tunable needed either a new interface or a wider signature, and anything not pushed had to
/// stay stranded on the prefab. Handing over the whole asset lets the projectile read whatever it
/// needs, live, with no copying and therefore nothing to drift.
///
/// The older interfaces are still honoured by FireAllMuzzles, so a prefab that implements only
/// those keeps working.
/// </summary>
public interface IProjectileDefinitionCarrier
{
    /// <param name="definition">The firing weapon's shared tuning asset. Never null in practice;
    /// implementations should still fail loudly rather than silently flying with defaults.</param>
    void SetDefinition(WeaponDefinition definition);
}
