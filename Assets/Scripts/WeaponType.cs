/// <summary>
/// Determines the firing behavior of a weapon slot.
/// Used by WeaponDefinition and HoverController_Weapons.
/// </summary>
public enum WeaponType
{
    /// <summary>One projectile per FirePressed event. Governed by cooldown.</summary>
    SingleShot,

    /// <summary>Continuous fire while FireHeld. Governed by fireRate. Optional wind-up.</summary>
    Automatic,

    /// <summary>FireHeld accumulates lock on target. FirePressed commits when lock confirmed.</summary>
    Missile,

    /// <summary>One mine spawned per FirePressed at muzzle position. No projectile force applied.</summary>
    Mine
}
