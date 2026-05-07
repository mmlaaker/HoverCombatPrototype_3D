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

    /// <summary>Dumbfire, soft-homing, or hold-to-lock missile. See MissileFireMode for sub-mode behavior.</summary>
    Missile,

    /// <summary>One mine spawned per FirePressed at muzzle position. No projectile force applied.</summary>
    Mine
}
