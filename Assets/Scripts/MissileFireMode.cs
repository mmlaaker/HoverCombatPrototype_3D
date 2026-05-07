/// <summary>
/// Firing behavior for WeaponType.Missile.
/// Selected on the WeaponDefinition asset.
/// </summary>
public enum MissileFireMode
{
    /// <summary>
    /// Fires immediately on FirePressed. No scanning, no homing.
    /// Behaves like SingleShot. Use for unguided rockets.
    /// </summary>
    Dumbfire,

    /// <summary>
    /// Fires immediately on FirePressed. Scans the cone once at fire time and
    /// passes the best in-cone target to the missile via IHomingTarget.
    /// If no target is in cone, the missile fires straight.
    /// Quick to use; less precise than HardLock.
    /// </summary>
    SoftHoming,

    /// <summary>
    /// FireHeld accumulates lock on a target. Releasing FireHeld while Locked
    /// launches the missile. Releasing while still Scanning aborts the lock.
    /// Most committed input; rewards keeping the cone on a target.
    /// </summary>
    HardLock
}
