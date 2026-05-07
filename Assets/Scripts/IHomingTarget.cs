using UnityEngine;

/// <summary>
/// IHomingTarget v1.0
///
/// Thin interface that allows HoverController_Weapons to pass a homing target
/// to spawned missiles without coupling to any specific missile implementation.
///
/// Implement this on any projectile MonoBehaviour that needs to track a target.
///
/// Usage:
///   var homing = spawnedProjectile.GetComponent&lt;IHomingTarget&gt;();
///   homing?.SetTarget(LockTarget);
///
/// Contract:
///   SetTarget is called once immediately after Instantiate, before the
///   missile's first Update or FixedUpdate.
///   Target may be null. The missile is responsible for handling that case
///   (typically by flying straight).
///   Not all projectiles need to implement this. GetComponent returns null
///   gracefully if the projectile is dumbfire-only.
/// </summary>
public interface IHomingTarget
{
    /// <summary>
    /// Receives the homing target from the spawning weapon.
    /// Called once immediately after the projectile is instantiated.
    /// </summary>
    /// <param name="target">Transform to home toward. May be null.</param>
    void SetTarget(Transform target);
}
