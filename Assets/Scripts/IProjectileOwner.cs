using UnityEngine;

/// <summary>
/// IProjectileOwner v1.0
/// ---------------------
/// Tells a spawned projectile which vehicle fired it.
///
/// Sibling of IProjectileDefinitionCarrier and IHomingTarget, pushed by
/// FireAllMuzzles at spawn through the same null-conditional pattern, so a prefab
/// implements it only if it needs it.
///
/// Why this exists: a rocket's splash is an OverlapSphere against
/// blast.damageLayers, and that mask includes both vehicle layers (193). Without
/// an owner the firer is indistinguishable from any other victim caught in its own
/// blast, which meant taking full enemy-strength knockback from your own rocket and
/// -- the moment combat.damage stops being 0 -- full self-damage with it. Neither
/// was chosen; they were what happened in the absence of the concept.
///
/// With an owner the firer becomes a distinct case that can be authored:
/// self-damage is always suppressed, and the self shove is scaled by
/// WeaponImpactTuning.selfImpactScale, which is the rocket jump dial. Setting that
/// to 0 excludes the firer from its own blast entirely.
///
/// Contract:
///   SetOwner is called once immediately after Instantiate, before the projectile's
///   first Start / Update / FixedUpdate.
///   The Transform passed is the firing vehicle's ROOT, which is where IDamageable
///   (VehicleHealth) lives, so implementers can resolve it with a single
///   GetComponentInChildren.
///   May be null in principle (a projectile spawned by something that is not a
///   vehicle); implementers must treat null as "no owner, nothing to exclude".
/// </summary>
public interface IProjectileOwner
{
    /// <summary>
    /// Receives the firing vehicle's root transform from the spawning weapon.
    /// Called once immediately after the projectile is instantiated.
    /// </summary>
    /// <param name="owner">Root transform of the vehicle that fired. May be null.</param>
    void SetOwner(Transform owner);
}
