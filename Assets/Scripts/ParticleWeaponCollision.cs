using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ParticleWeaponCollision v1.0
/// ---------------------------------
/// Attach to the same GameObject as a ParticleSystem-mode weapon emitter.
/// Handles OnParticleCollision callbacks — applies damage and impact force to hit objects.
///
/// Damage is read directly from the assigned WeaponDefinition asset.
/// Assign the same WeaponDefinition asset used in the vehicle's WeaponSlot.
/// Changing damage on the asset updates this script and all vehicles simultaneously.
///
/// Requirements:
///   • ParticleSystem Collision module must be enabled with:
///       - Type: World
///       - Mode: 3D
///       - Send Collision Messages: ON  (required for OnParticleCollision to fire)
///       - Lifetime Loss: 1             (kills particle on first hit)
///       - Collides With: enemy layers only
///   • Any damageable object must implement IDamageable on itself or a parent.
///   • Any object that should receive impact force must have a Rigidbody.
/// </summary>
public class ParticleWeaponCollision : MonoBehaviour
{
    [Tooltip("Shared weapon definition asset. Must be the same asset assigned to this " +
             "weapon's WeaponSlot on the vehicle. Damage value is read from here.")]
    [SerializeField] private WeaponDefinition weaponDefinition;

    [Header("🛠 Debug")]
    [Tooltip("Shared debug asset. When assigned, its master toggle overrides the local flag below. " +
             "Leave empty to use the local flag.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    [Tooltip("Mark every particle contact point in the Scene view, coloured by what the pellet " +
             "found: green hit something damageable, yellow hit a Rigidbody only, red hit plain " +
             "geometry and was absorbed.\n" +
             "Red clusters on flat ground while accelerating are the 'bullets eaten' symptom: the " +
             "emitter is colliding with the firer's own hull or the ground ahead of it. Also shows " +
             "the shotgun's spread pattern, which is what makes destabilizeFraction 1 work.")]
    [SerializeField] private bool drawDebug = false;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.enableDebugGizmos : drawDebug;

    private ParticleSystem ps;

    // Pre-allocated list reused every OnParticleCollision call — avoids per-frame GC allocation.
    private readonly List<ParticleCollisionEvent> collisionEvents = new List<ParticleCollisionEvent>();

    private void Awake()
    {
        ps = GetComponent<ParticleSystem>();

        if (ps == null)
            Debug.LogError("[ParticleWeaponCollision] No ParticleSystem found on this GameObject.", this);

        if (weaponDefinition == null)
            Debug.LogError("[ParticleWeaponCollision] No WeaponDefinition assigned. " +
                           "Damage will be 0 on all hits.", this);

        ConfigureCollisionMask();
    }

    /// <summary>
    /// Sets the particle system's Collides With mask to exclude this vehicle's own layer.
    /// VehicleLayerAssigner (execution order -20) runs before this, so the hierarchy
    /// is already on the correct layer by the time Awake fires here.
    /// </summary>
    private void ConfigureCollisionMask()
    {
        if (ps == null) return;

        int ownLayer = gameObject.layer;
        var collision = ps.collision;

        // Start from current mask, strip out own layer so bullets pass through teammates.
        int mask = collision.collidesWith;
        mask &= ~(1 << ownLayer);
        collision.collidesWith = mask;
    }

    private void OnParticleCollision(GameObject other)
    {
        int count = ParticlePhysicsExtensions.GetCollisionEvents(ps, other, collisionEvents);

        bool debug = ShouldDrawDebug;

        // NOTE (pooling debt): GetComponentInParent walks the hierarchy on every hit.
        // At high machine gun emission rates this accumulates. When projectile pooling
        // is introduced, cache IDamageable and Rigidbody on the pooled object at
        // retrieval time rather than walking the hierarchy per collision event.
        for (int i = 0; i < count; i++)
        {
            // Apply damage. Walk up the hierarchy — bullets typically hit child colliders.
            var damageable = other.GetComponentInParent<IDamageable>();
            damageable?.TakeDamage(weaponDefinition != null ? weaponDefinition.damage : 0f);

            // Apply impact force if a Rigidbody is present. intersection is the world-space
            // contact point, already available here, so destabilizeFraction has a real lever
            // arm to work with. Shared with the projectile path via WeaponImpact so both
            // destabilize identically for the same fraction.
            var rb = other.GetComponentInParent<Rigidbody>();
            if (weaponDefinition != null)
                WeaponImpact.Apply(rb,
                                   collisionEvents[i].velocity,
                                   collisionEvents[i].intersection,
                                   weaponDefinition.impactForce,
                                   weaponDefinition.destabilizeFraction,
                                   1f,
                                   debug);

            // Drawn for every contact, including the ones that found nothing to damage. Those
            // are the interesting ones when pellets are going missing.
            if (debug)
                WeaponDebugDraw.ParticleContact(collisionEvents[i].intersection,
                                                collisionEvents[i].velocity,
                                                rb != null,
                                                damageable != null);
        }
    }
}
