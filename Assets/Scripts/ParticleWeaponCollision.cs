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
    }

    private void OnParticleCollision(GameObject other)
    {
        int count = ParticlePhysicsExtensions.GetCollisionEvents(ps, other, collisionEvents);

        for (int i = 0; i < count; i++)
        {
            // Apply damage. Walk up the hierarchy — bullets typically hit child colliders.
            var damageable = other.GetComponentInParent<IDamageable>();
            damageable?.TakeDamage(weaponDefinition != null ? weaponDefinition.damage : 0f);

            // Apply impact force if a Rigidbody is present.
            var rb = other.GetComponentInParent<Rigidbody>();
            if (rb != null && weaponDefinition != null && weaponDefinition.impactForce > 0f)
            {
                Vector3 forceDir = collisionEvents[i].velocity.normalized;
                rb.AddForce(forceDir * weaponDefinition.impactForce, ForceMode.Impulse);
            }
        }
    }
}
