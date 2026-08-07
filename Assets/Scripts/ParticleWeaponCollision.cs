using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ParticleWeaponCollision v2.0
/// ---------------------------------
/// Attach to the same GameObject as a ParticleSystem-mode weapon emitter. Handles
/// OnParticleCollision (damage and impact force) and, as of v2.0, owns configuring the emitter.
///
/// v2.0: the emitter is now CONFIGURED FROM the WeaponDefinition rather than authored by hand.
///
/// Emission rate, burst count, muzzle velocity, bullet lifetime, spread and the collision mask
/// all come from `weaponDefinition.emitter`. This is the one tuning section that has to be pushed
/// rather than read live, because Unity's particle simulation reads its settings off this
/// ParticleSystem component and not off our asset. It is applied in Awake for play mode and from
/// OnValidate (plus WeaponDefinition's own OnValidate) so the editor never shows a stale number.
///
/// Before this, those values lived four override layers deep -- VFX_*.prefab, into the vehicle
/// prefab, into a prefab variant, into the scene instance -- and the player and AI craft had
/// quietly drifted apart on emission rate and muzzle velocity without anyone intending it.
///
/// Requirements, now ENFORCED in code rather than merely documented (they were silent failure
/// modes when an artist toggled them off):
///   • Collision module enabled, Type World, Send Collision Messages ON, Lifetime Loss 1.
///   • Any damageable object implements IDamageable on itself or a parent.
///   • Any object that should receive impact force has a Rigidbody.
/// </summary>
public class ParticleWeaponCollision : MonoBehaviour
{
    [Tooltip("Shared weapon definition asset. Must be the same asset assigned to this weapon's " +
             "WeaponSlot on the vehicle. ALL tuning is read from here, including how this emitter " +
             "is configured, so there is nothing left to hand-author on the ParticleSystem.")]
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

    /// <summary>The definition driving this emitter. Used by WeaponDefinition's editor sync.</summary>
    public WeaponDefinition Definition => weaponDefinition;

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
            Debug.LogError("[ParticleWeaponCollision] No WeaponDefinition assigned. This emitter is " +
                           "unconfigured and every hit will deal 0 damage.", this);

        ApplyDefinitionToEmitter();
    }

#if UNITY_EDITOR
    private void OnValidate()
    {
        // Delayed so we are not mutating objects mid-serialization or mid-import.
        UnityEditor.EditorApplication.delayCall += () =>
        {
            if (this == null) return;
            ApplyDefinitionToEmitter();
        };
    }
#endif

    /// <summary>
    /// Writes the definition's emitter section into this ParticleSystem. Safe to call in the
    /// editor or at runtime, and safe to call repeatedly.
    ///
    /// VehicleLayerAssigner (execution order -20) has already put this object on its vehicle's
    /// layer by the time Awake runs here, which is what makes the self-layer strip below work.
    /// </summary>
    public void ApplyDefinitionToEmitter()
    {
        if (ps == null) ps = GetComponent<ParticleSystem>();
        if (ps == null || weaponDefinition == null) return;

        var e = weaponDefinition.emitter;

        // --- rate of fire ---
        var emission = ps.emission;
        emission.enabled = true;
        emission.rateOverTime = e.emissionRate;

        if (e.burstCount > 0)
        {
            // Preserve whatever cycle/interval the VFX was authored with; only the count is tuning.
            var existing = emission.burstCount > 0 ? emission.GetBurst(0)
                                                   : new ParticleSystem.Burst(0f, 1, 1, 0.01f);
            existing.count = e.burstCount;
            emission.SetBursts(new[] { existing });
        }
        else
        {
            emission.SetBursts(new ParticleSystem.Burst[0]);
        }

        // --- bullet ---
        var main = ps.main;
        main.startSpeed = (e.startSpeedMin > 0f && e.startSpeedMin < e.startSpeed)
            ? new ParticleSystem.MinMaxCurve(e.startSpeedMin, e.startSpeed)
            : new ParticleSystem.MinMaxCurve(e.startSpeed);
        main.startLifetime = e.startLifetime;

        // --- spread ---
        var shape = ps.shape;
        shape.enabled = true;
        shape.shapeType = ParticleSystemShapeType.Cone;
        shape.angle  = e.coneAngle;
        shape.radius = e.coneRadius;

        // --- collision ---
        // These four are hard requirements, not preferences: without sendCollisionMessages the
        // OnParticleCollision below never fires at all, and without lifetimeLoss 1 a single pellet
        // keeps hitting and re-applying its impact force.
        var collision = ps.collision;
        collision.enabled = true;
        collision.type = ParticleSystemCollisionType.World;
        collision.mode = ParticleSystemCollisionMode.Collision3D;
        collision.sendCollisionMessages = true;
        collision.lifetimeLoss = 1f;

        // Author the full layer set once on the definition; strip the firer's own layer here so a
        // single shared value yields "everyone except me" on both the player and the AI. The old
        // code could only subtract from a hand-authored mask, which is why the mask had to be
        // duplicated per vehicle prefab.
        collision.collidesWith = e.damageLayers.value & ~(1 << gameObject.layer);
    }

    private void OnParticleCollision(GameObject other)
    {
        if (weaponDefinition == null) return;

        int count = ParticlePhysicsExtensions.GetCollisionEvents(ps, other, collisionEvents);

        bool debug = ShouldDrawDebug;
        float damage = weaponDefinition.combat.damage;
        float force  = weaponDefinition.impact.impactForce;
        float destab = weaponDefinition.impact.destabilizeFraction;

        // NOTE (pooling debt): GetComponentInParent walks the hierarchy on every hit.
        // At high machine gun emission rates this accumulates. When projectile pooling
        // is introduced, cache IDamageable and Rigidbody on the pooled object at
        // retrieval time rather than walking the hierarchy per collision event.
        for (int i = 0; i < count; i++)
        {
            // Apply damage. Walk up the hierarchy — bullets typically hit child colliders.
            var damageable = other.GetComponentInParent<IDamageable>();
            damageable?.TakeDamage(damage);

            // Apply impact force if a Rigidbody is present. intersection is the world-space
            // contact point, already available here, so destabilizeFraction has a real lever
            // arm to work with. Shared with the projectile path via WeaponImpact so both
            // destabilize identically for the same fraction.
            var rb = other.GetComponentInParent<Rigidbody>();
            WeaponImpact.Apply(rb,
                               collisionEvents[i].velocity,
                               collisionEvents[i].intersection,
                               force,
                               destab,
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
