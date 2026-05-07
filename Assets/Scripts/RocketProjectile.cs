using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// RocketProjectile v1.1
///
/// Dumbfire or soft-homing rocket modeled after the Halo rocket launcher.
/// Flies at constant speed, explodes on impact (or after lifetime), applies
/// direct-hit damage plus splash damage with falloff, and knocks back any
/// Rigidbody it catches.
///
/// When the spawning weapon supplies a target via IHomingTarget.SetTarget,
/// the rocket steers toward it with turn-rate limited correction; otherwise
/// it flies straight.
///
/// Damage value is supplied by the firing weapon via IProjectileDamageCarrier.
/// Everything else is tuned per-prefab on this script.
///
/// Prefab setup:
///   Rigidbody: Use Gravity OFF, Collision Detection: Continuous Dynamic, Drag 0
///   Collider:  convex (Capsule or Box), NOT a trigger
///   Layer:     a Projectile layer that doesn't collide with the firer's vehicle layer
///   Damage Layers: enemy vehicle layers (and any other splash targets)
///   Explosion Prefab: a self-destructing VFX prefab spawned at impact point
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class RocketProjectile : MonoBehaviour, IProjectileDamageCarrier, IHomingTarget
{
    // -------------------------------------------------------------------------
    // ✈️ Flight
    // -------------------------------------------------------------------------
    [Header("✈️ Flight")]
    [Tooltip("How fast the rocket travels in metres per second. Halo-style: 60 to 90.")]
    [SerializeField] private float speed = 70f;

    [Tooltip("Maximum time the rocket can fly before self-detonating. " +
             "Prevents stray rockets from drifting forever.")]
    [SerializeField] private float lifetime = 6f;

    [Tooltip("Short delay after spawn during which collisions are ignored. " +
             "Prevents the rocket from exploding on the firer's own collider at the muzzle. " +
             "Try 0.05 to 0.15.")]
    [SerializeField] private float armingDelay = 0.05f;

    // -------------------------------------------------------------------------
    // 💥 Impact
    // -------------------------------------------------------------------------
    [Header("💥 Impact")]
    [Tooltip("Splash radius in metres. Anything inside takes damage scaled by the falloff curve. " +
             "Halo-style: 5 to 8.")]
    [SerializeField] private float splashRadius = 6f;

    [Tooltip("Maps distance from the explosion centre (0 = centre, 1 = edge) to a damage and " +
             "force multiplier (0 to 1). Default: smooth falloff to zero at the edge.")]
    [SerializeField] private AnimationCurve splashFalloff = AnimationCurve.EaseInOut(0f, 1f, 1f, 0f);

    [Tooltip("Knockback applied to a directly hit Rigidbody, along the rocket's flight direction. " +
             "Tune this big enough to visibly punt a vehicle.")]
    [SerializeField] private float directHitImpactForce = 40f;

    [Tooltip("Knockback applied to Rigidbodies caught in splash. Pushed outward from the centre. " +
             "Scaled by the falloff curve.")]
    [SerializeField] private float splashImpactForce = 25f;

    [Tooltip("Layers that take splash damage and force. Set to enemy vehicle layers. " +
             "Terrain doesn't need to be in here; the rocket already explodes on terrain hit.")]
    [SerializeField] private LayerMask damageLayers = ~0;

    // -------------------------------------------------------------------------
    // 🎨 VFX
    // -------------------------------------------------------------------------
    [Header("🎨 VFX")]
    [Tooltip("Spawned at the impact point on detonation. Should self-destruct after its visuals finish.")]
    [SerializeField] private GameObject explosionPrefab;

    // -------------------------------------------------------------------------
    // 🎯 Homing
    // -------------------------------------------------------------------------
    [Header("🎯 Homing")]
    [Tooltip("Maximum rotation toward the homing target in degrees per second. " +
             "Only applies when a target is supplied via IHomingTarget.SetTarget. " +
             "0 disables steering (rocket flies straight even with a target). " +
             "60 to 120 feels soft; 180+ feels aggressive.")]
    [Min(0f)]
    [SerializeField] private float turnRate = 90f;

    // -------------------------------------------------------------------------
    // 🛠 Debug
    // -------------------------------------------------------------------------
    [Header("🛠 Debug")]
    [Tooltip("Draw the splash radius as a wire sphere around the rocket in the Scene view.")]
    [SerializeField] private bool drawDebug = true;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private Rigidbody rb;
    private float damage;
    private float age;
    private bool  exploded;
    private Transform homingTarget;

    // Reused per-explosion to dedupe IDamageable hits across a vehicle's
    // multiple child colliders. Static is safe: Explode is fully synchronous.
    private static readonly HashSet<IDamageable> _splashedThisExplosion = new();

    /// <summary>Receives the damage value from the spawning WeaponDefinition.</summary>
    public void SetDamage(float dmg) => damage = dmg;

    /// <summary>Receives the homing target from the spawning weapon. Null = dumbfire.</summary>
    public void SetTarget(Transform target) => homingTarget = target;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void Start()
    {
        // Fire-and-forget: set velocity once; no further force needed.
        // Rigidbody must have gravity off so the rocket stays level.
        rb.linearVelocity = transform.forward * speed;
    }

    private void Update()
    {
        age += Time.deltaTime;

        if (homingTarget != null && turnRate > 0f)
        {
            Vector3 toTarget = homingTarget.position - transform.position;
            if (toTarget.sqrMagnitude > 0.0001f)
            {
                Quaternion desired = Quaternion.LookRotation(toTarget);
                transform.rotation = Quaternion.RotateTowards(
                    transform.rotation, desired, turnRate * Time.deltaTime);
                rb.linearVelocity = transform.forward * speed;
            }
        }

        if (!exploded && age >= lifetime)
            Explode(transform.position, null);
    }

    private void OnCollisionEnter(Collision col)
    {
        if (exploded || age < armingDelay)
            return;

        Vector3 hitPoint = col.contactCount > 0 ? col.GetContact(0).point : transform.position;
        Explode(hitPoint, col.collider);
    }

    // -------------------------------------------------------------------------
    // Detonation
    // -------------------------------------------------------------------------
    /// <summary>
    /// Direct hit: full damage, knockback along flight direction.
    /// Splash: OverlapSphere on damageLayers, scaled damage and outward knockback,
    /// deduped against the direct hit so a centre-mass victim isn't double-counted.
    /// </summary>
    private void Explode(Vector3 epicenter, Collider directHitCollider)
    {
        if (exploded) return;
        exploded = true;

        _splashedThisExplosion.Clear();

        // Direct hit
        if (directHitCollider != null)
        {
            var directDamageable = directHitCollider.GetComponentInParent<IDamageable>();
            if (directDamageable != null)
            {
                directDamageable.TakeDamage(damage);
                _splashedThisExplosion.Add(directDamageable);
            }

            var directRb = directHitCollider.GetComponentInParent<Rigidbody>();
            if (directRb != null && directHitImpactForce > 0f)
                directRb.AddForce(transform.forward * directHitImpactForce, ForceMode.Impulse);
        }

        // Splash
        var hits = Physics.OverlapSphere(epicenter, splashRadius, damageLayers);
        foreach (var hit in hits)
        {
            var damageable = hit.GetComponentInParent<IDamageable>();
            if (damageable == null || _splashedThisExplosion.Contains(damageable)) continue;
            _splashedThisExplosion.Add(damageable);

            float dist  = Vector3.Distance(hit.transform.position, epicenter);
            float t     = Mathf.Clamp01(dist / splashRadius);
            float scale = splashFalloff.Evaluate(t);

            damageable.TakeDamage(damage * scale);

            var hitRb = hit.GetComponentInParent<Rigidbody>();
            if (hitRb != null && splashImpactForce > 0f)
            {
                Vector3 dir = (hit.transform.position - epicenter).normalized;
                hitRb.AddForce(dir * splashImpactForce * scale, ForceMode.Impulse);
            }
        }

        if (explosionPrefab != null)
            Instantiate(explosionPrefab, epicenter, Quaternion.identity);

        Destroy(gameObject);
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!drawDebug) return;
        Gizmos.color = new Color(1f, 0.5f, 0f, 0.5f);
        Gizmos.DrawWireSphere(transform.position, splashRadius);

        if (homingTarget != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, homingTarget.position);
        }
    }
#endif
}
