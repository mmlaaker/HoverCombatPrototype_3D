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
/// Damage arrives from the firing weapon via IProjectileDamageCarrier, and knockback via
/// IProjectileImpactCarrier. What stays tuned per-prefab on this script is flight (speed,
/// lifetime, arming, turn rate) and blast geometry (splashRadius, splashFalloff,
/// damageLayers), which has to match the explosion VFX.
///
/// Prefab setup:
///   Rigidbody: Use Gravity OFF, Collision Detection: Continuous Dynamic, Drag 0
///   Collider:  convex (Capsule or Box), NOT a trigger
///   Layer:     a Projectile layer that doesn't collide with the firer's vehicle layer
///   Damage Layers: enemy vehicle layers (and any other splash targets)
///   Explosion Prefab: a self-destructing VFX prefab spawned at impact point
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class RocketProjectile : MonoBehaviour, IProjectileDamageCarrier, IProjectileImpactCarrier, IHomingTarget
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

    // Knockback (directHitImpactForce, splashImpactForce, destabilizeFraction) is NOT authored
    // here. It arrives from the firing WeaponDefinition via IProjectileImpactCarrier, so the
    // three missile variants can differ in knockback without needing three prefab overrides.
    //
    // Blast geometry above (splashRadius, splashFalloff, damageLayers) is the one thing that did
    // not move, because the radius has to agree with whatever the explosion VFX draws. That is a
    // constraint of the current placeholder VFX rather than a real design boundary: when the final
    // explosions land, this should follow the forces onto the WeaponDefinition and leave the prefab
    // as pure delivery.

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
    [Tooltip("Shared debug asset. When assigned, its master toggle overrides the local flag below, " +
             "so one switch controls every debug visual in the project. Leave empty to use the " +
             "local flag.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    [Tooltip("Draw the splash radius in flight, plus the blast sphere, splash falloff per victim, " +
             "and the impulse split at each impact. Impact draws persist for a few seconds so you " +
             "can inspect them after the rocket is gone.")]
    [SerializeField] private bool drawDebug = true;

    [Tooltip("Also log one line per splash victim with the exact distance, falloff scale, and " +
             "damage dealt. The Scene view draws show the shape of a blast; this gives the numbers " +
             "when you need to compare two shots precisely.")]
    [SerializeField] private bool logSplash = false;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.enableDebugGizmos : drawDebug;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private Rigidbody rb;
    private float damage;
    private float age;
    private bool  exploded;
    private Transform homingTarget;

    // Supplied by the spawning WeaponDefinition via IProjectileImpactCarrier. Default to 0 so a
    // prefab dropped straight into a scene is inert rather than silently using stale tuning.
    private float directHitImpactForce;
    private float splashImpactForce;
    private float destabilizeFraction;

    // Reused per-explosion to dedupe IDamageable hits across a vehicle's
    // multiple child colliders. Static is safe: Explode is fully synchronous.
    private static readonly HashSet<IDamageable> _splashedThisExplosion = new();

    /// <summary>Receives the damage value from the spawning WeaponDefinition.</summary>
    public void SetDamage(float dmg) => damage = dmg;

    /// <summary>Receives knockback values from the spawning WeaponDefinition.</summary>
    public void SetImpact(float directHitForce, float splashForce, float destabilize)
    {
        directHitImpactForce = directHitForce;
        splashImpactForce    = splashForce;
        destabilizeFraction  = destabilize;
    }

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

        // Cached: ShouldDrawDebug walks a null check per call and Explode is a hot burst.
        bool debug = ShouldDrawDebug;
        if (debug) WeaponDebugDraw.Blast(epicenter, splashRadius);

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
            WeaponImpact.Apply(directRb, transform.forward, epicenter,
                               directHitImpactForce, destabilizeFraction, 1f, debug);
        }

        // Splash
        var hits = Physics.OverlapSphere(epicenter, splashRadius, damageLayers);
        foreach (var hit in hits)
        {
            var damageable = hit.GetComponentInParent<IDamageable>();
            if (damageable == null || _splashedThisExplosion.Contains(damageable)) continue;
            _splashedThisExplosion.Add(damageable);

            var hitRb = hit.GetComponentInParent<Rigidbody>();

            // Measure from the centre of mass, not the collider's own transform. A vehicle
            // is a compound collider whose children all resolve to one IDamageable, so only
            // the first collider OverlapSphere happens to return survives the dedupe above.
            // Keying off that collider's transform made both the falloff scale and the push
            // direction depend on iteration order: the wheel rims sit ~2.9m apart, which
            // swung splash damage and force by 3x between otherwise identical shots.
            Vector3 reference = hitRb != null ? hitRb.worldCenterOfMass : hit.transform.position;

            float dist  = Vector3.Distance(reference, epicenter);
            float t     = Mathf.Clamp01(dist / splashRadius);
            float scale = splashFalloff.Evaluate(t);

            damageable.TakeDamage(damage * scale);

            // The measured-from point is what silently broke before: children of a compound
            // collider all resolve to one IDamageable, so whichever collider OverlapSphere
            // returned first used to decide both falloff and push direction.
            if (debug) WeaponDebugDraw.SplashVictim(epicenter, reference, scale);

            if (logSplash)
                Debug.Log($"[Splash] {hit.transform.root.name} via '{hit.name}'  " +
                          $"dist={dist:F2}m / {splashRadius:F1}m  falloff={scale:F3}  " +
                          $"damage={damage * scale:F1}  force={splashImpactForce * scale:F0}", this);

            if (hitRb != null && splashImpactForce > 0f)
            {
                // Applied at the surface nearest the blast, so a blast off one flank shoves
                // that flank. ClosestPointOnBounds rather than Collider.ClosestPoint: the
                // latter returns the input unchanged for non-convex mesh colliders, which
                // these vehicles use.
                Vector3 dir = reference - epicenter;
                WeaponImpact.Apply(hitRb, dir, hitRb.ClosestPointOnBounds(epicenter),
                                   splashImpactForce, destabilizeFraction, scale, debug);
            }
        }

        if (explosionPrefab != null)
            Instantiate(explosionPrefab, epicenter, Quaternion.identity);

        Destroy(gameObject);
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!ShouldDrawDebug) return;
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
