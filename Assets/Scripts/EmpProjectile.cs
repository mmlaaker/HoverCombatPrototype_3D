using UnityEngine;

/// <summary>
/// EmpProjectile v1.0
///
/// Soft-homing single-shot projectile for the EMP ability. Flies at constant speed,
/// optionally bends toward a target supplied via IHomingTarget.SetTarget, and applies
/// an EMP freeze to whichever vehicle it hits.
///
/// Mechanically a stripped-down RocketProjectile: no splash, no damage, no explosion.
/// The visual is whatever ParticleSystem the prefab parents under itself
/// (electrical sparks / lightning trail).
///
/// Freeze duration is supplied by the spawning HoverController_EMP via SetFreezeDuration,
/// so per-vehicle EmpTuning drives the value rather than the prefab.
///
/// Prefab setup:
///   Rigidbody: Use Gravity OFF, Collision Detection: Continuous Dynamic, Drag 0
///   Collider:  convex (Capsule or Sphere), NOT a trigger
///   Layer:     a Projectile layer that doesn't collide with the firer's vehicle layer
///   ParticleSystem child: pure VFX, no collision module needed (this script handles hits)
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class EmpProjectile : MonoBehaviour, IHomingTarget
{
    // -------------------------------------------------------------------------
    // ✈️ Flight
    // -------------------------------------------------------------------------
    [Header("✈️ Flight")]
    [Tooltip("How fast the projectile travels in metres per second.")]
    [SerializeField] private float speed = 55f;

    [Tooltip("Maximum time the projectile can fly before self-destructing without effect. " +
             "Prevents stray shots from drifting forever.")]
    [SerializeField] private float lifetime = 5f;

    [Tooltip("Short delay after spawn during which collisions are ignored. " +
             "Prevents the projectile from triggering on the firer's own collider at the muzzle.")]
    [SerializeField] private float armingDelay = 0.05f;

    // -------------------------------------------------------------------------
    // 🎯 Homing
    // -------------------------------------------------------------------------
    [Header("🎯 Homing")]
    [Tooltip("Maximum rotation toward the homing target in degrees per second. " +
             "Only applies when a target is supplied via IHomingTarget.SetTarget. " +
             "0 disables steering. 60 to 120 feels soft; 180+ feels aggressive.")]
    [Min(0f)]
    [SerializeField] private float turnRate = 110f;

    // -------------------------------------------------------------------------
    // 💥 Impact
    // -------------------------------------------------------------------------
    [Header("💥 Impact")]
    [Tooltip("Optional knockback applied to a directly hit Rigidbody, along the flight direction. " +
             "0 = pure freeze, no physical jolt. Keep small to honor the 'tempo, not damage' contract.")]
    [SerializeField] private float impactForce = 0f;

    // -------------------------------------------------------------------------
    // 🛠 Debug
    // -------------------------------------------------------------------------
    [Header("🛠 Debug")]
    [SerializeField] private bool drawDebug = false;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private Rigidbody rb;
    private Transform homingTarget;
    private float freezeDuration;
    private float age;
    private bool consumed;

    /// <summary>Receives the homing target from the spawning ability. Null = flies straight.</summary>
    public void SetTarget(Transform target) => homingTarget = target;

    /// <summary>
    /// Receives the freeze duration from HoverController_EMP. Called once after Instantiate,
    /// before the first Update. Without this call the projectile applies a zero-duration freeze
    /// (effectively a no-op on hit).
    /// </summary>
    public void SetFreezeDuration(float duration) => freezeDuration = duration;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void Start()
    {
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

        if (!consumed && age >= lifetime)
            Consume(transform.position, null);
    }

    private void OnCollisionEnter(Collision col)
    {
        if (consumed || age < armingDelay)
            return;

        Vector3 hitPoint = col.contactCount > 0 ? col.GetContact(0).point : transform.position;
        Consume(hitPoint, col.collider);
    }

    // -------------------------------------------------------------------------
    // Hit resolution
    // -------------------------------------------------------------------------
    private void Consume(Vector3 hitPoint, Collider hitCollider)
    {
        if (consumed) return;
        consumed = true;

        if (hitCollider != null)
        {
            var shield = hitCollider.GetComponentInParent<HoverController_Shield>();
            bool absorbed = shield != null && shield.TryAbsorbEmp();

            if (!absorbed)
            {
                var energy = hitCollider.GetComponentInParent<HoverController_Energy>();
                if (energy != null && freezeDuration > 0f)
                    energy.ApplyEmpFreeze(freezeDuration);

                if (impactForce > 0f)
                {
                    var hitRb = hitCollider.GetComponentInParent<Rigidbody>();
                    if (hitRb != null)
                        hitRb.AddForce(transform.forward * impactForce, ForceMode.Impulse);
                }
            }
        }

        Destroy(gameObject);
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!drawDebug) return;

        Gizmos.color = new Color(0.4f, 0.9f, 1f, 0.8f);
        Gizmos.DrawWireSphere(transform.position, 0.4f);

        if (homingTarget != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.position, homingTarget.position);
        }
    }
#endif
}
