using UnityEngine;

/// <summary>
/// EmpProjectile v1.2
///
/// v1.2: flight moved from Update to FixedUpdate, closing the same timestep mismatch
/// RocketProjectile v1.6 closed. v1.1 moved hit detection onto the physics tick but left steering,
/// age and the lifetime fuse on the render tick. Three real consequences, stated carefully because
/// the obvious complaint about this shape is the one thing that was NOT wrong:
///
///   1. Steering ran at frame rate against a solver running at 100Hz. Total turn per second was
///      correct (RotateTowards by turnRate * deltaTime integrates to turnRate either way), but the
///      heading the solver actually integrated was only refreshed once per frame, so the shot flew
///      in frame-length straight segments and its PATH depended on frame rate. At 30fps that is
///      3.3 physics steps per heading update. This is the same reason RocketProjectile v1.6 moved:
///      the turn the projectile commits to should be the turn the sweep tests.
///   2. transform.rotation was written from Update on a Rigidbody, mixing direct transform writes
///      into a body the solver owns.
///   3. The arming gate compared a frame-quantised age against physics-rate steps. During arming,
///      every physics step in a frame saw the same stale age and each one reset sweepFrom,
///      discarding sweep history; arming could also land up to a frame late, which at 55 m/s and
///      30fps is 1.8m of uncovered travel.
///
///   NOT a defect, contrary to first reading: age itself. age += Time.deltaTime in Update
///   accumulates to correct wall-clock time regardless of frame rate, so armingDelay and lifetime
///   fired at the right MOMENT. Only their granularity relative to the physics tick was wrong.
///
/// FixedUpdate now owns age, steering, hit detection and the fuse in that order, and the order is
/// load-bearing: the sweep's lookahead must be computed after steering has set the velocity.
///
/// v1.1: hit detection no longer depends on OnCollisionEnter, which this projectile is too fast
/// to receive. Measured, the callback fires at 45 m/s and is silent at 50; `speed` ships at 55,
/// so the EMP bounced off its target and expired on `lifetime` without ever applying a freeze.
/// Detection is now a swept SphereCast in FixedUpdate via the shared <see cref="ProjectileSweep"/>.
/// See that class for the full measurement and for why no CollisionDetectionMode fixes it.
/// OnCollisionEnter is retained as a backstop; Consume is idempotent.
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
///   Layer:     Projectile (8), on the ROOT and every child. It still collides with both vehicle
///              layers on purpose; the firer is rejected by identity via IProjectileOwner rather
///              than by the matrix. See the same note on RocketProjectile for why.
///   ParticleSystem child: pure VFX, no collision module needed (this script handles hits)
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class EmpProjectile : MonoBehaviour, IHomingTarget, IProjectileOwner
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
    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug. " +
             "Assign the shared HoverDebugSettings asset so one switch covers this too.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    [SerializeField] private bool drawDebug = false;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.IsEnabled(HoverDebugCategory.Impacts) : drawDebug;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private Rigidbody rb;
    private Transform homingTarget;
    private Transform owner;
    private float freezeDuration;
    private float age;
    private bool consumed;

    // Swept hit detection. See ProjectileSweep for why this replaces the contact callback.
    private Vector3 sweepFrom;
    private int     sweepMask;
    private float   sweepRadius;
    private bool    sweepArmed;

    /// <summary>Receives the homing target from the spawning ability. Null = flies straight.</summary>
    public void SetTarget(Transform target) => homingTarget = target;

    /// <summary>
    /// Receives the firing vehicle's root, so the sweep and the collision backstop can both
    /// reject it. The Projectile layer still collides with vehicle layers by design, so this is
    /// what stops the shot freezing the vehicle that fired it.
    /// </summary>
    public void SetOwner(Transform owner) => this.owner = owner;

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
        ProjectileSweep.Configure(GetComponentInChildren<Collider>(), out sweepMask, out sweepRadius);
    }

    private void Start()
    {
        rb.linearVelocity = transform.forward * speed;
        sweepFrom = rb.position;
    }

    /// <summary>
    /// Turns toward the homing target, rate-limited. No target or no turn rate means the shot
    /// holds the heading Start gave it; gravity is off, so nothing else acts on it.
    ///
    /// Runs on the physics tick so the path is identical at any frame rate, and so the turn the
    /// projectile commits to is the same turn the sweep below tests.
    /// </summary>
    private void Steer(float dt)
    {
        if (homingTarget == null || turnRate <= 0f) return;

        Vector3 toTarget = homingTarget.position - transform.position;
        if (toTarget.sqrMagnitude < 1e-6f) return;

        transform.rotation = Quaternion.RotateTowards(
            transform.rotation, Quaternion.LookRotation(toTarget.normalized), turnRate * dt);

        rb.linearVelocity = transform.forward * speed;
    }

    /// <summary>
    /// Owns the whole tick: age, steering, hit detection, then the lifetime fuse.
    ///
    /// The order matters. Steer sets the velocity this step will actually be integrated with, so
    /// the sweep's one-step lookahead has to be computed after it, or the projectile tests a path
    /// it is no longer on. Same structure as RocketProjectile.FixedUpdate, deliberately: these two
    /// are siblings and the bug this fixes was already fixed once over there.
    /// </summary>
    private void FixedUpdate()
    {
        if (consumed) return;

        float dt = Time.fixedDeltaTime;
        age += dt;

        Steer(dt);

        Vector3 now = rb.position;

        // Disarmed through armingDelay so the shot cannot trigger on the firer at the muzzle,
        // and so the first sweep does not span the spawn point.
        if (age < armingDelay)
        {
            sweepFrom  = now;
            sweepArmed = true;
            return;
        }

        if (!sweepArmed) { sweepFrom = now; sweepArmed = true; return; }

        // Lookahead is this step's displacement; see ProjectileSweep for why it is required.
        Vector3 step = rb.linearVelocity * dt;

        if (ProjectileSweep.TryHit(transform, owner, sweepFrom, now, step, sweepRadius, sweepMask,
                                   out Vector3 point, out Collider col))
        {
            Consume(point, col);
            return;
        }

        sweepFrom = now;

        if (age >= lifetime)
            Consume(transform.position, null);
    }

    /// <summary>
    /// Backstop only; the sweep in FixedUpdate is the real detection at this projectile's speed.
    /// Consume is idempotent, so a duplicate report from both paths does nothing.
    /// </summary>
    private void OnCollisionEnter(Collision col)
    {
        if (consumed || age < armingDelay)
            return;

        // Same owner filter the sweep applies; the Projectile layer collides with vehicle layers
        // on purpose, so the firer's own hull can reach this callback.
        if (owner != null && col.collider.transform.IsChildOf(owner))
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
        if (!ShouldDrawDebug) return;

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
