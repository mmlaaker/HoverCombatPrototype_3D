using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// RocketProjectile v2.0
///
/// v2.0: all tuning moved to the WeaponDefinition, and this component reads it LIVE.
///
/// There is nothing serialized on this script any more except debug flags. Speed, lifetime,
/// arming, steering, the flare, blast radius, falloff, damage layers and the explosion prefab all
/// come from the WeaponDefinition handed over at spawn via SetDefinition, and are read through
/// shorthand properties every time they are used, exactly the way the hover controllers read
/// VehicleTuningProfile. Nothing is copied into local fields, so the asset cannot drift out of
/// sync with the projectile, and editing a value during play mode retunes rockets already in
/// flight.
///
/// This replaces the older SetDamage / SetImpact push. Those pushed three values and left a dozen
/// more stranded on the prefab, which meant tuning one weapon took edits in three assets.
///
/// v1.5: detonation no longer depends on OnCollisionEnter.
///
/// It could not. Measured against static city geometry, the callback fires at 30 m/s and is
/// silent at 70, which is the speed this prefab actually ships; against a vehicle hull it is
/// already silent at 30. Above that threshold the rocket bounced off the target, drifted away
/// with gravity off, and self-detonated on `lifetime` six seconds later. The direct-hit branch
/// therefore never executed at all: every "hit" in the build was really the lifetime fuse
/// firing splash-only from wherever the rocket had come to rest, which is why `impactForce`
/// was inert while `splashImpactForce` appeared to work.
///
/// This is a tunnelling problem and no CollisionDetectionMode fixes it. Discrete, Continuous,
/// ContinuousDynamic and ContinuousSpeculative were each measured at 70 m/s and all four are
/// silent, so CCD is not doing the job it exists to do here.
///
/// The fix is to stop asking the contact system and sweep instead: every FixedUpdate,
/// SphereCast from where the rocket was to where it now is, using the collider's own radius and
/// the same layer mask the physics engine would have used. A sweep cannot tunnel, because it
/// tests the whole swept volume rather than the endpoint.
///
/// OnCollisionEnter is kept as a backstop for anything the sweep misses (a target that spawns
/// on top of the rocket, or an overlap at the sweep origin, which SphereCast does not report).
/// Both paths funnel into Explode, which is idempotent via `exploded`, so a double report is
/// harmless.
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
///   Layer:     Projectile (8), on the ROOT and every child. See the note below.
///   Damage Layers: enemy vehicle layers (and any other splash targets)
///   Explosion Prefab: a self-destructing VFX prefab spawned at impact point
///
/// On the Projectile layer, and why it still collides with vehicles:
///   An earlier version of this comment asked for "a Projectile layer that doesn't collide with
///   the firer's vehicle layer". That is the wrong shape, and it cannot work here. ProjectileSweep
///   DERIVES its mask from the collision matrix, deliberately, so the sweep and the physics engine
///   can never disagree about what is solid. Make the matrix ignore vehicles and the sweep ignores
///   them too, and projectiles fly straight through their targets.
///   So the matrix keeps Projectile colliding with Default and both vehicle layers, and the firer
///   is filtered by IDENTITY instead: the owner Transform arrives via IProjectileOwner and is
///   rejected in ProjectileSweep.TryHit and in OnCollisionEnter. Projectile ignores itself (a
///   volley must not detonate itself), Ignore Raycast, TransparentFX, Water and UI.
///   Before this, the only thing stopping a rocket detonating on its own launcher was arming
///   delay, and that protection was incidental: it held solely because speed x armingDelay
///   happened to exceed the muzzle's 0.08m clearance over the chassis.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class RocketProjectile : MonoBehaviour, IProjectileDefinitionCarrier, IHomingTarget, IProjectileOwner
{
    // -------------------------------------------------------------------------
    // Tuning
    // -------------------------------------------------------------------------
    // There is deliberately nothing to tune on this component. Every value the rocket uses
    // arrives with the WeaponDefinition handed to it at spawn (see SetDefinition) and is read
    // LIVE off that asset, exactly the way the hover controllers read VehicleTuningProfile.
    // Nothing is copied, so nothing can drift, and editing the asset mid-flight retunes rockets
    // already in the air.
    //
    // Blast geometry used to live here on the grounds that splashRadius had to match whatever
    // the explosion VFX drew. That dependency now runs the other way: the definition states the
    // radius, and the VFX is authored to match it.

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

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.IsEnabled(HoverDebugCategory.Impacts) : drawDebug;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private Rigidbody rb;
    private float age;
    private bool  exploded;
    private Transform homingTarget;

    // The vehicle that fired this, pushed at spawn by FireAllMuzzles. Null is legal
    // and simply means "nothing to exclude" (a projectile spawned by something that
    // is not a vehicle, or an older prefab wired before IProjectileOwner existed).
    private Transform owner;

    // The whole tuning surface, handed over at spawn by FireAllMuzzles. Read live, never copied.
    private WeaponDefinition def;

    // Shorthands, mirroring the `F => profile.foundation` convention the hover controllers use.
    private WeaponCombatTuning C  => def.combat;
    private WeaponImpactTuning I  => def.impact;
    private WeaponFlightTuning FL => def.flight;
    private WeaponHomingTuning H  => def.homing;
    private WeaponBlastTuning  B  => def.blast;

    // Swept hit detection. See the class header for why this exists instead of relying on
    // OnCollisionEnter. sweepFrom is the position at the end of the previous physics step;
    // sweepArmed gates the first sweep so we never test back across the muzzle.
    private Vector3 sweepFrom;
    private int     sweepMask;
    private float   sweepRadius;
    private bool    sweepArmed;

    // Flight shape. The flare works by chasing an aim point that starts offset to one side of
    // the target and slides onto it, so these are captured once when homing begins and the
    // curve stays in a stable plane instead of re-deciding itself every frame.
    private Vector3 launchForward;
    private Vector3 flareOffsetDir;
    private float   flareBaseRange;
    private float   flareStartAge;
    private bool    flareReady;

    // Drives MissileFlareMode.Alternate. Static so consecutive shots from any launcher split
    // in opposite directions, which is the whole point of alternating.
    private static int _alternateCounter;

    // Reused per-explosion to dedupe IDamageable hits across a vehicle's
    // multiple child colliders. Static is safe: Explode is fully synchronous.
    private static readonly HashSet<IDamageable> _splashedThisExplosion = new();

    /// <summary>
    /// Receives the whole tuning asset from the firing weapon. Called by FireAllMuzzles before
    /// the projectile's first tick, and the only thing this projectile needs to function.
    /// </summary>
    public void SetDefinition(WeaponDefinition definition) => def = definition;

    /// <summary>Receives the homing target from the spawning weapon. Null = dumbfire.</summary>
    public void SetTarget(Transform target) => homingTarget = target;

    /// <summary>
    /// Receives the firing vehicle's root. Used only to give the firer its own case
    /// in the splash loop: no self-damage ever, and a separately scaled shove.
    /// </summary>
    public void SetOwner(Transform owner) => this.owner = owner;

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
        // Fail loud, like the hover controllers do on a missing profile. They disable themselves;
        // a disabled projectile would just hang in mid-air, so this one leaves instead.
        if (def == null)
        {
            Debug.LogError($"[RocketProjectile] '{name}': no WeaponDefinition. It must be supplied " +
                           "via SetDefinition at spawn (FireAllMuzzles does this). Destroying.", this);
            Destroy(gameObject);
            return;
        }

        // Fire-and-forget: set velocity once; no further force needed.
        // Rigidbody must have gravity off so the rocket stays level.
        rb.linearVelocity = transform.forward * FL.speed;
        sweepFrom     = rb.position;
        launchForward = transform.forward;
    }

    /// <summary>
    /// Picks which side this shot swings out to, as a unit vector perpendicular to the launch
    /// heading. Called once, at the moment homing begins.
    /// </summary>
    private Vector3 ComputeFlareOffsetDir()
    {
        float roll;
        switch (H.flareDirection)
        {
            case MissileFlareMode.Right:  roll = 90f;  break;
            case MissileFlareMode.Left:   roll = 270f; break;
            case MissileFlareMode.Up:     roll = 0f;   break;
            case MissileFlareMode.Down:   roll = 180f; break;
            case MissileFlareMode.Random: roll = Random.Range(0f, 360f); break;
            default:                      roll = (_alternateCounter++ % 2 == 0) ? 90f : 270f; break;
        }

        float r = roll * Mathf.Deg2Rad;
        return (transform.right * Mathf.Sin(r) + transform.up * Mathf.Cos(r)).normalized;
    }

    /// <summary>
    /// Straight, then home onto a moving aim point.
    ///
    /// The flare is not a separate flight phase and deliberately so. An earlier version flew a
    /// fixed "wrong" heading open-loop for a fixed number of degrees, which looked great and
    /// missed constantly: it ignored the target completely while flaring, and it applied the
    /// same detour whether the target was 20m or 200m away, so close shots sailed straight past.
    ///
    /// Instead the missile always homes, but early on it homes at a point offset to one side of
    /// the target, and that offset shrinks to zero over flareDuration. Because the aim point
    /// *ends* on the target, convergence is guaranteed by construction rather than by tuning,
    /// and the curve comes for free. Scaling the offset by range makes it self-adjusting.
    ///
    /// Runs on the physics tick, so the path is identical at any frame rate and the turn the
    /// missile commits to is the same turn the sweep below tests.
    /// </summary>
    private void Steer(float dt)
    {
        // Phase 1: straight out of the tube, no steering at all.
        if (age < H.homingDelay)
        {
            rb.linearVelocity = transform.forward * FL.speed;
            return;
        }

        // Dumbfire, or homing with no target: hold current heading.
        if (homingTarget == null || H.turnRate <= 0f) return;

        // Captured on the first steering tick rather than at spawn, because the firing weapon
        // supplies the target after Instantiate and a late SetTarget would otherwise measure
        // its range against nothing.
        if (!flareReady)
        {
            flareBaseRange = Vector3.Distance(transform.position, homingTarget.position);
            flareOffsetDir = ComputeFlareOffsetDir();
            flareStartAge  = age;
            flareReady     = true;
        }

        Vector3 aimPoint = homingTarget.position;

        if (H.flareOffset > 0f && H.flareDuration > 0f)
        {
            float t = Mathf.Clamp01((age - flareStartAge) / H.flareDuration);

            // SmoothStep rather than a straight lerp: the offset holds early, which is what
            // establishes a visible arc, then eases out instead of snapping onto the target.
            float blend = 1f - Mathf.SmoothStep(0f, 1f, t);
            aimPoint += flareOffsetDir * (H.flareOffset * flareBaseRange * blend);
        }

        Vector3 toAim = aimPoint - transform.position;
        if (toAim.sqrMagnitude < 1e-6f) return;

        transform.rotation = Quaternion.RotateTowards(
            transform.rotation, Quaternion.LookRotation(toAim.normalized), H.turnRate * dt);

        rb.linearVelocity = transform.forward * FL.speed;
    }

    /// <summary>
    /// Owns the whole tick: age, steering, hit detection, then the lifetime fuse.
    ///
    /// The order matters. Steer sets the velocity this step will actually be integrated with,
    /// so the sweep's one-step lookahead has to be computed after it, or the missile tests a
    /// path it is no longer on.
    ///
    /// This replaces the old Update-based steering. Flight now advances on the physics tick, so
    /// the path is identical at any frame rate and the turn the missile commits to is the same
    /// turn the sweep tests. Steering in Update while colliding in FixedUpdate is exactly the
    /// timestep mismatch the architecture notes call out as a jitter source.
    /// </summary>
    private void FixedUpdate()
    {
        if (exploded) return;

        float dt = Time.fixedDeltaTime;
        age += dt;

        Steer(dt);

        Vector3 now = rb.position;

        // Stay disarmed through armingDelay so the rocket cannot detonate on the firer's own
        // hull at the muzzle, and so the first sweep does not span the spawn point.
        if (age < FL.armingDelay)
        {
            sweepFrom  = now;
            sweepArmed = true;
            return;
        }

        if (!sweepArmed) { sweepFrom = now; sweepArmed = true; return; }

        // Lookahead is this step's displacement. FixedUpdate runs before the solver, so testing
        // it here detonates on approach instead of after the solver has deflected the rocket.
        Vector3 step = rb.linearVelocity * dt;

        if (ProjectileSweep.TryHit(transform, owner, sweepFrom, now, step, sweepRadius, sweepMask,
                                   out Vector3 point, out Collider col))
        {
            Explode(point, col);
            return;
        }

        sweepFrom = now;

        if (age >= FL.lifetime)
            Explode(transform.position, null);
    }

    /// <summary>
    /// Backstop only. At this prefab's shipped speed this never fires (see the class header);
    /// the sweep in FixedUpdate is the real detection. Kept because the sweep cannot see a
    /// collider already overlapping its origin, and because slow-moving variants are still
    /// legitimately served by the contact system. Explode is idempotent, so if both report the
    /// same hit only the first one does anything.
    /// </summary>
    private void OnCollisionEnter(Collision col)
    {
        if (exploded || age < FL.armingDelay)
            return;

        // Same owner filter the sweep applies. The Projectile layer collides with both vehicle
        // layers on purpose (see ProjectileSweep.TryHit), so the firer's own hull can reach this
        // callback, and detonating on it would be exactly the failure the layer split exists to
        // prevent.
        if (owner != null && col.collider.transform.IsChildOf(owner))
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
        if (debug) WeaponDebugDraw.Blast(epicenter, B.splashRadius);

        // The firer, resolved once. Reference comparison is enough: every collider on
        // a vehicle resolves through GetComponentInParent to the same VehicleHealth
        // instance, which is the same one this finds from the root.
        IDamageable ownerDamageable = owner != null
            ? owner.GetComponentInChildren<IDamageable>()
            : null;

        // Direct hit
        if (directHitCollider != null)
        {
            var directDamageable = directHitCollider.GetComponentInParent<IDamageable>();
            bool directIsOwner   = directDamageable != null && directDamageable == ownerDamageable;

            if (directDamageable != null)
            {
                // Unreachable in practice at the shipped arming delay, which keeps the
                // rocket from detonating inside its own launcher. Guarded anyway so the
                // "you never damage yourself" rule holds on every path, not just splash.
                if (!directIsOwner)
                    directDamageable.TakeDamage(C.damage);

                _splashedThisExplosion.Add(directDamageable);
            }

            var directRb = directHitCollider.GetComponentInParent<Rigidbody>();
            float directForce = directIsOwner ? I.impactForce * I.selfImpactScale : I.impactForce;

            WeaponImpact.Apply(directRb, transform.forward, epicenter,
                               directForce, I.destabilizeFraction, 1f, debug);
        }

        // Splash
        var hits = Physics.OverlapSphere(epicenter, B.splashRadius, B.damageLayers);
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
            float t     = Mathf.Clamp01(dist / B.splashRadius);
            float scale = B.splashFalloff.Evaluate(t);

            // The firer gets its own case. Never damaged by its own blast; shoved by a
            // separately authored fraction, which is what makes the rocket jump a dial
            // rather than an accident. selfImpactScale 0 excludes it outright, since
            // WeaponImpact.Apply early-outs on a zero magnitude.
            bool isOwner = damageable == ownerDamageable;

            if (!isOwner)
                damageable.TakeDamage(C.damage * scale);

            // The measured-from point is what silently broke before: children of a compound
            // collider all resolve to one IDamageable, so whichever collider OverlapSphere
            // returned first used to decide both falloff and push direction.
            if (debug) WeaponDebugDraw.SplashVictim(epicenter, reference, scale);

            float splashForce = isOwner
                ? I.splashImpactForce * I.selfImpactScale
                : I.splashImpactForce;

            if (logSplash)
                Debug.Log($"[Splash] {hit.transform.root.name} via '{hit.name}'{(isOwner ? " [SELF]" : "")}  " +
                          $"dist={dist:F2}m / {B.splashRadius:F1}m  falloff={scale:F3}  " +
                          $"damage={(isOwner ? 0f : C.damage * scale):F1}  force={splashForce * scale:F0}", this);

            if (hitRb != null && splashForce > 0f)
            {
                // Applied at the surface nearest the blast, so a blast off one flank shoves
                // that flank. ClosestPointOnBounds rather than Collider.ClosestPoint: the
                // latter returns the input unchanged for non-convex mesh colliders, which
                // these vehicles use.
                Vector3 dir = reference - epicenter;
                WeaponImpact.Apply(hitRb, dir, hitRb.ClosestPointOnBounds(epicenter),
                                   splashForce, I.destabilizeFraction, scale, debug);
            }
        }

        if (B.explosionPrefab != null)
            Instantiate(B.explosionPrefab, epicenter, Quaternion.identity);

        Destroy(gameObject);
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!ShouldDrawDebug) return;

        // Guarded independently of Start: gizmos run on the prefab asset and in edit mode, where
        // no weapon has handed this projectile a definition yet. Same discipline the hover
        // controllers apply to every path that can execute before Awake.
        if (def == null) return;

        Gizmos.color = new Color(1f, 0.5f, 0f, 0.5f);
        Gizmos.DrawWireSphere(transform.position, B.splashRadius);

        if (homingTarget != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, homingTarget.position);
        }
    }
#endif
}
