using UnityEngine;

/// <summary>
/// ProjectileSweep v1.0
/// --------------------
/// The single implementation of "did this projectile pass through anything since the last
/// physics step". Sibling of <see cref="WeaponImpact"/>: that one owns what a hit does, this
/// one owns whether a hit happened.
///
/// Why this exists instead of OnCollisionEnter
/// -------------------------------------------
/// Measured in this project, the contact callback is speed-gated and both projectiles ship
/// above the gate:
///
///   EMP     (SphereCollider  r 0.5  ) fires at 45 m/s, silent at 50. Ships at 55.
///   Missile (CapsuleCollider r 0.165) fires at 30 m/s, silent at 70. Ships at 70.
///
/// Against a vehicle hull, whose colliders are convex MeshColliders on a dynamic Rigidbody,
/// the threshold is lower still. Above it the projectile bounces off, drifts, and self-detonates
/// on its lifetime timer somewhere harmless, so the direct-hit branch never runs.
///
/// This is not a CollisionDetectionMode problem. Discrete, Continuous, ContinuousDynamic and
/// ContinuousSpeculative were each measured at 70 m/s and all four are silent, which means CCD
/// is not doing the job it exists to do. It is also not global: a plain cube with a Rigidbody,
/// dropped on the same floor, receives its callback normally. Only fast movers are affected.
///
/// A sweep sidesteps the whole question. It tests the swept volume between two positions rather
/// than the endpoint, so it cannot tunnel regardless of speed or timestep.
///
/// Known limitation, deliberately accepted: SphereCast does not report colliders that already
/// overlap the sweep origin. Callers keep their OnCollisionEnter as a backstop for that case.
/// </summary>
public static class ProjectileSweep
{
    // Reused every sweep so this allocates nothing per projectile per FixedUpdate. Static is
    // safe for the same reason RocketProjectile's splash set is: a sweep is fully synchronous
    // and the results are consumed before the next call can begin.
    private static readonly RaycastHit[] _hits     = new RaycastHit[16];
    private static readonly Collider[]   _overlaps = new Collider[16];

    /// <summary>
    /// Derives the sweep radius and layer mask from the projectile's own collider, so the sweep
    /// hits exactly what the physics engine would have hit and nothing else. Call once in Awake.
    /// </summary>
    /// <param name="col">The projectile's collider. May sit on a child of the Rigidbody.</param>
    public static void Configure(Collider col, out int mask, out float radius)
    {
        if (col == null)
        {
            // No collider is a real setup error, but returning a usable pair keeps the caller
            // from NaN-ing its way through every FixedUpdate.
            mask   = Physics.AllLayers;
            radius = 0.1f;
            Debug.LogError("[ProjectileSweep] No collider found. Sweep will use fallback values.");
            return;
        }

        mask = BuildCollisionMask(col.gameObject.layer);

        // Inflated by twice the contact offset, and this is load-bearing. The solver holds
        // bodies Physics.defaultContactOffset apart, so a projectile it has already stopped is
        // resting that far clear of the surface it is "touching". A sweep or overlap at the
        // collider's exact radius therefore misses by precisely that gap, every time, and the
        // projectile sits there un-detonated. The first version of this fix failed for exactly
        // this reason. The cost is detonating a few centimetres early, which is nothing against
        // a blast radius measured in metres.
        radius = DeriveRadius(col) + Mathf.Max(Physics.defaultContactOffset * 2f, 0.01f);
    }

    /// <summary>
    /// Tests the volume the projectile occupied last step, plus the volume it is about to move
    /// through this step, and reports the nearest hit that is not part of the projectile.
    ///
    /// The lookahead is not optional, it is the whole point. FixedUpdate runs *before* the
    /// physics step, so a purely retrospective sweep is always one step behind the solver: by
    /// the time it tests the segment that reached a wall, the solver has already stopped the
    /// projectile at that wall, and it stopped it `Physics.defaultContactOffset` short of the
    /// surface. The retrospective segment therefore ends just before the thing it hit and
    /// reports nothing, forever. Sweeping one step *forward* instead detonates on approach,
    /// before the solver ever gets to deflect it.
    /// </summary>
    /// <param name="self">Projectile root, used to reject its own colliders.</param>
    /// <param name="owner">
    /// Firing vehicle's root, rejected the same way as <paramref name="self"/>. May be null.
    ///
    /// Load-bearing now that projectiles sit on their own layer. That layer deliberately still
    /// COLLIDES with both vehicle layers, because the mask this sweep uses is derived from the
    /// collision matrix and making it ignore vehicles would mean projectiles pass straight through
    /// their targets. So the matrix keeps direct hits working and the owner is filtered here
    /// instead, which is the only place that can tell the shooter apart from a legitimate target.
    ///
    /// Without this the arming delay is the sole protection against detonating on your own hull,
    /// and that protection is incidental: it holds only while speed x armingDelay happens to
    /// exceed the muzzle's clearance over the chassis. Lower a missile's speed and it evaporates.
    /// </param>
    /// <param name="from">Position at the end of the previous physics step.</param>
    /// <param name="current">Position now, before this step is simulated.</param>
    /// <param name="step">Displacement this step will apply, ie. velocity * fixedDeltaTime.</param>
    /// <returns>True if something was hit, with the contact point and collider.</returns>
    public static bool TryHit(Transform self, Transform owner, Vector3 from, Vector3 current, Vector3 step,
                              float radius, int mask, out Vector3 point, out Collider collider)
    {
        point    = current;
        collider = null;

        Vector3 to    = current + step;
        Vector3 delta = to - from;
        float distance = delta.magnitude;

        if (distance > 1e-5f)
        {
            Vector3 dir = delta / distance;

            // Non-alloc, because this runs every FixedUpdate on every projectile in flight.
            // QueryTriggerInteraction.Ignore: triggers are volumes, not geometry, and a rocket
            // should fly through a checkpoint rather than detonate on it.
            int count = Physics.SphereCastNonAlloc(from, radius, dir, _hits, distance, mask,
                                                   QueryTriggerInteraction.Ignore);

            float best = float.MaxValue;
            for (int i = 0; i < count; i++)
            {
                var hit = _hits[i];
                if (hit.collider == null) continue;
                if (hit.collider.transform.IsChildOf(self)) continue;
                if (owner != null && hit.collider.transform.IsChildOf(owner)) continue;

                if (hit.distance < best)
                {
                    best     = hit.distance;
                    collider = hit.collider;

                    // distance 0 means the cast began already touching this collider, in which
                    // case hit.point is not meaningful and comes back as zero.
                    point = hit.distance > 0f ? hit.point : hit.collider.ClosestPoint(from);
                }
            }

            if (collider != null) return true;
        }

        // Resting or already-overlapping contact. SphereCast never reports a collider that
        // overlaps its origin, which is precisely the state a projectile ends up in if the
        // solver managed to stop it against a surface before the sweep fired. Without this the
        // projectile sits embedded in its target until its lifetime fuse expires, which is the
        // exact behaviour this class was written to eliminate.
        int overlaps = Physics.OverlapSphereNonAlloc(current, radius, _overlaps, mask,
                                                     QueryTriggerInteraction.Ignore);
        for (int i = 0; i < overlaps; i++)
        {
            var c = _overlaps[i];
            if (c == null) continue;
            if (c.transform.IsChildOf(self)) continue;
            if (owner != null && c.transform.IsChildOf(owner)) continue;

            collider = c;
            point    = c.ClosestPoint(current);
            return true;
        }

        return false;
    }

    // -------------------------------------------------------------------------
    // Internals
    // -------------------------------------------------------------------------

    /// <summary>
    /// Every layer the given layer is allowed to collide with, straight from the project's
    /// collision matrix. Reproducing the matrix rather than hand-authoring a mask means the
    /// sweep and the physics engine can never disagree about what is solid.
    /// </summary>
    private static int BuildCollisionMask(int layer)
    {
        int mask = 0;
        for (int i = 0; i < 32; i++)
            if (!Physics.GetIgnoreLayerCollision(layer, i))
                mask |= 1 << i;
        return mask;
    }

    /// <summary>
    /// Sweep radius matching the collider's real world-space size. Scale matters here: the
    /// missile's capsule is authored at radius 0.5 but sits on a node scaled to 0.33, so its
    /// actual radius is 0.165 and a sweep at 0.5 would detonate on near misses.
    /// </summary>
    private static float DeriveRadius(Collider col)
    {
        Vector3 s = col.transform.lossyScale;

        if (col is SphereCollider sphere)
            return sphere.radius * Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.y), Mathf.Abs(s.z));

        if (col is CapsuleCollider capsule)
        {
            // The radius scales with the two axes perpendicular to the capsule's own direction.
            float a, b;
            switch (capsule.direction)
            {
                case 0:  a = Mathf.Abs(s.y); b = Mathf.Abs(s.z); break;  // X
                case 1:  a = Mathf.Abs(s.x); b = Mathf.Abs(s.z); break;  // Y
                default: a = Mathf.Abs(s.x); b = Mathf.Abs(s.y); break;  // Z
            }
            return capsule.radius * Mathf.Max(a, b);
        }

        // Anything else: the smallest half-extent is the conservative choice, since an
        // over-wide sweep detonates on near misses and that reads as a bug to a player.
        Vector3 e = col.bounds.extents;
        return Mathf.Max(0.01f, Mathf.Min(e.x, Mathf.Min(e.y, e.z)));
    }
}
