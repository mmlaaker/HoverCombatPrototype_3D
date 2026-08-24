using UnityEngine;

/// <summary>
/// TargetingScan v1.0
///
/// Stateless helper for forward-cone target acquisition. Used by missiles for
/// SoftHoming/HardLock target picking, and by any other weapon or ability
/// that needs to "find the best enemy in front of me right now".
///
/// The math (overlap-sphere placed half a range out, sized so its boundary
/// covers the full cone width, then per-hit angle filtered) is lifted from
/// the original inline implementation in HoverController_Weapons. Behavior
/// is preserved exactly so existing missile tuning still feels the same.
///
/// The caller owns the scan buffer to keep this allocator-free per call.
/// Reuse a single Collider[] field per consumer; size 16 is generous for
/// small vehicle rosters and matches the original missile buffer.
/// </summary>
public static class TargetingScan
{
    /// <summary>
    /// Picks the closest in-cone target from a forward-cone overlap scan.
    /// Returns the best target's Transform, or null if nothing qualifies.
    ///
    /// "Best" means smallest angle from origin.forward. Range is checked
    /// against actual distance, not the overlap sphere radius (which
    /// extends slightly beyond the cone tip).
    ///
    /// Self-skip: any hit whose root matches origin.root is ignored.
    /// </summary>
    /// <param name="origin">Firer's transform. Position and forward define the cone.</param>
    /// <param name="rangeMeters">Maximum distance to a valid target.</param>
    /// <param name="coneHalfAngleDegrees">Half-angle of the cone from origin.forward.</param>
    /// <param name="mask">Layers eligible to be targeted.</param>
    /// <param name="scanBuffer">Caller-owned overlap buffer. Reused across calls.</param>
    /// <param name="exclude">
    /// Targets already committed by a multi-lock cascade. Skipped so a second lock finds
    /// somebody NEW rather than re-acquiring whoever is nearest the nose, which would make
    /// every missile in a volley chase one target regardless of what else is in the cone.
    /// Null means no exclusions, which is the single-lock and SoftHoming case.
    /// Compared by root transform, because a scan returns whichever child collider Physics
    /// happened to hit and the same vehicle can present several.
    /// </param>
    public static Transform PickBestInCone(
        Transform origin,
        float rangeMeters,
        float coneHalfAngleDegrees,
        LayerMask mask,
        Collider[] scanBuffer,
        System.Collections.Generic.List<Transform> exclude = null)
    {
        if (origin == null || scanBuffer == null || scanBuffer.Length == 0)
            return null;

        Vector3 scanOrigin = origin.position + origin.forward * (rangeMeters * 0.5f);
        float   scanRadius = rangeMeters * Mathf.Tan(coneHalfAngleDegrees * Mathf.Deg2Rad);

        int hitCount = Physics.OverlapSphereNonAlloc(scanOrigin, scanRadius, scanBuffer, mask);

        Transform bestTarget = null;
        float     bestAngle  = coneHalfAngleDegrees + 1f;
        Transform selfRoot   = origin.root;

        for (int i = 0; i < hitCount; i++)
        {
            var hit = scanBuffer[i];
            if (hit == null) continue;
            if (hit.transform.root == selfRoot) continue;

            if (exclude != null)
            {
                bool alreadyTaken = false;
                for (int e = 0; e < exclude.Count; e++)
                    if (exclude[e] != null && exclude[e].root == hit.transform.root) { alreadyTaken = true; break; }
                if (alreadyTaken) continue;
            }

            Vector3 toTarget = hit.transform.position - origin.position;
            if (toTarget.magnitude > rangeMeters) continue;

            float angle = Vector3.Angle(origin.forward, toTarget);
            if (angle < coneHalfAngleDegrees && angle < bestAngle)
            {
                bestAngle  = angle;
                bestTarget = hit.transform;
            }
        }

        return bestTarget;
    }
}
