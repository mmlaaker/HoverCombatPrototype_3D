using UnityEngine;

/// <summary>
/// WeaponImpact v1.0
/// -----------------
/// The single implementation of "apply a knockback impulse, split between the contact point
/// and the centre of mass". Every delivery path calls this, so a particle hit and a rocket
/// hit destabilize identically for the same destabilizeFraction.
///
/// AddForce alone applies at the centre of mass by definition, so the lever arm is zero and
/// no impact can produce rotation regardless of magnitude. Applying the whole impulse at the
/// contact point instead does rotate, but ties rotation to the lever arm: the force that
/// gives a satisfying punt spins the target several full turns and saturates
/// maxAngularVelocity, which flattens physically distinct hits into one behaviour.
///
/// Splitting decouples the two. The off-centre share generates torque through its real lever
/// arm, so the rotation axis stays physical and hit location still determines the response;
/// the centre-of-mass share carries the remaining knockback, so tuning rotation never changes
/// how far the target is punted.
/// </summary>
public static class WeaponImpact
{
    /// <param name="body">Target rigidbody. Null is a no-op.</param>
    /// <param name="direction">Push direction, normalized internally. A zero vector is a no-op.</param>
    /// <param name="worldPoint">World-space contact point the off-centre share is applied at.</param>
    /// <param name="force">Impulse magnitude before scaling.</param>
    /// <param name="destabilizeFraction">0..1 share applied at worldPoint rather than the centre of mass.</param>
    /// <param name="scale">Falloff multiplier. 1 for a direct hit.</param>
    /// <param name="drawDebug">Draw the impulse split in the Scene view. Callers pass their own
    /// debug flag; the draw itself compiles out of player builds.</param>
    public static void Apply(Rigidbody body, Vector3 direction, Vector3 worldPoint,
                             float force, float destabilizeFraction, float scale = 1f,
                             bool drawDebug = false)
    {
        if (body == null) return;

        float magnitude = force * scale;
        if (magnitude <= 0f) return;

        // A blast centred exactly on the centre of mass leaves no outward direction to push
        // along; normalizing a zero vector would hand Unity a NaN impulse.
        if (direction.sqrMagnitude < 1e-6f) return;

        Vector3 impulse  = direction.normalized * magnitude;
        float   fraction = Mathf.Clamp01(destabilizeFraction);

        // Drawn here rather than at the call sites so every delivery path is instrumented by
        // the same code that decides the split.
        if (drawDebug)
            WeaponDebugDraw.Impact(body, worldPoint, direction, fraction);

        if (fraction <= 0f)
        {
            body.AddForce(impulse, ForceMode.Impulse);
            return;
        }

        body.AddForceAtPosition(impulse * fraction, worldPoint, ForceMode.Impulse);

        if (fraction < 1f)
            body.AddForce(impulse * (1f - fraction), ForceMode.Impulse);
    }
}
