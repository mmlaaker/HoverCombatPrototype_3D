using System.Diagnostics;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>
/// WeaponDebugDraw v1.0
/// --------------------
/// Scene-view draws for the impact half of the weapon system. The targeting half is already
/// covered by gizmos on HoverController_Weapons; this covers what happens after the hit.
///
/// Everything here uses Debug.DrawLine with a duration rather than OnDrawGizmos, for two
/// reasons: impacts are instantaneous and need to linger to be readable, and the rocket
/// destroys itself at the end of Explode, so it has no OnDrawGizmos left to run.
///
/// Every method is [Conditional("UNITY_EDITOR")], so calls and their arguments compile out
/// of player builds entirely. Callers still gate on their own debug flag to keep the Scene
/// view quiet during normal play.
///
/// Text labels are deliberately absent: Handles.Label only works from OnDrawGizmos/OnGUI,
/// and these are called from FixedUpdate-time collision code. Magnitudes are encoded as
/// colour and length instead, with RocketProjectile.logSplash available when exact numbers
/// are needed.
/// </summary>
public static class WeaponDebugDraw
{
    /// <summary>Seconds impact draws stay on screen.</summary>
    public static float Lifetime = 4f;

    /// <summary>World length of an arrow representing the full impulse. Impulses run into the
    /// tens of thousands, so they are drawn as a share of this rather than to scale.</summary>
    public static float ImpulseVisualLength = 4f;

    private static readonly Color ContactColor    = new Color(1f, 1f, 1f, 1f);
    private static readonly Color LeverArmColor   = new Color(1f, 0f, 1f, 1f);
    private static readonly Color CenterMassColor = new Color(0f, 1f, 1f, 1f);
    private static readonly Color SpinShareColor  = new Color(1f, 0.25f, 0.1f, 1f);
    private static readonly Color ShoveShareColor = new Color(0.2f, 0.5f, 1f, 1f);
    private static readonly Color BlastColor      = new Color(1f, 0.5f, 0f, 1f);

    // -------------------------------------------------------------------------
    // 1. Impulse split
    // -------------------------------------------------------------------------
    /// <summary>
    /// Draws the mechanism behind destabilizeFraction: where the hit landed, where the centre
    /// of mass is, the lever arm between them, and how the impulse was divided.
    ///
    /// Read it as: magenta is the lever arm doing the rotating. A long magenta line with a long
    /// red arrow means a hit that will tumble the target. A near-zero magenta line means the hit
    /// landed on the centre of mass and will not rotate it no matter what the fraction says.
    /// </summary>
    [Conditional("UNITY_EDITOR")]
    public static void Impact(Rigidbody body, Vector3 worldPoint, Vector3 direction, float fraction)
    {
        if (body == null || direction.sqrMagnitude < 1e-6f) return;

        Vector3 dir = direction.normalized;
        Vector3 com = body.worldCenterOfMass;

        // Contact point and centre of mass, so the offset between them is obvious.
        DrawCross(worldPoint, 0.35f, ContactColor);
        DrawCross(com, 0.5f, CenterMassColor);

        // The lever arm. This is the whole reason an off-centre hit rotates anything.
        Debug.DrawLine(worldPoint, com, LeverArmColor, Lifetime);

        float spin  = Mathf.Clamp01(fraction);
        float shove = 1f - spin;

        // Off-centre share, drawn at the contact point where it actually acts.
        if (spin > 0f)
            DrawArrow(worldPoint, dir * (ImpulseVisualLength * spin), SpinShareColor);

        // Centre-of-mass share, drawn at the centre of mass. Pure translation, no torque.
        if (shove > 0f)
            DrawArrow(com, dir * (ImpulseVisualLength * shove), ShoveShareColor);
    }

    // -------------------------------------------------------------------------
    // 2. Splash attribution
    // -------------------------------------------------------------------------
    /// <summary>Blast sphere at the moment of detonation, so the radius can be checked against
    /// who actually got caught.</summary>
    [Conditional("UNITY_EDITOR")]
    public static void Blast(Vector3 epicenter, float radius)
    {
        DrawWireSphere(epicenter, radius, BlastColor);
        DrawCross(epicenter, 0.6f, BlastColor);
    }

    /// <summary>
    /// One victim of a blast: a line from the epicenter to the point splash was measured from,
    /// coloured by the falloff scale that point produced (green = full damage, red = nothing).
    ///
    /// The measured-from point is the thing worth watching. It should sit at the target's centre
    /// of mass. If it jumps between wheels or panels on otherwise identical shots, falloff is
    /// keying off collider iteration order again.
    /// </summary>
    [Conditional("UNITY_EDITOR")]
    public static void SplashVictim(Vector3 epicenter, Vector3 measuredFrom, float scale)
    {
        Color c = Color.Lerp(Color.red, Color.green, Mathf.Clamp01(scale));
        Debug.DrawLine(epicenter, measuredFrom, c, Lifetime);

        // Marker size tracks the falloff scale, so weak hits read as small at a glance.
        DrawCross(measuredFrom, 0.25f + 0.75f * Mathf.Clamp01(scale), c);
    }

    // -------------------------------------------------------------------------
    // 4. Particle contacts
    // -------------------------------------------------------------------------
    /// <summary>
    /// Where a particle actually landed, colour-coded by what it found there:
    ///   green  = damageable target, the hit counted
    ///   yellow = rigidbody but nothing damageable, knockback only
    ///   red    = neither, the pellet was absorbed by geometry
    ///
    /// A cluster of red on flat ground while accelerating is the "bullets eaten" symptom:
    /// the emitter is colliding with the firer's own hull or the terrain in front of it.
    /// The short tail shows the approach direction, which tells you which it was.
    /// </summary>
    [Conditional("UNITY_EDITOR")]
    public static void ParticleContact(Vector3 point, Vector3 incomingVelocity,
                                       bool hasRigidbody, bool hasDamageable)
    {
        Color c = hasDamageable ? Color.green
                : hasRigidbody  ? Color.yellow
                                : Color.red;

        DrawCross(point, 0.2f, c);

        if (incomingVelocity.sqrMagnitude > 1e-6f)
            Debug.DrawLine(point, point - incomingVelocity.normalized * 0.6f, c, Lifetime);
    }

    // -------------------------------------------------------------------------
    // Primitives
    // -------------------------------------------------------------------------
    [Conditional("UNITY_EDITOR")]
    private static void DrawCross(Vector3 center, float size, Color color)
    {
        float h = size * 0.5f;
        Debug.DrawLine(center - Vector3.right   * h, center + Vector3.right   * h, color, Lifetime);
        Debug.DrawLine(center - Vector3.up      * h, center + Vector3.up      * h, color, Lifetime);
        Debug.DrawLine(center - Vector3.forward * h, center + Vector3.forward * h, color, Lifetime);
    }

    [Conditional("UNITY_EDITOR")]
    private static void DrawArrow(Vector3 origin, Vector3 vector, Color color)
    {
        Vector3 tip = origin + vector;
        Debug.DrawLine(origin, tip, color, Lifetime);

        // Barbs, built off any axis not parallel to the shaft.
        Vector3 dir  = vector.normalized;
        Vector3 side = Vector3.Cross(dir, Mathf.Abs(dir.y) > 0.9f ? Vector3.right : Vector3.up).normalized;
        Vector3 up   = Vector3.Cross(dir, side);

        float barb = vector.magnitude * 0.2f;
        Debug.DrawLine(tip, tip - dir * barb + side * barb * 0.5f, color, Lifetime);
        Debug.DrawLine(tip, tip - dir * barb - side * barb * 0.5f, color, Lifetime);
        Debug.DrawLine(tip, tip - dir * barb + up   * barb * 0.5f, color, Lifetime);
        Debug.DrawLine(tip, tip - dir * barb - up   * barb * 0.5f, color, Lifetime);
    }

    [Conditional("UNITY_EDITOR")]
    private static void DrawWireSphere(Vector3 center, float radius, Color color)
    {
        const int segments = 24;
        float step = 360f / segments;

        for (int i = 0; i < segments; i++)
        {
            float a = i * step * Mathf.Deg2Rad;
            float b = (i + 1) * step * Mathf.Deg2Rad;

            Vector2 p1 = new Vector2(Mathf.Cos(a), Mathf.Sin(a)) * radius;
            Vector2 p2 = new Vector2(Mathf.Cos(b), Mathf.Sin(b)) * radius;

            Debug.DrawLine(center + new Vector3(p1.x, p1.y, 0f), center + new Vector3(p2.x, p2.y, 0f), color, Lifetime);
            Debug.DrawLine(center + new Vector3(p1.x, 0f, p1.y), center + new Vector3(p2.x, 0f, p2.y), color, Lifetime);
            Debug.DrawLine(center + new Vector3(0f, p1.x, p1.y), center + new Vector3(0f, p2.x, p2.y), color, Lifetime);
        }
    }
}
