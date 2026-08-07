using System;
using UnityEngine;

/// <summary>
/// Knockback tuning, consumed by both delivery paths through the shared WeaponImpact helper.
/// Pure designer-facing values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class WeaponImpactTuning
{
    // -------------------------------------------------------------------------
    // 👊 Impact
    // -------------------------------------------------------------------------
    [Header("👊 Impact")]
    [Tooltip("How hard a direct hit shoves the target, along the direction the shot was travelling. " +
             "0 for no knockback at all.\n" +
             "The two delivery modes need wildly different numbers here, so never copy a value from " +
             "one to the other:\n" +
             "ParticleSystem counts PER PARTICLE, so a shotgun blast lands this much once per " +
             "pellet. Think hundreds, up to a few thousand.\n" +
             "Instantiated applies it once, on the hit. Think tens of thousands before a rocket " +
             "visibly punts a vehicle.\n\n" +
             "Useful reference point: the craft weighs 1000kg and tops out at 60 m/s, so an impulse " +
             "of 60000 is exactly one full top speed of knockback. Anything much past that and the " +
             "target is travelling faster than it can drive, which reads as being removed from the " +
             "fight rather than disrupted.")]
    [Min(0f)]
    public float impactForce = 50f;

    [Tooltip("How hard the explosion shoves everything caught in it. Pushes outward from the blast " +
             "centre and weakens with distance. Rockets only; weapons that do not explode ignore it.\n\n" +
             "Note a direct hit does NOT also splash. The victim of a direct hit is excluded from " +
             "the blast, so a rocket delivers Impact Force or a falloff-scaled Splash Force, never " +
             "both. Splash is what near misses deliver.")]
    [Min(0f)]
    public float splashImpactForce = 0f;

    // Note for whoever tunes this next, kept out of the tooltip because it is history rather than
    // guidance: an older version of the tooltip warned that past ~0.3 hits saturate against Unity's
    // spin speed ceiling. That was measured and is wrong. The ceiling is 50 rad/s and the hardest
    // hit in the build reaches 34.7, so it is never touched. The limit that actually matters is the
    // 80 degree flip threshold, and it bites well below 0.3.
    [Tooltip("How much of a hit lands where it actually connected, rather than dead centre.\n" +
             "At 0, every hit is a clean shove and nothing ever spins. Turn it up and hit location " +
             "starts to matter: clip a flank and the target rotates, catch it square and it just " +
             "gets pushed. It does not change how FAR anything flies, only how much it tumbles, so " +
             "spin and shove tune separately.\n\n" +
             "Where to start:\n" +
             "0 for anything you hold down. Spin stacks up frame after frame and sends vehicles " +
             "into a spin nobody can recover from.\n" +
             "0.15 for rockets. A glancing hit wobbles and recovers; a solid off-centre hit tips " +
             "past the point of no return.\n" +
             "1 for shotguns, where the pellets do the work themselves: a centred blast cancels " +
             "out into a hard shove, one that catches a flank concentrates and spins hard.\n\n" +
             "Careful going up from 0.15 on anything that hits hard. This does not fade out " +
             "smoothly at the top: knock a vehicle past 80 degrees of tilt and it is fully locked " +
             "out for about 1.6 seconds, unable to steer, thrust or jump. A rocket at 0.15 " +
             "catching a flank already rolls its target all the way over every time, so going " +
             "higher does not make hits more dramatic. It just widens the band of hits that end " +
             "the exchange outright.")]
    [Range(0f, 1f)]
    public float destabilizeFraction = 0f;
}
