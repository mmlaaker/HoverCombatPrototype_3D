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
             "Useful reference point: the craft weighs 1000kg and tops out at 105 m/s, so an " +
             "impulse of 105000 is exactly one full top speed of knockback. Anything much past " +
             "that and the target is travelling faster than it can drive, which reads as being " +
             "removed from the fight rather than disrupted.\n\n" +
             "Divide by 1000 to read any value here as metres per second. Measured in play, not " +
             "derived: the Rocket at 175000 throws a stationary craft at exactly 175 m/s.\n\n" +
             "This reference said 60 m/s and 60000 until 2026-08-24. The speed pass moved top " +
             "speed to 105 and nothing rechecked it, so every weapon authored against it was " +
             "sized against a ceiling that had not existed for some time (TODO 5.16).")]
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

    [Tooltip("How much of Splash Impact Force lands on the vehicle that FIRED the shot, as a " +
             "fraction. This is the rocket jump dial.\n\n" +
             "The firer never takes splash DAMAGE regardless of this value; only the shove is " +
             "scaled. Set it to 0 to exclude yourself from your own blast completely.\n\n" +
             "Sizing it: the craft weighs 1000kg, so the self shove in m/s is " +
             "Splash Impact Force x falloff x this / 1000. Because a rocket cannot arm closer than " +
             "Speed x Arming Delay, and because it does not inherit your velocity, you close on " +
             "your own blast while it flies. Measured on the Dumbfire (splash 50000, radius 10, " +
             "arming 3.5m): the unscaled self shove is 21.5 m/s from a standstill and 40.1 m/s at " +
             "full boost. So this is strongest exactly when you are already fastest, which reads " +
             "as skill (shooting a wall you are charging) rather than noise.\n\n" +
             "Compare against the abilities it competes with before picking a number: Air Jump " +
             "Impulse is 25 m/s and Jump Impulse Max is 40. At 0.5 a rocket jump lands between a " +
             "tap jump and a charged one, which keeps it a real tool without outclassing the " +
             "energy-gated mobility.\n\n" +
             "Destabilize Fraction applies to the self hit too, but axis-aligned blasts (dead " +
             "ahead, straight down, square on a flank) put the contact point on the push axis " +
             "through the centre of mass and produce exactly zero spin. Only diagonal blasts " +
             "tumble you, peaking around 4.2 rad/s at Destabilize Fraction 0.15 -- comfortably " +
             "under the ~10-14 rad/s where a craft commits to a flip. That headroom is linear in " +
             "Destabilize Fraction, so self-flips become possible past roughly 0.36. Recheck this " +
             "if you raise it.")]
    [Range(0f, 1f)]
    public float selfImpactScale = 0.5f;
}
