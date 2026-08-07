using System;
using UnityEngine;

/// <summary>
/// How a spawned projectile travels. Instantiated mode only; ParticleSystem weapons ignore it.
/// Read live by RocketProjectile off the WeaponDefinition, so editing these during play mode
/// affects projectiles already in the air.
///
/// Pure designer-facing values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class WeaponFlightTuning
{
    // -------------------------------------------------------------------------
    // ✈️ Flight
    // -------------------------------------------------------------------------
    [Header("✈️ Flight")]
    [Tooltip("How fast the projectile travels, in metres per second. Halo-style rockets sit around " +
             "60 to 90.\n\n" +
             "Worth knowing: vehicles top out at 60 m/s. At a projectile speed of 70 you are only " +
             "closing on a target running flat-out at 10 m/s, so homing weapons cannot realistically " +
             "chase down someone fleeing in a straight line. If you want them to, this is the number " +
             "to raise, not the turn rate.")]
    [Min(1f)]
    public float speed = 70f;

    [Tooltip("Maximum seconds the projectile can fly before it gives up and detonates where it is. " +
             "Stops strays drifting forever.")]
    [Min(0.1f)]
    public float lifetime = 6f;

    [Tooltip("Short delay after spawn during which the projectile cannot detonate. Stops it going " +
             "off on the firer's own hull at the muzzle. Try 0.05 to 0.15.\n\n" +
             "At speed this is also a minimum arming distance: 0.05 at 70 m/s means the projectile " +
             "is inert for its first 3.5 metres.")]
    [Min(0f)]
    public float armingDelay = 0.05f;
}
