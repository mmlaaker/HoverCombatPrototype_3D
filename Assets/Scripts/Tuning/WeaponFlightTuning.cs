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
    [Tooltip("How fast it flies, in metres per second.\n\n" +
             "The rule: projectiles outrun vehicles. A boosting craft does about 131, so anything " +
             "slower than that gets outrun by somebody simply driving away in a straight line. " +
             "Speed is what CATCHES a runner; Turn Rate is what a dodge beats.\n\n" +
             "CHANGING THIS MOVES THREE OTHER THINGS, and every one of them has broken a weapon " +
             "here already. It widens the turning circle (speed / turn rate). It pushes out the " +
             "arming distance (speed x arming delay), which can stop the weapon detonating near " +
             "you at all. And it shortens the flight, which can leave the flare still swinging " +
             "wide on arrival so the missile misses. Watch the Derived readout below when you " +
             "move this.")]
    [Min(1f)]
    public float speed = 70f;

    [Tooltip("Seconds before it gives up and detonates wherever it is. Stops strays flying " +
             "forever.\n\n" +
             "Speed x this is the real maximum range, and it is easy to leave far too long: at " +
             "185 m/s, 6 seconds reaches over a kilometre. If you want a weapon to expire at a " +
             "sensible distance, this is the dial — about 2 seconds puts a fast missile around " +
             "350-400m.")]
    [Min(0.1f)]
    public float lifetime = 6f;

    [Tooltip("The safety fuse: seconds after launch during which it cannot detonate. Stops it " +
             "going off on your own hull as it leaves the tube.\n\n" +
             "THIS IS A TIME BUT IT BEHAVES AS A DISTANCE. The projectile is inert for its first " +
             "speed x delay metres, so raising speed pushes that boundary out with it. A craft " +
             "hovers about 7m up, so once the arming distance passes that, the weapon can no " +
             "longer explode below its own firer -- which is exactly how rocket jumping was " +
             "silently killed once already. Measured then: 1.9 m/s of self-shove at 10m arming, " +
             "34 m/s at 3.5m.\n\n" +
             "Pick the DISTANCE you want and divide by speed. 3.5m is the value the whole roster " +
             "uses, so at 200 m/s that is 0.0175.")]
    [Min(0f)]
    public float armingDelay = 0.05f;
}
