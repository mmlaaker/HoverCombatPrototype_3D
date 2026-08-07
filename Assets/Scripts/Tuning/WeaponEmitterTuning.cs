using System;
using UnityEngine;

/// <summary>
/// Particle emitter configuration. ParticleSystem mode only.
///
/// Unlike every other tuning section, these values cannot simply be read live: Unity's particle
/// simulation reads its settings off the ParticleSystem component, not off this asset. So
/// ParticleWeaponCollision writes them into the emitter, at runtime in Awake and again in the
/// editor via OnValidate. That is the only reason this section behaves differently from the rest.
///
/// Before this existed, these values lived four override layers deep (VFX_*.prefab into the
/// vehicle prefab into a prefab variant into the scene instance), and the player and AI craft had
/// silently drifted apart on emission rate and muzzle velocity.
///
/// Pure designer-facing values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class WeaponEmitterTuning
{
    // -------------------------------------------------------------------------
    // 🔊 Emitter
    // -------------------------------------------------------------------------
    [Header("🔊 Emitter")]
    [Tooltip("Bullets per second while the trigger is held. This is what actually controls how " +
             "fast a sustained gun looks and feels; Fire Rate up in Combat only paces ammo drain.\n" +
             "Leave at 0 for burst weapons like the Shotgun, which use Burst Count instead.")]
    [Min(0f)]
    public float emissionRate = 10f;

    [Tooltip("Pellets released in one go per shot. This is the Shotgun's whole identity: a cone of " +
             "pellets, each one carrying Impact Force separately, so a single burst lands many " +
             "times over.\n" +
             "Leave at 0 for sustained guns, which use Emission Rate instead.")]
    [Min(0)]
    public int burstCount = 0;

    [Tooltip("Muzzle velocity in metres per second. Vehicles top out at 60, so anything under about " +
             "100 will visibly trail behind a fleeing target and feel wrong.")]
    [Min(0f)]
    public float startSpeed = 150f;

    [Tooltip("Optional lower bound for muzzle velocity. Leave at 0 for a constant speed.\n" +
             "Set it below Start Speed and each bullet picks a random speed in between, which makes " +
             "a burst read as a spray of individual pellets rather than a solid wall moving as one " +
             "object. The Shotgun uses this.")]
    [Min(0f)]
    public float startSpeedMin = 0f;

    [Tooltip("Seconds a bullet lives before vanishing. This is the weapon's effective range divided " +
             "by its speed: 0.5s at 150 m/s is about 75 metres of reach.\n" +
             "Tune this rather than the range, because it is what the player actually sees when " +
             "their tracers stop short.")]
    [Min(0.01f)]
    public float startLifetime = 0.5f;

    [Tooltip("Spread in degrees, measured from the centre outward. 1 to 2 is a tight, accurate " +
             "stream. Around 15 is shotgun scatter, and that spread is what makes a burst rotate a " +
             "target instead of just shoving it, because the pellets land in many different places.")]
    [Range(0f, 90f)]
    public float coneAngle = 1f;

    [Tooltip("Radius of the muzzle the bullets leave from, in metres. Near 0 fires from a single " +
             "point; larger values spread the origin out across a wider barrel, which reads better " +
             "on shotguns than on a rifle.")]
    [Min(0f)]
    public float coneRadius = 0.0001f;

    [Tooltip("Which layers the bullets collide with. Include every vehicle layer plus Default; the " +
             "firer's own layer is stripped out automatically at runtime, so the player and the AI " +
             "each end up unable to shoot themselves without needing separate values here.")]
    public LayerMask damageLayers = ~0;
}
