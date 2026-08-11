using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Weapons and ParticleWeaponCollision.
/// Pure designer-facing values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class WeaponCombatTuning
{
    // -------------------------------------------------------------------------
    // ⚔️ Combat
    // -------------------------------------------------------------------------
    [Header("⚔️ Combat")]
    [Tooltip("Health removed per hit.\n" +
             "Watch out: for ParticleSystem weapons this counts PER PARTICLE, so a shotgun blast " +
             "takes off this much for every pellet that connects. Check Burst Count in the " +
             "Emitter section before picking a number here.")]
    [Min(0f)]
    public float damage = 25f;

    [Tooltip("Ammo capacity. Set to 0 for unlimited, which is what the Machine Gun uses.")]
    [Min(0)]
    public int maxAmmo = 30;

    [Tooltip("How much ammo you spawn with. Ignored when Max Ammo is 0.")]
    [Min(0)]
    public int startingAmmo = 30;

    [Tooltip("Shots per second.\n" +
             "For ParticleSystem weapons this only paces ammo drain and the gap between bursts. How " +
             "fast bullets actually appear is the Emitter section's Emission Rate, so turning this " +
             "up will not make the gun look faster.")]
    [Min(0.01f)]
    public float fireRate = 10f;

    [Tooltip("Camera recoil kick per shot, thrown backward. 0 is off, and 0 is the default, so " +
             "recoil is OPT IN per weapon rather than something every gun inherits.\n\n" +
             "Read by HoverCameraImpulseRouter on OnWeaponFired, which fires from both the " +
             "projectile and the ParticleSystem paths, so this works for either kind of weapon. " +
             "For scale: the hard landing punch is 2 and the EMP launch is 1.2.\n\n" +
             "Pick this against FIRE RATE, not against how big the gun looks. The kick lands once " +
             "per shot, so the same value that reads as a thump on a rocket becomes a permanent " +
             "tremor on anything automatic. Rough guide: a single-shot weapon can take 0.5 to 1.5, " +
             "anything over about 10 shots per second wants 0.2 or less, and above 20 it is " +
             "probably better left at 0 and sold through the muzzle VFX instead.\n\n" +
             "Only strength is per weapon. The SHAPE of the kick lives on the Impulse_Light source " +
             "on the vehicle and is shared by every weapon.")]
    [Min(0f)]
    public float recoilVelocity = 0f;
}
