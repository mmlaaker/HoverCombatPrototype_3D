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
}
