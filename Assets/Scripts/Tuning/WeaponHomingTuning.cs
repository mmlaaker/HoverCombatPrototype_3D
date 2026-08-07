using System;
using UnityEngine;

/// <summary>
/// Steering and the showy part of the flight path. Missile weapons only.
///
/// The flare is not a separate flight phase. The projectile always homes, but early on it homes
/// at a point offset to one side of its target, and that offset shrinks to zero over
/// flareDuration. Because the aim point ends on the target, convergence is guaranteed by
/// construction rather than by tuning, so the curve costs nothing in accuracy.
///
/// Pure designer-facing values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class WeaponHomingTuning
{
    // -------------------------------------------------------------------------
    // 🎯 Homing
    // -------------------------------------------------------------------------
    [Header("🎯 Homing")]
    [Tooltip("How sharply the missile can turn, in degrees per second. Only does anything for " +
             "missiles that get given a target (Soft Homing and Hard Lock). A Dumbfire rocket " +
             "ignores it. 0 means it never steers.\n\n" +
             "This is really a 'how tight is its turning circle' dial. At the speed these fly, " +
             "roughly:\n" +
             "  90  = a wide 45m arc, looks graceful, misses a lot\n" +
             "  160 = a 25m arc, still reads as a curve and actually connects\n" +
             "  250 = a 16m arc, snaps onto the target hard\n\n" +
             "Beware setting this low for looks. Measured against a target sitting 15m off to " +
             "one side at 30m range: at 90 the missile sailed past 9m wide, at 120 it only " +
             "clipped the edge of the blast, and 160 was the first value that scored a clean " +
             "direct hit. If a homing missile feels like it keeps 'nearly' hitting, this is the " +
             "number, not the tracking logic.")]
    [Min(0f)]
    public float turnRate = 90f;

    [Tooltip("How many seconds the missile flies dead straight before it starts chasing " +
             "anything. Think of it as the missile leaving the tube before it starts thinking.\n\n" +
             "At 0 it snaps onto its target the instant it appears, which is what makes homing " +
             "shots feel cheap and unavoidable. 0.2 to 0.5 gives the target a moment to see it " +
             "coming and react, and makes the missile look like it has some weight to it.\n\n" +
             "At the speed these fly, 0.3 seconds is roughly 20 metres of straight flight, so " +
             "at knife-fighting range a long delay means it never gets to turn at all.")]
    [Min(0f)]
    public float homingDelay = 0f;

    // -------------------------------------------------------------------------
    // 💫 Flare
    // -------------------------------------------------------------------------
    [Header("💫 Flare")]
    [Tooltip("How far out to one side the missile swings before curling back in. 0 turns the " +
             "flare off. 0.2 to 0.4 gives a clear curve you can read on screen.\n\n" +
             "Measured as a share of the distance to the target, not a fixed number of metres, so " +
             "the same setting looks right at every range: wide and dramatic at 200m, barely a " +
             "bend at 20m. It costs nothing in accuracy, because the missile aims at a point " +
             "beside its target and that point slides onto the target as it closes.\n\n" +
             "Worth knowing: the swing is measured from the TARGET, not from where you were " +
             "pointing. Flare left at something already on your left and the path just " +
             "straightens instead of bulging. The big arcs happen when you fire at something " +
             "roughly in front of you.")]
    [Range(0f, 1f)]
    public float flareOffset = 0f;

    [Tooltip("How many seconds the missile takes to come back in line after swinging wide. " +
             "0 turns the flare off.\n\n" +
             "Short, around 0.3, reads as a quick flick. Long, over 1.0, reads as a slow " +
             "sweeping arc. The catch is that the missile is off to one side for that whole " +
             "time, so a long flare gives a moving target more room to leave before it commits. " +
             "This is the showiness-versus-reliability dial.")]
    [Min(0f)]
    public float flareDuration = 0f;

    [Tooltip("Which way 'outward' is for this missile. Alternate and Random vary it per shot, " +
             "which is what stops a volley looking like the same scripted animation played " +
             "several times.")]
    public MissileFlareMode flareDirection = MissileFlareMode.Alternate;
}
