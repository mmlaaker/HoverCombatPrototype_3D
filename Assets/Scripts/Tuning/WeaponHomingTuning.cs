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
    [Tooltip("How fast the nose can swing, in degrees per second. Only matters for missiles given " +
             "a target (Soft Homing and Hard Lock); a Dumbfire ignores it. 0 means it never steers.\n\n" +
             "THIS IS THE EVASION DIAL. Speed decides whether the missile can catch a runner; this " +
             "decides whether a dodge beats it. Set it high and the weapon becomes unavoidable; " +
             "set it low and clever movement gets rewarded.\n\n" +
             "Do not tune the number, tune the TURNING CIRCLE it produces, which is speed divided " +
             "by this. That circle is what the player actually dodges, and it MOVES WHEN SPEED " +
             "MOVES: raise speed and the missile turns more lazily even though you never touched " +
             "this field. The Derived readout below shows the radius in metres -- watch that, not " +
             "this number.\n\n" +
             "It also caps everything else the missile is asked to do. The nose can only cover " +
             "turn rate x time degrees, so a flare or a weave that asks for more than that simply " +
             "averages out into a straight line and the dial you turned appears to do nothing.")]
    [Min(0f)]
    public float turnRate = 90f;

    [Tooltip("Seconds it flies dead straight before it starts chasing. The missile leaving the " +
             "tube before it starts thinking.\n\n" +
             "At 0 it snaps onto its target immediately, which is what makes a homing shot feel " +
             "cheap and unavoidable. A short delay lets the target SEE it coming and react, and " +
             "gives the missile a sense of weight.\n\n" +
             "It costs range, because this is straight flight the missile spends not steering: at " +
             "170 m/s even 0.2 seconds burns 34 metres. At knife-fighting range a long delay means " +
             "it arrives before it ever gets to turn, so the weapon behaves like a dumbfire " +
             "exactly when it is needed most.")]
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

    [Tooltip("Seconds the missile takes to come back in line after swinging wide. 0 turns the " +
             "flare off.\n\n" +
             "THE FLARE MUST FINISH BEFORE THE MISSILE ARRIVES, or it is still aiming wide when " +
             "it gets there and misses. Flight time is range divided by speed, so this is only " +
             "safe relative to how fast the weapon flies and how close the fight is. Tripling " +
             "missile speed once left both homing weapons missing everything inside about 40m, " +
             "because a flare authored for a slow missile never had time to resolve.\n\n" +
             "The Derived readout below turns this into metres -- 'flare resolves after N m of " +
             "travel'. Anything closer than that number is a shot that arrives still swinging. " +
             "Set it so that distance sits comfortably inside your usual fighting range.\n\n" +
             "Longer also means the missile spends more of its flight off to one side, which " +
             "gives a moving target more room to leave. Showiness versus reliability.")]
    [Min(0f)]
    public float flareDuration = 0f;

    [Tooltip("Which way 'outward' is for this missile. Alternate and Random vary it per shot, " +
             "which is what stops a volley looking like the same scripted animation played " +
             "several times.\n\n" +
             "Careful with Alternate on a volley of more than two: it only ever produces two " +
             "directions, so five missiles fly as three stacked on one path and two on the " +
             "other, and it reads as having fired two missiles. Random fans them properly.")]
    public MissileFlareMode flareDirection = MissileFlareMode.Alternate;

    [Tooltip("How far the missile wanders off the direct line on its way in, as a share of the " +
             "distance it still has to travel. 0 is off and the missile flies straight at what " +
             "it is chasing.\n\n" +
             "Unlike the flare above, which is one arc at the start, this runs for the WHOLE " +
             "flight — the missile squirms the entire way. Fired as a volley each missile gets " +
             "its own random phase, so they writhe independently and arrive from all around " +
             "instead of as a formation.\n\n" +
             "It cannot cause a miss. The wander is scaled by the distance REMAINING, so it " +
             "shrinks to nothing as the missile closes and the last stretch is always straight " +
             "at the target.\n\n" +
             "0.08 to 0.15 is a lively squiggle. Past about 0.25 the missile spends so long going " +
             "sideways that a moving target has time to leave.")]
    [Range(0f, 0.5f)]
    public float weaveAmplitude = 0f;

    [Tooltip("How many times per second the wander swings back and forth. Only matters when " +
             "Weave Amplitude is above 0.\n\n" +
             "The missile has to be able to FOLLOW its own squiggle: Turn Rate caps how fast the " +
             "nose can swing, so too high a frequency just averages out into a straight line and " +
             "the whole effect vanishes. 2 to 5 is the usable band at the turn rates in this " +
             "project; past about 8 it stops reading as motion and starts reading as jitter.\n\n" +
             "The two axes deliberately run at different rates, so the path is an irregular " +
             "wander rather than a tidy corkscrew.")]
    [Range(0f, 12f)]
    public float weaveFrequency = 3f;
}
