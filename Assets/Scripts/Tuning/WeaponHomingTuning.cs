using System;
using UnityEngine;

/// <summary>
/// Steering and the showy part of the flight path. Missile weapons only.
///
/// The flare is not a separate flight phase. The projectile always homes, but early on it homes
/// at a point offset to one side of its target, and that offset shrinks to zero over
/// the flare. Because the aim point ends on the target, convergence is guaranteed by
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
    [Tooltip("How far off-target the missile swings before curling back in. 0 turns the flare " +
             "off. 0.2 to 0.4 gives a clear curve you can read on screen; 0.8 and up is a " +
             "pronounced arc.\n\n" +
             "EASIEST WAY TO SET IT: this number is the tangent of the launch angle, so it IS " +
             "the angle the missile peels off at, and that angle is the same at every range. " +
             "0.3 leaves at 17 degrees, 0.5 at 27, 0.85 at 40, 1.0 at 45. Pick the departure " +
             "angle you want to see and use the matching number.\n\n" +
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

    [Tooltip("How much of the trip the missile spends swinging wide before it settles onto the " +
             "target, as a SHARE OF THE FLIGHT rather than a number of seconds. 0 turns the flare " +
             "off. 0.6 means the arc occupies the first 60% of the journey and the last 40% is " +
             "the missile converging.\n\n" +
             "Because it is a share, the arc looks the SAME at every range -- a 30m snap shot and " +
             "a 125m lob trace the same shape, just scaled. It also cannot be broken by changing " +
             "the missile's speed or its lock range, which is the entire reason it is expressed " +
             "this way.\n\n" +
             "This used to be authored in seconds and it did not survive contact. Seconds are " +
             "absolute; flight time is not, so the same setting was a dramatic arc up close and a " +
             "barely visible kink far out -- worst at exactly the ranges you have time to watch " +
             "it. Raising lock range made it worse, and raising missile speed silently rescaled " +
             "it. It needed a runtime safety cap just to stop point-blank shots sailing straight " +
             "over the target. None of that applies now: a share of the flight cannot outlast the " +
             "flight.\n\n" +
             "Higher means a bigger, showier arc and less of the trip spent lining up, which " +
             "gives a moving target more room to leave. Showiness versus reliability. Past about " +
             "0.65 the missile runs out of room to converge and starts arriving still turning.")]
    [Range(0f, 0.75f)]
    public float flareFlightFraction = 0f;

    [Tooltip("Which way the missile swings before curling back in. All of these are the same " +
             "thing -- an angle measured from straight up -- so they are directly comparable: " +
             "Loft 0, Right +90, Left -90, and Random anywhere between.\n\n" +
             "LOFT climbs, tips over and drops onto the target from above. Reach for it when you " +
             "want missiles seen raining down rather than snaking across the ground.\n\n" +
             "RANDOM is continuous, not a shuffle of the other entries: any angle from full left, " +
             "through straight up, to full right, including every diagonal. Best for volleys, " +
             "because no two missiles take the same path.\n\n" +
             "Careful with Alternate on a volley of more than two: it only ever produces two " +
             "directions, so five missiles fly as three stacked on one path and two on the " +
             "other, and it reads as having fired two missiles. Random fans them properly.\n\n" +
             "All of them are measured against the WORLD, not the car, so an arc you author is " +
             "the arc you get whether you fired level or halfway through a bank. Nothing rolls " +
             "below horizontal either: that is not a flare, it is a hole in the ground -- the aim " +
             "point ends up under the map and the missile detonates on the road.")]
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
