using UnityEngine;

/// <summary>
/// Which family of debug visuals a draw belongs to. Splitting these apart is a performance
/// decision, not organisation for its own sake.
///
/// Measured 2026-08-07 on the prototype scene with two vehicles and no combat, 60 frames each:
///
///   gizmos ON    mean 4.75ms   median 3.75ms   p95 7.22ms
///   gizmos OFF   mean 3.36ms   median 3.32ms   p95 3.88ms
///
/// So roughly 1.4ms on the mean, but the p95 nearly DOUBLES. That tail is the part that matters:
/// the cost is not a uniform tax, it is occasional expensive frames, which is exactly the shape of
/// a dropped frame. And that was the cheap case, with a particle weapon as the active slot; a
/// missile slot adds a cone scan whose sphere radius is lockRange x tan(lockConeAngle), which on
/// WD_HardLockMissile is 115 metres.
///
/// Physics is unaffected either way (the worst frame still sits inside the 10ms fixed timestep, so
/// no FixedUpdate starvation and the simulation stays deterministic). But a TUNING pass judges
/// FEEL, and inconsistent frame delivery corrupts exactly that judgement -- you end up tuning
/// handling against a rendering artefact. Hence per-category control: run one readout while
/// tuning, not all of them.
/// </summary>
public enum HoverDebugCategory
{
    /// <summary>Propulsion's speed / force / blend / drift readout.</summary>
    Movement,

    /// <summary>Foundation: hover rays, ceiling duck, unstick and flip recovery diagnostics.</summary>
    Recovery,

    /// <summary>Weapons, Aim and EMP: muzzles, emitters, lock cones, acquisition preview.</summary>
    Weapons,

    /// <summary>Energy, Health and Shield bars.</summary>
    Resources,

    /// <summary>AIHoverInput: roam area, waypoint, target line, avoidance fan.</summary>
    AI,

    /// <summary>Projectile and particle impact draws: blast spheres, splash attribution, contacts.</summary>
    Impacts
}

/// <summary>
/// HoverDebugSettings v2.0
/// -----------------------
/// Global debug toggle for every gizmo and debug draw in the project. One asset lives at
/// Assets/Data/HoverDebugSettings.asset and is assigned to every component that draws anything,
/// so a single switch controls the lot.
///
/// v2.0: split into categories. There is still a master switch, but each family of draws can now
/// be turned off independently, because leaving all of them on measurably costs frames (see
/// HoverDebugCategory for the numbers).
///
/// Suggested presets:
///   Movement tuning   Movement only. One label, no physics queries, cheapest useful readout.
///   Handling / flips  Movement + Recovery.
///   Weapon tuning     Weapons + Impacts.
///   Judging feel      Master off. Frame delivery is what you are assessing, so do not pay for
///                     instrumentation while assessing it.
///
/// When no asset is assigned to a component, that component falls back to its own local
/// drawDebug bool, which is the pre-v1.0 behaviour and how everything ran before the asset
/// existed at all.
/// </summary>
[CreateAssetMenu(fileName = "HoverDebugSettings", menuName = "Hover/Debug Settings")]
public class HoverDebugSettings : ScriptableObject
{
    [Tooltip("Master switch. When off, every category is off regardless of the flags below. " +
             "Turn this off while judging feel: you are assessing frame delivery, and the gizmos " +
             "measurably affect it.")]
    public bool enableDebugGizmos = true;

    [Header("Categories")]
    [Tooltip("Propulsion's tuning readout: speed against the live caps, which force is acting, " +
             "the four authority blends, drift entry gates, and the shoulder angle. " +
             "The cheapest of these to leave on: one label, no physics queries.")]
    public bool movement = true;

    [Tooltip("Foundation: hover point spheres and rays, ceiling duck, floor contact normal, " +
             "unstick timer, and the flip recovery gate diagnostics. " +
             "The heaviest label load in the project (up to five labels).")]
    public bool recovery = true;

    [Tooltip("Weapons, Aim and EMP: muzzle and emitter markers, lock cones, and the soft-homing " +
             "acquisition preview. The preview is a cone scan, now throttled rather than run " +
             "once per repaint.")]
    public bool weapons = true;

    [Tooltip("Energy, Health and Shield bars above the chassis.")]
    public bool resources = true;

    [Tooltip("AIHoverInput: roam disc, current waypoint, state label, fire range, and the " +
             "obstacle avoidance ray fan. The avoidance rays are now drawn from cached hits " +
             "rather than re-cast per repaint.")]
    public bool ai = true;

    [Tooltip("Projectile and particle impacts: blast spheres, per-victim splash attribution, " +
             "the impulse split, and particle contact points. These persist for a few seconds " +
             "after each hit by design.")]
    public bool impacts = true;

    /// <summary>
    /// True when this category should draw. The master switch gates everything, so a caller only
    /// needs this one call rather than checking both.
    /// </summary>
    public bool IsEnabled(HoverDebugCategory category)
    {
        if (!enableDebugGizmos) return false;

        switch (category)
        {
            case HoverDebugCategory.Movement:  return movement;
            case HoverDebugCategory.Recovery:  return recovery;
            case HoverDebugCategory.Weapons:   return weapons;
            case HoverDebugCategory.Resources: return resources;
            case HoverDebugCategory.AI:        return ai;
            case HoverDebugCategory.Impacts:   return impacts;
            default:                           return false;
        }
    }
}
