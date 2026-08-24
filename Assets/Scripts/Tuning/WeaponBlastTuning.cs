using System;
using UnityEngine;

/// <summary>
/// Explosion geometry. Instantiated mode only.
///
/// This used to live on the projectile prefab, on the grounds that the radius had to agree with
/// whatever the explosion VFX drew. That was always a workaround rather than a boundary, and the
/// dependency runs the other way now: this asset states the radius and the VFX is authored to
/// match it.
///
/// Pure designer-facing values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class WeaponBlastTuning
{
    // -------------------------------------------------------------------------
    // 💣 Blast
    // -------------------------------------------------------------------------
    [Header("💣 Blast")]
    [Tooltip("Blast radius in metres. Anything inside takes damage and force scaled by the falloff " +
             "curve below. Halo-style rockets sit around 5 to 8. Much past that and near misses " +
             "start deciding fights the shot did not really earn.\n\n" +
             "The explosion VFX should be authored to match this number. If the boom looks smaller " +
             "than the blast, players will not understand why a near miss shoved them.")]
    [Min(0f)]
    public float splashRadius = 6f;

    [Tooltip("Maps distance from the explosion centre to a damage and force multiplier. Left edge " +
             "is dead centre, right edge is the rim of the blast; height is the share of full " +
             "effect at that distance.\n" +
             "The default eases smoothly to nothing at the rim. A curve that stays high and then " +
             "drops off a cliff reads as a much more dangerous weapon, because there is no gentle " +
             "outer band to survive in.")]
    public AnimationCurve splashFalloff = AnimationCurve.EaseInOut(0f, 1f, 1f, 0f);

    [Tooltip("PROPS ONLY. How far below the explosion to push from, in metres, for everything " +
             "that is not a craft. Lifts debris instead of stamping it into the floor. Does not " +
             "change how hard the blast hits, only which way it throws, and it does not touch " +
             "vehicles at all.\n\n" +
             "Why props need it and craft do not: a blast shoves outward from its own centre, and " +
             "a rocket goes off at roughly craft height. A craft is level with that, so its push " +
             "is already sideways. A crate on the ground is BELOW it, so 'outward' is mostly " +
             "straight down. Measured on a crate 8m out: 84% of the push drove it into the " +
             "ground, the floor absorbed it and friction ate the rest, so a crate handed 19,000 " +
             "of impulse moved 0.00 m/s. The same impulse sideways threw it 18.6m.\n\n" +
             "Where to start: about half the blast radius. At 0 the blast is unchanged.\n\n" +
             "Raise it if blasts leave ground debris looking bolted down. There is no vehicle " +
             "flip risk to trade against, because craft are exempt -- that exemption is the whole " +
             "reason this is scoped to props rather than applied to the blast as a whole.")]
    [Min(0f)]
    [UnityEngine.Serialization.FormerlySerializedAs("upwardBias")]
    public float propUpwardBias = 0f;

    [Tooltip("Which layers the blast catches. Terrain does not need to be in here; the projectile " +
             "already detonates on terrain contact, this is about who takes the damage and force.")]
    public LayerMask damageLayers = ~0;

    [Tooltip("Spawned at the impact point when the projectile detonates. Should clean itself up " +
             "after its visuals finish; nothing here destroys it.")]
    public GameObject explosionPrefab;
}
