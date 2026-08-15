using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Energy. Pure designer-facing
/// values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class EnergyTuning
{
    // -------------------------------------------------------------------------
    // ⚡ Pool
    // -------------------------------------------------------------------------
    [Header("⚡ Pool")]
    [Tooltip("The size of your whole ability budget. An abstract unit: only its ratio to the costs " +
             "means anything.\n\n" +
             "Read it against the spends elsewhere in this profile (EMP, Shield, jump, dodge, and " +
             "boost per second). Raising this makes every ability cheaper in practice, so it is the " +
             "one knob that loosens all of them at once.")]
    [Min(1f)]
    public float maxEnergy = 100f;

    [Tooltip("Energy on spawn. Set it to Max Energy to start full.")]
    [Min(0f)]
    public float startingEnergy = 100f;

    // -------------------------------------------------------------------------
    // 🔋 Regeneration
    // -------------------------------------------------------------------------
    [Header("🔋 Regeneration")]
    [Tooltip("How fast the pool refills, once the lockout has expired.\n\n" +
             "With Regen Delay, this is what sets the tempo of the whole game: how long after " +
             "spending before you can act again. The boost readout in Propulsion shows the full " +
             "burn-and-refill cycle.")]
    [Min(0f)]
    public float regenRate = 20f;

    [Tooltip("Refill only while the springs are carrying the craft, so airtime never refills the " +
             "meter.\n\n" +
             "This is what makes tricks the only way to earn energy while airborne, instead of " +
             "hang time quietly paying you for doing nothing. It also makes a long fall and a flip " +
             "recovery genuinely expensive, since neither is 'on the ground'.\n\n" +
             "Turn it off to get the old behaviour back, which is the cheapest way to feel what it " +
             "is doing.")]
    public bool regenRequiresGround = true;

    [Tooltip("How long after spending before refilling starts.\n\n" +
             "This is the punish window, and it is what makes a spend a commitment rather than a " +
             "rhythm. Raise it to make emptying the tank genuinely costly. Set 0 to remove the " +
             "pause entirely and let energy refill the instant you stop spending.")]
    [Min(0f)]
    public float regenDelay = 1f;

    // -------------------------------------------------------------------------
    // 🎯 Trick payout
    // -------------------------------------------------------------------------
    [Header("🎯 Trick payout")]
    [Tooltip("What ONE barrel roll pays.\n\n" +
             "This is the floor of the whole economy: the cheapest trick a player will land over " +
             "and over. Size it against the charged jump cost in Propulsion, since a full charge " +
             "jump is what buys the trick window. Raise it to make ordinary jumping self-funding, " +
             "lower it to keep boost tight and make tricks a discount rather than an income.\n\n" +
             "A partial roll pays pro rata, so half a roll is half of this.")]
    [Min(0f)]
    public float trickBarrelRollEnergy = 10f;

    [Tooltip("What EACH barrel roll after the first pays.\n\n" +
             "Set it above One Barrel Roll and stacking rolls becomes the point; set it below and " +
             "the first roll is the only one worth doing. This is where the risk lives, because " +
             "another rotation spends airtime you wanted for the landing and a blown landing " +
             "forfeits everything banked.")]
    [Min(0f)]
    public float trickBarrelRollRepeatEnergy = 15f;

    [Tooltip("What ONE flip pays.\n\n" +
             "A flip takes roughly twice the airtime of a barrel roll, so pay it at least twice a " +
             "roll or players will simply never flip. Pay it more than that and the flip becomes " +
             "the money trick, which is defensible: it sweeps the nose through vertical and leaves " +
             "you worse set up for landing.")]
    [Min(0f)]
    public float trickFlipEnergy = 25f;

    [Tooltip("What EACH flip after the first pays.\n\n" +
             "Same job as the barrel roll repeat value, on the more expensive axis. Airtime is the " +
             "real limit here: flat ground rarely buys a second flip, so this mostly sets what big " +
             "drops are worth.")]
    [Min(0f)]
    public float trickFlipRepeatEnergy = 30f;

    [Tooltip("What a corkscrew pays, as a fraction of the same revolutions flown cleanly.\n\n" +
             "Below one this makes mixing the axes a sloppier, cheaper trick and pushes players to " +
             "commit to a clean roll or a clean flip. At one a corkscrew pays the same as clean " +
             "rotation. Above one it becomes a bonus for the harder-looking trick.\n\n" +
             "Applied to the PAYOUT and on a slope rather than a cliff: revolutions still complete " +
             "at the normal rate, and the discount is the flight's average diagonal-ness, so a " +
             "nearly clean roll is barely touched and only an evenly split corkscrew pays the full " +
             "multiplier.")]
    [Min(0f)]
    public float trickCorkscrewMultiplier = 0.5f;

    [Tooltip("Rounds a corkscrew-discounted payout to the nearest multiple of this. 0 turns it off.\n\n" +
             "Only discounted payouts are rounded. A clean trick already lands on a round number, " +
             "because the four prices above are, so rounding it would only risk moving a value you " +
             "deliberately set.\n\n" +
             "Nearest rather than upward on purpose: rounding up would erase the corkscrew discount " +
             "outright whenever it came to less than one increment, which is most of the time.")]
    [Min(0f)]
    public float trickPayoutRounding = 5f;

    [Tooltip("How close to a single axis a rotation has to be before it counts as CLEAN and escapes " +
             "the corkscrew discount entirely. It is the share of the turning that has to be on one " +
             "axis, so 0.9 means a tenth of it may be off-axis and still score full.\n\n" +
             "Without this the discount starts the instant a rotation is not perfect, which shorts a " +
             "flip that looked clean. Raise it toward 1 to demand precision, lower it to forgive a " +
             "sloppier stick. At 1 the discount begins immediately, which is the old behaviour.\n\n" +
             "Roughly, the slop it allows on the stick: 0.95 is 3 degrees, 0.9 is 6, 0.8 is 14, " +
             "0.75 is 18. Beyond the tolerance the discount ramps in smoothly, reaching full only at " +
             "an evenly split corkscrew.")]
    [Range(0.5f, 1f)]
    public float trickCleanAxisTolerance = 0.9f;

    [Tooltip("How much of a turn counts as a completed revolution.\n\n" +
             "Below 1 so a rotation that LOOKS finished pays even when it falls just short. Part of " +
             "that slack covers the measurement itself, which reads about 3% under a true 360 " +
             "because it integrates a rate rather than comparing poses; the rest is forgiveness for " +
             "execution.\n\n" +
             "Applied per revolution rather than once, so the shortfall cannot accumulate over a " +
             "long spin. Lower it to be more generous about half-finished tricks, raise it toward 1 " +
             "to demand the rotation genuinely comes all the way round.")]
    [Range(0.5f, 1f)]
    public float trickRevolutionThreshold = 0.9f;

    [Tooltip("How far the craft may be BANKED at the moment it reaches the ground and still count as " +
             "a landing, measured against the surface underneath rather than against straight up. " +
             "So landing parallel to a hillside reads as clean, the way it looks.\n\n" +
             "Lower it to demand a tidier landing. Raise it toward the downed threshold in " +
             "Foundation and it stops meaning anything, because past that the craft is on its back " +
             "and forfeits anyway.")]
    [Range(0f, 180f)]
    public float trickMaxLandingRollAngle = 45f;

    [Tooltip("The same test on the other axis: how far nose-up or nose-down the craft may be when " +
             "it reaches the ground.\n\n" +
             "Worth setting differently from the roll limit only if landing nose-first should be " +
             "punished harder than landing banked, or the reverse. Start them equal and split them " +
             "if one of the two keeps stealing tricks you felt you landed.")]
    [Range(0f, 180f)]
    public float trickMaxLandingFlipAngle = 45f;

    [Tooltip("How long the craft must stay on its belly and out of recovery after touching down " +
             "before the trick pays.\n\n" +
             "A tumble often flips the craft a moment AFTER first contact, so paying instantly " +
             "would pay for tumbles. This window is what makes landing it mean staying landed. " +
             "Raise it if scrappy landings are still getting paid; lower it if a clean landing " +
             "feels like it pays late.\n\n" +
             "Bouncing back into the air cancels the window rather than forfeiting, so a bounce " +
             "into another rotation continues the same trick.")]
    [Min(0f)]
    public float trickSettleSeconds = 0.5f;
}
