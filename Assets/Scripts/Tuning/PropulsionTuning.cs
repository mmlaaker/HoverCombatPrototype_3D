using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Propulsion. Pure designer-facing
/// values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class PropulsionTuning
{
    // -------------------------------------------------------------------------
    // 🚀 Drive
    // -------------------------------------------------------------------------
    [Header("🚀 Drive")]
    [Tooltip("How hard the chassis accelerates forward at full throttle, before boost.")]
    public float maxForwardAccel = 25f;

    [Tooltip("Forward top speed before boost.")]
    public float topSpeed = 40f;

    [Tooltip("How hard the chassis accelerates in reverse at full throttle.")]
    public float maxReverseAccel = 15f;

    [Tooltip("Reverse top speed. Independent of forward top speed. " +
             "Tune this alongside Max Reverse Accel and Strafe Top Speed; they should feel like a matched budget.")]
    public float reverseTopSpeed = 20f;

    // -------------------------------------------------------------------------
    // 🔄 Turning
    // -------------------------------------------------------------------------
    [Header("🔄 Turning")]
    [Tooltip("How fast the chassis rotates at full turn input. Higher feels twitchier and more arcade.")]
    public float yawAccel = 8f;

    [Tooltip("How firmly turning settles. Counter-torque proportional to current yaw rate. Higher kills wobble.")]
    [Range(0f, 20f)]
    public float yawDamping = 6f;

    [Tooltip("How much steering authority is retained airborne. 0 disables air turning, 1 is full ground response.")]
    [Range(0f, 1f)]
    public float airTurnMultiplier = 0.5f;

    // -------------------------------------------------------------------------
    // ⚡ Boost
    // -------------------------------------------------------------------------
    [Header("⚡ Boost")]
    [Tooltip("Master switch for boost.")]
    public bool enableBoost = true;

    [Tooltip("How much harder the chassis accelerates while boosting.")]
    [Range(1f, 3f)]
    public float boostAccelMultiplier = 1.75f;

    [Tooltip("How much higher the top speed climbs while boosting.")]
    [Range(1f, 3f)]
    public float boostSpeedMultiplier = 1.5f;

    [Tooltip("How long it takes for boost to ramp in and ramp out. Longer feels smoother, shorter feels punchier.")]
    [Min(0.01f)]
    public float boostBlendSeconds = 0.35f;

    [Tooltip("Energy drained per second of continuous boost. " +
             "Defaults assume a full 100-energy tank lasts about 5 seconds of pure boost.")]
    [Min(0f)]
    public float boostEnergyPerSecond = 20f;

    // -------------------------------------------------------------------------
    // 💨 Strafe Dodge
    // -------------------------------------------------------------------------
    [Header("💨 Strafe Dodge")]
    [Tooltip("Peak strength of the dodge burst. Front-loaded and tapers to zero. Mass independent.")]
    [Min(0f)]
    public float dodgeForce = 120f;

    [Tooltip("How long the dodge burst lasts as it tapers to zero. Shorter is snappier, longer is more jet-like. Try 0.15 to 0.35.")]
    [Min(0.01f)]
    public float dodgeDuration = 0.25f;

    [Tooltip("Flat energy cost per dodge.")]
    [Min(0f)]
    public float dodgeEnergyCost = 20f;

    [Tooltip("Seconds between dodges. Prevents spam.")]
    [Min(0f)]
    public float dodgeCooldown = 0.5f;

    // -------------------------------------------------------------------------
    // 🦘 Jump
    // -------------------------------------------------------------------------
    [Header("🦘 Jump")]
    [Tooltip("Master switch for jump.")]
    public bool enableJump = true;

    [Tooltip("Jump strength on a quick tap (minimum charge). Mass independent so all vehicles reach the same height.")]
    [Min(0f)]
    public float jumpImpulseMin = 4f;

    [Tooltip("Jump strength on a fully charged hold. Mass independent so all vehicles reach the same height.")]
    [Min(0f)]
    public float jumpImpulseMax = 12f;

    [Tooltip("How long the player must hold to reach a full charge. Charge holds at full until release.")]
    [Min(0.05f)]
    public float jumpMaxChargeTime = 2f;

    [Tooltip("Cooldown after landing before another jump is allowed.")]
    [Min(0f)]
    public float jumpGroundedLockout = 0.2f;

    [Tooltip("Energy cost for a grounded jump. Flat cost regardless of charge level.")]
    [Min(0f)]
    public float jumpGroundedEnergyCost = 25f;

    [Tooltip("Energy cost for an air jump.")]
    [Min(0f)]
    public float jumpAirEnergyCost = 25f;

    [Tooltip("Air jump strength. Not charge-based. Mass independent so all vehicles reach the same height.")]
    [Min(0f)]
    public float airJumpImpulse = 7f;

    // -------------------------------------------------------------------------
    // 🧲 Drag
    // -------------------------------------------------------------------------
    [Header("🧲 Drag")]
    [Tooltip("Sideways resistance to lateral velocity. " +
             "Controls how tightly the chassis tracks its heading and how hard strafe accel must push to build sideways speed. " +
             "Higher needs higher Strafe Accel to compensate. 0 is fully slippery sideways.")]
    [Range(0f, 50f)]
    public float lateralDamp = 2f;

    [Tooltip("Forward and reverse resistance to longitudinal velocity. " +
             "Controls coasting bleed-off when throttle is released. " +
             "Only applies near-zero throttle (drive and drag are mutually exclusive). " +
             "0 means the chassis coasts indefinitely.")]
    [Range(0f, 50f)]
    public float forwardDamp = 2f;

    // -------------------------------------------------------------------------
    // 🌀 Drift
    // -------------------------------------------------------------------------
    [Header("🌀 Drift")]
    [Tooltip("Lateral resistance while fully drifting. " +
             "Lower than Lateral Damp so the chassis slides through the turn. 0 is fully free.")]
    [Range(0f, 50f)]
    public float driftLateralDamp = 0f;

    [Tooltip("Forward resistance while fully drifting. " +
             "Typically close to Forward Damp; drift changes how the chassis slides sideways, not how it coasts forward. " +
             "Lower this slightly to carry more forward momentum through a drift.")]
    [Range(0f, 50f)]
    public float driftForwardDamp = 2f;

    [Tooltip("Yaw rate multiplier while fully drifting. " +
             "Above 1 means the nose rotates faster than the momentum vector, creating the shouldering angle. Try 1.2 to 1.6.")]
    [Range(1f, 3f)]
    public float driftYawMultiplier = 1.4f;

    [Tooltip("Minimum turn input required to engage drift. Prevents drift from triggering on gentle steering. Try 0.3 to 0.5.")]
    [Range(0f, 1f)]
    public float driftTurnThreshold = 0.4f;

    [Tooltip("Minimum forward speed required to start a drift. " +
             "Once drifting, speed is no longer checked: you own the drift until the button releases. " +
             "Aligns naturally with Strafe Top Speed: outpacing strafe means you can drift.")]
    [Min(0f)]
    public float minDriftSpeed = 20f;

    [Tooltip("How long it takes for drift to ramp in and out. Faster is snappier. Try 0.1 to 0.25.")]
    [Min(0.01f)]
    public float driftBlendSeconds = 0.15f;

    [Tooltip("Maximum chassis lean at full drift. Try 15 to 25. " +
             "Exaggerate slightly: readability matters at speed.")]
    [Range(0f, 45f)]
    public float maxBankAngle = 20f;

    [Tooltip("Subtle bank applied during normal turns (not drift). " +
             "Gives the chassis a constant carving look. Stacks with drift bank, so keep it low. Try 3 to 5.")]
    [Range(0f, 15f)]
    public float passiveBankAngle = 4f;

    [Tooltip("How fast the chassis lean catches up to the target angle. Try 6 to 10.")]
    [Range(1f, 20f)]
    public float bankLerpSpeed = 8f;

    // -------------------------------------------------------------------------
    // 🎯 Strafe Mode
    // -------------------------------------------------------------------------
    [Header("🎯 Strafe Mode")]
    [Tooltip("Master switch for strafe / aim mode (Left Trigger).")]
    public bool enableStrafe = true;

    [Tooltip("Maximum speed sustainable in strafe mode. " +
             "Entry speed above this bleeds off naturally via the soft cap. " +
             "Cannot be re-built above this threshold once in strafe.")]
    [Min(1f)]
    public float strafeTopSpeed = 20f;

    [Tooltip("Acceleration for omni-directional strafe movement on the ground plane. " +
             "Lower than forward accel: strafe is maneuvering, not charging.")]
    [Min(0f)]
    public float strafeAccel = 15f;

    [Tooltip("Maximum nose tilt (up or down) in strafe mode. " +
             "Applied via torque; Foundation's leveling fights it, creating natural resistance.")]
    [Range(5f, 45f)]
    public float strafePitchLimit = 15f;

    [Tooltip("Strength of the torque that drives the nose toward the target pitch in strafe mode. " +
             "Keep LOW: Foundation's Leveling Torque Strength fights it. Above 5 will oscillate. Try 2 to 5.")]
    [Range(0.5f, 15f)]
    public float strafePitchTorque = 3f;

    [Tooltip("Damping on pitch angular velocity in strafe mode. " +
             "Critical for preventing oscillation. Pair with Strafe Pitch Torque. Try 4 to 8.")]
    [Range(0f, 20f)]
    public float strafePitchDamping = 6f;

    [Tooltip("Aim sensitivity in degrees per second at full stick deflection. " +
             "Controls how fast the pitch angle changes with stick input. Try 60 to 120.")]
    [Range(10f, 300f)]
    public float strafePitchSensitivity = 90f;

    [Tooltip("How long it takes strafe mode to ramp in or out on trigger press / release.")]
    [Min(0.05f)]
    public float strafeModeBlendSeconds = 0.2f;

    [Tooltip("Resistance per unit of excess lateral speed in strafe. " +
             "Caps lateral velocity at Strafe Top Speed without a hard clamp. " +
             "Only acts on the lateral axis. Try 20 to 60.")]
    [Range(0f, 120f)]
    public float strafeLateralCapStrength = 40f;
}
