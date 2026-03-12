using UnityEngine;

/// <summary>
/// IHoverInputProvider v6.1 (StrafeX)
/// ------------------------------------------
/// Interface for providing hovercraft input to the Propulsion, Weapons, and Camera systems.
/// Implementations can be player, AI, or network-driven.
///
/// All axis values are expected pre-normalized. Controllers clamp defensively
/// but providers should not rely on that.
///
/// v6.1 changes:
///   • StrafeX added — Left Stick X axis for lateral movement in Strafe Mode.
///     Separate from TurnInput to avoid conflict with aim yaw on Right Stick X.
///     Bind to Left Stick/X [Gamepad].
///
/// v6.0 changes:
///   • StrafeHeld added  — true while Left Trigger / L2 is held. Activates Strafe/Aim Mode.
///   • CameraLookY added — Right Stick Y axis. Drive Mode: camera pitch. Strafe Mode: vehicle pitch.
///   • AimInput added    — Right Stick as Vector2 convenience accessor (X = TurnInput, Y = CameraLookY).
///   • UnityEngine using directive added to support Vector2.
///
/// v5.1 changes:
///   • CycleWeapon replaced with CycleWeaponNext and CycleWeaponPrev.
///     Next = D-Pad Right / E. Prev = D-Pad Left / Q.
///
/// v5.0 changes:
///   • FireHeld added    — true while fire button is held. Used by automatic weapons (Minigun).
///   • FirePressed added — true on the frame fire button is first pressed. Used by single-shot
///                         and instantaneous weapons (Shotgun, Rail, Missile, Mine).
///   • CycleWeapon added — true on the frame the cycle button is pressed. Consumed once per press.
/// </summary>
public interface IHoverInputProvider
{
    /// <summary>
    /// Forward/reverse movement input. Range: -1 (reverse) to +1 (forward).
    /// </summary>
    float ThrottleInput { get; }

    /// <summary>
    /// Turning input. Range: -1 (left) to +1 (right).
    /// In Strafe Mode this drives aim yaw — consumers are responsible for
    /// routing correctly based on StrafeHeld.
    /// </summary>
    float TurnInput { get; }

    /// <summary>
    /// Right Stick Y axis.
    /// Drive Mode:   camera pitch control (read by HoverCameraController).
    /// Strafe Mode:  vehicle pitch / aim elevation (read by Propulsion).
    /// Range: -1 (down) to +1 (up).
    /// </summary>
    float CameraLookY { get; }

    /// <summary>
    /// Right Stick as Vector2 (X = TurnInput, Y = CameraLookY).
    /// Convenience accessor for consumers that need both axes simultaneously.
    /// </summary>
    Vector2 AimInput { get; }

    /// <summary>
    /// True while Left Trigger / L2 is held.
    /// Activates Strafe/Aim Mode in Propulsion and Camera.
    /// </summary>
    bool StrafeHeld { get; }

    /// <summary>
    /// Left Stick X axis — lateral omni-directional movement in Strafe Mode.
    /// Range: -1 (left) to +1 (right).
    /// Separate from TurnInput so Right Stick X can own aim yaw in Strafe Mode
    /// without conflicting with lateral movement.
    /// Bind to: Left Stick/X [Gamepad].
    /// </summary>
    float StrafeX { get; }

    /// <summary>
    /// True while boost is held.
    /// </summary>
    bool Boost { get; }

    /// <summary>
    /// True while drift is held (L1).
    /// Drift only activates in Propulsion when TurnInput also exceeds the
    /// drift turn threshold — holding drift while going straight has no effect.
    /// </summary>
    bool Drift { get; }

    /// <summary>
    /// True while jump is held (Cross / X).
    /// Charge accumulates in Propulsion while this is true and the craft is grounded.
    /// Impulse fires on release. Air jump fires immediately on press while airborne.
    /// </summary>
    bool Jump { get; }

    /// <summary>
    /// True while the fire button is held (R2 / Right Mouse).
    /// Used by automatic weapons (Minigun) that fire continuously while held.
    /// Also drives missile lock scanning — lock accumulates while held over a target.
    /// </summary>
    bool FireHeld { get; }

    /// <summary>
    /// True on the first frame the fire button is pressed (rising edge only).
    /// Used by single-shot weapons (Shotgun, Rail) and instantaneous actions (Mine placement).
    /// For missiles: commit fires on the pressed frame if a lock is already confirmed.
    /// Propulsion should not consume or react to this — it belongs to Weapons only.
    /// </summary>
    bool FirePressed { get; }

    /// <summary>
    /// True on the frame the cycle-next button is pressed (rising edge only).
    /// Advances the active weapon slot forward by one (wraps at end of list).
    /// D-Pad Right / E.
    /// </summary>
    bool CycleWeaponNext { get; }

    /// <summary>
    /// True on the frame the cycle-previous button is pressed (rising edge only).
    /// Retreats the active weapon slot backward by one (wraps at start of list).
    /// D-Pad Left / Q.
    /// </summary>
    bool CycleWeaponPrev { get; }
}
