/// <summary>
/// IHoverInputProvider v3.0 (Drift Support)
/// ------------------------------------------
/// Interface for providing hovercraft input to the Propulsion system.
/// Implementations can be player, AI, or network-driven.
///
/// All axis values are expected pre-normalized. Controllers clamp defensively
/// but providers should not rely on that.
/// </summary>
public interface IHoverInputProvider
{
    /// <summary>
    /// Forward/reverse movement input. Range: -1 (reverse) to +1 (forward).
    /// </summary>
    float ThrottleInput { get; }

    /// <summary>
    /// Turning input. Range: -1 (left) to +1 (right).
    /// </summary>
    float TurnInput { get; }

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
}
