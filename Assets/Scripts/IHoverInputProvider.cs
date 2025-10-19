/// <summary>
/// IHoverInputProvider v2.0 (Boost Support)
/// ----------------------------------------
/// Interface for providing hovercraft input to the Propulsion system.
/// Implementations can be player, AI, or network-driven.
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
    /// Boost input flag. True while boost control is active.
    /// </summary>
    bool Boost { get; }
}