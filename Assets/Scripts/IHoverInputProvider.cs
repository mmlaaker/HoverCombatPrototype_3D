/// <summary>
/// Common interface for hovercraft input.
/// Allows player and AI controllers to drive the same propulsion system.
/// </summary>
public interface IHoverInputProvider
{
    /// <summary>Forward/reverse input, typically W/S. Range -1 to +1.</summary>
    float Throttle { get; }

    /// <summary>Strafing input, optional (used for hover drones). Range -1 to +1.</summary>
    float Strafe { get; }

    /// <summary>Turning input, typically A/D or Left/Right. Range -1 to +1.</summary>
    float Turn { get; }

    /// <summary>Boost toggle, true while boosting.</summary>
    bool Boost { get; }
}