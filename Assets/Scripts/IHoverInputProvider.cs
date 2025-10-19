/// <summary>
/// IHoverInputProvider v2.0
/// ------------------------
/// Standardized input interface for hover vehicle control.
/// Used by HoverController_Propulsion to receive directional input.
/// </summary>
public interface IHoverInputProvider
{
    /// <summary>
    /// Forward/backward input (-1 = reverse, 0 = idle, 1 = forward)
    /// </summary>
    float ThrottleInput { get; }

    /// <summary>
    /// Left/right turning input (-1 = left, 0 = neutral, 1 = right)
    /// </summary>
    float TurnInput { get; }
}