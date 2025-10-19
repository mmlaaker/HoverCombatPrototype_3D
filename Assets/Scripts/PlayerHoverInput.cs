using UnityEngine;

/// <summary>
/// PlayerHoverInput v2.0
/// ----------------------
/// Player-controlled input provider for HoverController_Propulsion.
/// Maps Unity Input axes to ThrottleInput and TurnInput.
/// </summary>
[DisallowMultipleComponent]
public class PlayerHoverInput : MonoBehaviour, IHoverInputProvider
{
    [Header("🎮 Input Mapping")]
    [Tooltip("Name of the input axis for forward/backward movement (Project Settings → Input Manager or Input System).")]
    public string throttleAxis = "Vertical";

    [Tooltip("Name of the input axis for left/right turning.")]
    public string turnAxis = "Horizontal";

    [Header("⚙️ Input Settings")]
    [Tooltip("Apply smoothing to input for a more analog feel.")]
    [Range(0f, 1f)] public float inputSmoothing = 0.15f;

    private float smoothedThrottle;
    private float smoothedTurn;

    // Exposed properties for Propulsion
    public float ThrottleInput => smoothedThrottle;
    public float TurnInput => smoothedTurn;

    void Update()
    {
        // Raw input
        float targetThrottle = Input.GetAxisRaw(throttleAxis);
        float targetTurn = Input.GetAxisRaw(turnAxis);

        // Smooth interpolation for analog feel
        smoothedThrottle = Mathf.Lerp(smoothedThrottle, targetThrottle, 1f - Mathf.Pow(1f - inputSmoothing, Time.deltaTime * 60f));
        smoothedTurn = Mathf.Lerp(smoothedTurn, targetTurn, 1f - Mathf.Pow(1f - inputSmoothing, Time.deltaTime * 60f));
    }
}