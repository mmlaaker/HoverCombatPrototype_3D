using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerHoverInput : MonoBehaviour, IHoverInputProvider
{
    [Header("Boost Key")]
    public Key boostKey = Key.LeftShift;
    [Range(0f, 1f)] public float smoothing = 0.15f;

    public float Throttle { get; private set; }
    public float Strafe => 0f;
    public float Turn { get; private set; }
    public bool Boost { get; private set; }

    private float throttleVelocity;
    private float turnVelocity;

    void Update()
    {
        // New Input System reads via Keyboard.current
        float rawThrottle = 0f;
        if (Keyboard.current.wKey.isPressed) rawThrottle += 1f;
        if (Keyboard.current.sKey.isPressed) rawThrottle -= 1f;

        float rawTurn = 0f;
        if (Keyboard.current.dKey.isPressed) rawTurn += 1f;
        if (Keyboard.current.aKey.isPressed) rawTurn -= 1f;

        Throttle = Mathf.SmoothDamp(Throttle, rawThrottle, ref throttleVelocity, smoothing);
        Turn = Mathf.SmoothDamp(Turn, rawTurn, ref turnVelocity, smoothing);
        Boost = Keyboard.current[boostKey].isPressed;
    }
}