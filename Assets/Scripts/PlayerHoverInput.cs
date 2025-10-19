using UnityEngine;

/// <summary>
/// PlayerHoverInput v3.0 (Boost Support Edition)
/// ---------------------------------------------
/// • Reads throttle, turn, and boost input
/// • Supports both legacy and new Input System
/// • Works with IHoverInputProvider interface
/// </summary>
public class PlayerHoverInput : MonoBehaviour, IHoverInputProvider
{
    [Header("🕹 Input Settings")]
    [Tooltip("Axis name for forward/reverse movement.")]
    [SerializeField] private string throttleAxis = "Vertical";

    [Tooltip("Axis name for turning (left/right).")]
    [SerializeField] private string turnAxis = "Horizontal";

    [Tooltip("Key used to activate boost (hold to boost).")]
    [SerializeField] private KeyCode boostKey = KeyCode.LeftShift;

    [Tooltip("If enabled, uses Unity's new Input System instead of legacy Input.GetAxis.")]
    [SerializeField] private bool useNewInputSystem = false;

    // Input values (read-only via interface)
    public float ThrottleInput { get; private set; }
    public float TurnInput { get; private set; }
    public bool Boost { get; private set; }

#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
    // Optional new Input System bindings (if you’re using InputActions)
    private UnityEngine.InputSystem.InputAction moveAction;
    private UnityEngine.InputSystem.InputAction turnAction;
    private UnityEngine.InputSystem.InputAction boostAction;
#endif

    void Update()
    {
        if (useNewInputSystem)
        {
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
            // Using Unity's new Input System (InputActions)
            if (moveAction == null)
            {
                // Auto-bind default actions (optional)
                moveAction = new UnityEngine.InputSystem.InputAction("Throttle", binding: "<Gamepad>/leftStick/y");
                turnAction = new UnityEngine.InputSystem.InputAction("Turn", binding: "<Gamepad>/leftStick/x");
                boostAction = new UnityEngine.InputSystem.InputAction("Boost", binding: "<Keyboard>/leftShift");
                moveAction.Enable();
                turnAction.Enable();
                boostAction.Enable();
            }

            ThrottleInput = moveAction.ReadValue<float>();
            TurnInput = turnAction.ReadValue<float>();
            Boost = boostAction.ReadValue<float>() > 0.5f;
#else
            Debug.LogWarning("[PlayerHoverInput] New Input System not enabled in Player Settings.");
#endif
        }
        else
        {
            // Using classic Input Manager (GetAxis + GetKey)
            ThrottleInput = Input.GetAxisRaw(throttleAxis);
            TurnInput = Input.GetAxisRaw(turnAxis);
            Boost = Input.GetKey(boostKey);
        }

        // Optional deadzone smoothing
        if (Mathf.Abs(ThrottleInput) < 0.05f) ThrottleInput = 0f;
        if (Mathf.Abs(TurnInput) < 0.05f) TurnInput = 0f;
    }

    private void OnDisable()
    {
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
        moveAction?.Disable();
        turnAction?.Disable();
        boostAction?.Disable();
#endif
    }
}
