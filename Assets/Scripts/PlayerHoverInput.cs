using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// PlayerHoverInput v7.0 (Jump Support)
/// ------------------------------------------------
/// Reads throttle, turn, boost, drift, and jump via Unity's New Input System.
/// Requires a PlayerInput component with a HoverControls InputActionAsset assigned.
/// Expected action map: 'Hover' with actions:
///   • 'Throttle' (float)  — Left Stick Y
///   • 'Turn'     (float)  — Right Stick X
///   • 'Boost'    (Button) — Square
///   • 'Drift'    (Button) — L1
///   • 'Jump'     (Button) — Cross / X
///
/// Supported devices:
/// • Keyboard: W/S (throttle) + A/D (turn) + Left Shift (boost) + Left Alt (drift) + Space (jump)
/// • PS4: Left Stick Y (throttle) + Right Stick X (turn) + Square (boost) + L1 (drift) + Cross (jump)
///
/// Setup:
/// 1. Add 'Jump' action (Button) to the Hover action map in HoverControls asset
/// 2. Bind to Cross [PlayStation Controller]
/// 3. Bind to Space [Keyboard]
/// </summary>
[RequireComponent(typeof(PlayerInput))]
public class PlayerHoverInput : MonoBehaviour, IHoverInputProvider
{
    // ── IHoverInputProvider ──────────────────────────────────────────────

    /// <summary>Forward/reverse input. Range: -1 (reverse) to +1 (forward).</summary>
    public float ThrottleInput { get; private set; }

    /// <summary>Turn input. Range: -1 (left) to +1 (right).</summary>
    public float TurnInput { get; private set; }

    /// <summary>True while boost is held.</summary>
    public bool Boost { get; private set; }

    /// <summary>True while drift is held (L1).</summary>
    public bool Drift { get; private set; }

    /// <summary>True while jump is held (Cross / X).</summary>
    public bool Jump { get; private set; }

    // ── Serialized ───────────────────────────────────────────────────────

    [Header("Input Settings")]
    [Tooltip("Deadzone threshold below which axis input is zeroed out.")]
    [SerializeField, Range(0f, 0.3f)] private float deadzone = 0.05f;

    // ── Private ──────────────────────────────────────────────────────────

    private InputAction _throttleAction;
    private InputAction _turnAction;
    private InputAction _boostAction;
    private InputAction _driftAction;
    private InputAction _jumpAction;

    // ── Unity Lifecycle ──────────────────────────────────────────────────

    private void Awake()
    {
        var playerInput = GetComponent<PlayerInput>();

        if (playerInput.actions == null)
        {
            Debug.LogError("[PlayerHoverInput] No InputActionAsset assigned to PlayerInput component. " +
                           "Assign HoverControls to the PlayerInput Actions field in the Inspector.", this);
            enabled = false;
            return;
        }

        _throttleAction = playerInput.actions["Hover/Throttle"];
        _turnAction     = playerInput.actions["Hover/Turn"];
        _boostAction    = playerInput.actions["Hover/Boost"];
        _driftAction    = playerInput.actions["Hover/Drift"];
        _jumpAction     = playerInput.actions["Hover/Jump"];

        if (_throttleAction == null || _turnAction == null || _boostAction == null ||
            _driftAction    == null || _jumpAction == null)
        {
            Debug.LogError("[PlayerHoverInput] One or more actions not found in InputActionAsset. " +
                           "Expected action map 'Hover' with actions: " +
                           "'Throttle' (float), 'Turn' (float), 'Boost' (Button), " +
                           "'Drift' (Button), 'Jump' (Button).", this);
            enabled = false;
            return;
        }
    }

    private void Update()
    {
        ThrottleInput = ApplyDeadzone(_throttleAction.ReadValue<float>());
        TurnInput     = ApplyDeadzone(_turnAction.ReadValue<float>());
        Boost         = _boostAction.ReadValue<float>() > 0.5f;
        Drift         = _driftAction.ReadValue<float>() > 0.5f;
        Jump          = _jumpAction.ReadValue<float>()  > 0.5f;
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    private float ApplyDeadzone(float value)
        => Mathf.Abs(value) < deadzone ? 0f : value;
}
