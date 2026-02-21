using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// PlayerHoverInput v4.0
/// ----------------------------------------
/// Reads movement (throttle + turn) and boost via Unity's New Input System.
/// Requires a PlayerInput component with a HoverControls InputActionAsset assigned.
/// Expected action map: 'Hover' with actions: 'Move' (Vector2), 'Boost' (Button)
///
/// Supported devices (configure bindings in HoverControls asset):
/// • Keyboard: WASD / Arrow Keys + Left Shift for boost
/// • PS4 DualShock (Bluetooth): Left Stick + Cross button for boost
///
/// Setup:
/// 1. Attach PlayerInput component to this GameObject
/// 2. Assign HoverControls asset to PlayerInput.Actions
/// 3. Set Default Map to 'Hover' in PlayerInput inspector
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

    // ── Serialized ───────────────────────────────────────────────────────
    
    [Header("Input Settings")]
    [Tooltip("Deadzone threshold below which axis input is zeroed out.")]
    [SerializeField, Range(0f, 0.3f)] private float deadzone = 0.05f;

    // ── Private ──────────────────────────────────────────────────────────
    
    private InputAction _moveAction;
    private InputAction _boostAction;

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

        _moveAction  = playerInput.actions["Hover/Move"];
        _boostAction = playerInput.actions["Hover/Boost"];

        if (_moveAction == null || _boostAction == null)
        {
            Debug.LogError("[PlayerHoverInput] One or more actions not found in InputActionAsset. " +
                           "Expected action map 'Hover' with actions: 'Move' (Vector2), 'Boost' (Button).", this);
            enabled = false;
            return;
        }
    }

    private void Update()
    {
        var move  = _moveAction.ReadValue<Vector2>();
        ThrottleInput = ApplyDeadzone(move.y);
        TurnInput     = ApplyDeadzone(move.x);
        Boost         = _boostAction.ReadValue<float>() > 0.5f;
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    private float ApplyDeadzone(float value)
        => Mathf.Abs(value) < deadzone ? 0f : value;
}
