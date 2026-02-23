using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// PlayerHoverInput v5.0 (Drift Support)
/// ----------------------------------------
/// Reads movement (throttle + turn), boost, and drift via Unity's New Input System.
/// Requires a PlayerInput component with a HoverControls InputActionAsset assigned.
/// Expected action map: 'Hover' with actions:
///   • 'Move'  (Vector2) — Left Stick
///   • 'Boost' (Button)  — L3 (or reassigned)
///   • 'Drift' (Button)  — L1
///
/// Supported devices (configure bindings in HoverControls asset):
/// • Keyboard: WASD / Arrow Keys + Left Shift (boost) + Q (drift)
/// • PS4 DualShock: Left Stick + L3 (boost) + L1 (drift)
///
/// Setup:
/// 1. Attach PlayerInput component to this GameObject
/// 2. Assign HoverControls asset to PlayerInput.Actions
/// 3. Set Default Map to 'Hover' in PlayerInput inspector
/// 4. Add a 'Drift' Button action bound to L1 in your HoverControls asset
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

    /// <summary>
    /// True while drift is held (L1).
    /// Whether drift actually engages depends on TurnInput exceeding the
    /// threshold defined in HoverController_Propulsion.
    /// </summary>
    public bool Drift { get; private set; }

    // ── Serialized ───────────────────────────────────────────────────────

    [Header("Input Settings")]
    [Tooltip("Deadzone threshold below which axis input is zeroed out.")]
    [SerializeField, Range(0f, 0.3f)] private float deadzone = 0.05f;

    // ── Private ──────────────────────────────────────────────────────────

    private InputAction _moveAction;
    private InputAction _boostAction;
    private InputAction _driftAction;

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
        _driftAction = playerInput.actions["Hover/Drift"];

        if (_moveAction == null || _boostAction == null || _driftAction == null)
        {
            Debug.LogError("[PlayerHoverInput] One or more actions not found in InputActionAsset. " +
                           "Expected action map 'Hover' with actions: 'Move' (Vector2), 'Boost' (Button), 'Drift' (Button).", this);
            enabled = false;
            return;
        }
    }

    private void Update()
    {
        var move      = _moveAction.ReadValue<Vector2>();
        ThrottleInput = ApplyDeadzone(move.y);
        TurnInput     = ApplyDeadzone(move.x);
        Boost         = _boostAction.ReadValue<float>() > 0.5f;
        Drift         = _driftAction.ReadValue<float>() > 0.5f;
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    private float ApplyDeadzone(float value)
        => Mathf.Abs(value) < deadzone ? 0f : value;
}
