using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// PlayerHoverInput v9.1 (Left Stick as Vector2)
/// ------------------------------------------------
/// Reads all vehicle and camera inputs via Unity's New Input System.
/// Requires a PlayerInput component with a HoverControls InputActionAsset assigned.
///
/// Expected action map: 'Hover' with actions:
///   • 'Throttle'         (Vector2) — Left Stick (FULL STICK — not Y only)
///   • 'Turn'             (float)   — Right Stick X
///   • 'CameraLookY'      (float)   — Right Stick Y
///   • 'Strafe'           (Button)  — L2 / Left Trigger
///   • 'Boost'            (Button)  — L3
///   • 'Drift'            (Button)  — L1
///   • 'Jump'             (Button)  — Cross / X
///   • 'Fire'             (Button)  — R2 / Left Mouse
///   • 'CycleWeaponNext'  (Button)  — D-Pad Right / E
///   • 'CycleWeaponPrev'  (Button)  — D-Pad Left / Q
///
/// Input routing notes:
///   • Throttle is now a Vector2 — Y drives forward/back, X drives StrafeX lateral.
///     Reading both from the same action prevents Unity's per-axis normalization
///     from crushing one component when both are deflected simultaneously.
///   • TurnInput is Right Stick X in both modes — consumers decide context.
///   • CameraLookY is Right Stick Y — Drive: camera pitch / Strafe: vehicle pitch.
///   • AimInput is TurnInput + CameraLookY composed as a Vector2 convenience.
///   • StrafeHeld is a raw hold — no rising edge needed.
///
/// Input Asset changes from v9.0:
///   • Delete the 'StrafeX' action entirely.
///   • Change 'Throttle' binding from Left Stick/Y to Left Stick [Gamepad].
///   • Set Throttle Action Type = Value, Control Type = Vector2.
///
/// v9.1 changes:
///   • Throttle reads full Left Stick Vector2 — X → StrafeX, Y → ThrottleInput.
///   • Removed StrafeX action and _strafeXAction entirely.
/// </summary>
[RequireComponent(typeof(PlayerInput))]
public class PlayerHoverInput : MonoBehaviour, IHoverInputProvider
{
    // ── IHoverInputProvider ──────────────────────────────────────────────

    /// <summary>Forward/reverse throttle. Range: -1 to +1.</summary>
    public float ThrottleInput { get; private set; }

    /// <summary>Right Stick X. Yaw in both modes — consumer selects context.</summary>
    public float TurnInput { get; private set; }

    /// <summary>
    /// Right Stick Y. Drive Mode: camera pitch. Strafe Mode: vehicle pitch.
    /// Range: -1 (down) to +1 (up).
    /// </summary>
    public float CameraLookY { get; private set; }

    /// <summary>Right Stick as Vector2 (X = TurnInput, Y = CameraLookY).</summary>
    public Vector2 AimInput { get; private set; }

    /// <summary>True while Left Trigger / L2 is held. Activates Strafe Mode.</summary>
    public bool StrafeHeld { get; private set; }

    /// <summary>Left Stick X — lateral movement in Strafe Mode. Range: -1 to +1.</summary>
    public float StrafeX { get; private set; }

    /// <summary>True while boost is held.</summary>
    public bool Boost { get; private set; }

    /// <summary>True while drift is held.</summary>
    public bool Drift { get; private set; }

    /// <summary>True while jump is held.</summary>
    public bool Jump { get; private set; }

    /// <summary>True while fire is held. Drives Minigun wind-up, missile scanning.</summary>
    public bool FireHeld { get; private set; }

    /// <summary>True on first frame fire is pressed (rising edge).</summary>
    public bool FirePressed { get; private set; }

    /// <summary>True on first frame D-Pad Right / E pressed (rising edge).</summary>
    public bool CycleWeaponNext { get; private set; }

    /// <summary>True on first frame D-Pad Left / Q pressed (rising edge).</summary>
    public bool CycleWeaponPrev { get; private set; }

    /// <summary>True on first frame shield is pressed (rising edge).</summary>
    public bool ShieldPressed { get; private set; }

    /// <summary>True on first frame EMP is pressed (rising edge).</summary>
    public bool EmpPressed { get; private set; }

    // ── Serialized ───────────────────────────────────────────────────────

    [Header("Input Settings")]
    [Tooltip("Deadzone threshold below which axis input is zeroed out.")]
    [SerializeField, Range(0f, 0.3f)] private float deadzone = 0.05f;

    // ── Private ──────────────────────────────────────────────────────────

    private InputAction _throttleAction;
    private InputAction _turnAction;
    private InputAction _cameraLookYAction;
    private InputAction _strafeAction;
    private InputAction _boostAction;
    private InputAction _driftAction;
    private InputAction _jumpAction;
    private InputAction _fireAction;
    private InputAction _cycleWeaponNextAction;
    private InputAction _cycleWeaponPrevAction;
    private InputAction _shieldAction;
    private InputAction _empAction;

    private bool _fireHeldLastFrame;
    private bool _cycleNextHeldLastFrame;
    private bool _cyclePrevHeldLastFrame;
    private bool _shieldHeldLastFrame;
    private bool _empHeldLastFrame;

    // ── Unity Lifecycle ──────────────────────────────────────────────────

    private void Awake()
    {
        var playerInput = GetComponent<PlayerInput>();

        if (playerInput.actions == null)
        {
            Debug.LogError("[PlayerHoverInput] No InputActionAsset assigned to PlayerInput.", this);
            enabled = false;
            return;
        }

        _throttleAction        = playerInput.actions["Hover/Throttle"];
        _turnAction            = playerInput.actions["Hover/Turn"];
        _cameraLookYAction     = playerInput.actions["Hover/CameraLookY"];
        _strafeAction          = playerInput.actions["Hover/Strafe"];
        _boostAction           = playerInput.actions["Hover/Boost"];
        _driftAction           = playerInput.actions["Hover/Drift"];
        _jumpAction            = playerInput.actions["Hover/Jump"];
        _fireAction            = playerInput.actions["Hover/Fire"];
        _cycleWeaponNextAction = playerInput.actions["Hover/CycleWeaponNext"];
        _cycleWeaponPrevAction = playerInput.actions["Hover/CycleWeaponPrev"];
        _shieldAction          = playerInput.actions["Hover/Shield"];
        _empAction             = playerInput.actions["Hover/EMP"];

        if (_throttleAction == null || _turnAction == null || _cameraLookYAction == null ||
            _strafeAction == null   || _boostAction == null || _driftAction == null ||
            _jumpAction == null     || _fireAction == null  ||
            _cycleWeaponNextAction == null || _cycleWeaponPrevAction == null ||
            _shieldAction == null   || _empAction == null)
        {
            Debug.LogError(
                "[PlayerHoverInput] One or more actions missing from InputActionAsset. " +
                "Expected Hover action map with: Throttle, Turn, CameraLookY, Strafe, " +
                "Boost, Drift, Jump, Fire, CycleWeaponNext, CycleWeaponPrev, Shield, EMP.", this);
            enabled = false;
        }
    }

    private void Update()
    {
        // Read full Left Stick as Vector2 — avoids Unity per-axis normalization
        // that crushes one component when both axes are deflected simultaneously.
        Vector2 leftStick = _throttleAction.ReadValue<Vector2>();
        ThrottleInput = ApplyDeadzone(leftStick.y);
        StrafeX       = ApplyDeadzone(leftStick.x);

        TurnInput     = ApplyDeadzone(_turnAction.ReadValue<float>());
        CameraLookY   = ApplyDeadzone(_cameraLookYAction.ReadValue<float>());
        AimInput      = new Vector2(TurnInput, CameraLookY);

        StrafeHeld = _strafeAction.ReadValue<float>() > 0.5f;
        Boost      = _boostAction.ReadValue<float>()  > 0.5f;
        Drift      = _driftAction.ReadValue<float>()  > 0.5f;
        Jump       = _jumpAction.ReadValue<float>()   > 0.5f;

        bool fireRaw = _fireAction.ReadValue<float>() > 0.5f;
        FireHeld     = fireRaw;
        FirePressed  = fireRaw && !_fireHeldLastFrame;
        _fireHeldLastFrame = fireRaw;

        bool cycleNextRaw       = _cycleWeaponNextAction.ReadValue<float>() > 0.5f;
        CycleWeaponNext         = cycleNextRaw && !_cycleNextHeldLastFrame;
        _cycleNextHeldLastFrame = cycleNextRaw;

        bool cyclePrevRaw       = _cycleWeaponPrevAction.ReadValue<float>() > 0.5f;
        CycleWeaponPrev         = cyclePrevRaw && !_cyclePrevHeldLastFrame;
        _cyclePrevHeldLastFrame = cyclePrevRaw;

        bool shieldRaw         = _shieldAction.ReadValue<float>() > 0.5f;
        ShieldPressed          = shieldRaw && !_shieldHeldLastFrame;
        _shieldHeldLastFrame   = shieldRaw;

        bool empRaw            = _empAction.ReadValue<float>() > 0.5f;
        EmpPressed             = empRaw && !_empHeldLastFrame;
        _empHeldLastFrame      = empRaw;
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    private float ApplyDeadzone(float value)
        => Mathf.Abs(value) < deadzone ? 0f : value;
}
