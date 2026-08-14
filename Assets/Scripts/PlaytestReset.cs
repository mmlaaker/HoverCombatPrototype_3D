using UnityEngine;
using UnityEngine.InputSystem;

// GamepadButton lives in .LowLevel while Key and Gamepad do not. Aliased rather
// than pulling the whole LowLevel namespace in for one enum.
using GamepadButton = UnityEngine.InputSystem.LowLevel.GamepadButton;

/// <summary>
/// PlaytestReset v1.0
/// ------------------------------------------------
/// Returns the player craft to the pose it started the session in, with its
/// velocities zeroed. Nothing else. Held rather than tapped, so it cannot be
/// hit by accident mid-run.
///
/// WHAT THIS IS NOT
///
///   This is NOT the respawn system, and finishing it does not advance TODO 1.1.
///   That item covers spawn point selection, post-respawn invulnerability, AI
///   state reset and the ownership question. This is a playtest escape hatch for
///   a soft-lock or a fall outside the test environment, and it deliberately
///   does not touch health, energy, shield, ammo or weapon slot: a reset that
///   silently topped those up would make every run after it unreadable, and
///   would quietly become a half-built respawn that nobody trusts.
///
///   It also cannot reuse VehicleHealth.Respawn(). Per TODO 1.1 that stub does
///   not reset position or velocity, which is the entire job here.
///
/// WHY THE SPAWN POSE IS CAPTURED IN Start
///
///   The authored scene pose IS the starting point, which is what was asked
///   for. Capturing later would mean capturing wherever the craft had settled
///   or drifted to, and "reset" would slowly stop meaning anything across a
///   long session.
///
/// THE CAMERA HAS TO BE TOLD
///
///   Cinemachine damps toward the follow target from its own remembered world
///   position. Move the craft without telling it and the camera spends the next
///   second flying across the level, which looks far more broken than whatever
///   you pressed reset to escape. HoverCameraController.NotifyVehicleWarped
///   exists for this and is the only reason this component knows the camera
///   exists at all.
///
/// THE LOG LINE IS AN INSTRUMENT, NOT NOISE
///
///   A teleport is a colossal one-frame position delta, and MotionTrace derives
///   visAlong, visSpeed and residM from drawn position deltas. A reset therefore
///   lands in the CSV as an enormous speed spike that looks exactly like the
///   sort of artefact these sessions are hunting. The log stamps
///   Time.unscaledTime, the SAME clock MotionTrace writes into its t column, so
///   the row can be found and discounted instead of investigated. Keep that
///   stamp if this is ever rewritten.
///
/// Usage: drop on any scene object. Leave vehicle unassigned to auto-find the
/// player craft. Hold Select (View / Share / Back) on the pad, or R on the
/// keyboard, for holdSeconds.
/// </summary>
[DisallowMultipleComponent]
public class PlaytestReset : MonoBehaviour
{
    [Header("Target")]
    [Tooltip("The player craft. Leave empty to find it automatically by its PlayerHoverInput, " +
             "which is the one component AI craft do not share.")]
    [SerializeField] private Transform vehicle;

    [Header("Trigger")]
    [Tooltip("Off disables the hotkeys entirely, without removing the component or losing the " +
             "captured spawn pose. Intended for a run where a stray press would invalidate the " +
             "measurement.")]
    [SerializeField] private bool enableHotkeys = true;

    [Tooltip("Gamepad button, held. Select is View on Xbox, Share on PlayStation, and is the one " +
             "face-adjacent button HoverControls.inputactions leaves unbound, so it cannot " +
             "collide with a real control.")]
    [SerializeField] private GamepadButton gamepadButton = GamepadButton.Select;

    [Tooltip("Keyboard equivalent. The input asset binds no keyboard controls at all, so any " +
             "plain key is free. Read directly off the device rather than through an action, " +
             "for the same reason MotionTrace reads its marker key that way: a debug utility " +
             "should not require editing the shipped input asset.")]
    [SerializeField] private Key keyboardKey = Key.R;

    [Tooltip("How long the button must be held. Long enough that a fumbled grip cannot trigger " +
             "it, short enough to be usable one-handed while the craft is stuck. Resets the " +
             "moment the button is released.")]
    [SerializeField, Range(0.1f, 3f)] private float holdSeconds = 0.6f;

    // ── Private ──────────────────────────────────────────────────────────

    private Rigidbody             _rb;
    private HoverCameraController _camera;

    private Vector3    _spawnPosition;
    private Quaternion _spawnRotation;
    private bool       _armed;      // false when there is nothing to reset

    private float _held;
    private int   _resetCount;

    // ── Unity Lifecycle ──────────────────────────────────────────────────

    private void Start()
    {
        if (vehicle == null)
        {
            var input = Object.FindFirstObjectByType<PlayerHoverInput>();
            if (input != null) vehicle = input.transform;
        }

        if (vehicle == null)
        {
            Debug.LogWarning("[PlaytestReset] No vehicle assigned and no PlayerHoverInput in the " +
                             "scene. Reset is inactive for this session.", this);
            return;
        }

        _rb = vehicle.GetComponent<Rigidbody>();
        if (_rb == null)
        {
            Debug.LogWarning($"[PlaytestReset] '{vehicle.name}' has no Rigidbody. Reset is " +
                             "inactive: teleporting the transform alone would leave the " +
                             "simulation holding the old velocity and drag the craft " +
                             "straight back out.", this);
            return;
        }

        _camera = Object.FindFirstObjectByType<HoverCameraController>();
        if (_camera == null)
        {
            Debug.LogWarning("[PlaytestReset] No HoverCameraController found. Reset will still " +
                             "work, but the camera will visibly fly to catch up.", this);
        }

        _spawnPosition = vehicle.position;
        _spawnRotation = vehicle.rotation;
        _armed         = true;
    }

    private void Update()
    {
        if (!_armed || !enableHotkeys) return;

        if (!ButtonHeld())
        {
            _held = 0f;
            return;
        }

        // Unscaled: a reset must stay reachable if a bug has left timeScale at
        // zero, which is one of the soft-locks this exists to escape.
        _held += Time.unscaledDeltaTime;

        if (_held < holdSeconds) return;

        _held = 0f;
        DoReset();
    }

    // ── Internals ────────────────────────────────────────────────────────

    private bool ButtonHeld()
    {
        var pad = Gamepad.current;
        if (pad != null && pad[gamepadButton].isPressed) return true;

        var kb = Keyboard.current;
        return kb != null && kb[keyboardKey].isPressed;
    }

    private void DoReset()
    {
        Vector3 delta = _spawnPosition - vehicle.position;

        // Interpolation is toggled off across the move and restored after. Left
        // on, the Rigidbody smoothly DRAWS the teleport: one frame showing the
        // craft streaked between the old position and the new one, which is
        // both ugly and a fake reading for anything measuring drawn motion.
        RigidbodyInterpolation interpolation = _rb.interpolation;
        _rb.interpolation = RigidbodyInterpolation.None;

        _rb.linearVelocity  = Vector3.zero;
        _rb.angularVelocity = Vector3.zero;
        _rb.position        = _spawnPosition;
        _rb.rotation        = _spawnRotation;

        // The Rigidbody assignments above do not reach the Transform until the
        // next physics step, and everything reading the craft this frame reads
        // the Transform. Syncing now keeps the two from disagreeing for a frame,
        // which is exactly long enough for the camera warp below to be handed a
        // stale position.
        vehicle.SetPositionAndRotation(_spawnPosition, _spawnRotation);
        Physics.SyncTransforms();

        _rb.interpolation = interpolation;

        if (_camera != null) _camera.NotifyVehicleWarped(delta);

        _resetCount++;

        // t is Time.unscaledTime deliberately: it is MotionTrace's clock, so
        // this line locates the corrupted rows in the CSV. See the class notes.
        Debug.Log($"[PlaytestReset] RESET {_resetCount} at t={Time.unscaledTime:F2}s, " +
                  $"moved {delta.magnitude:F1}m. Motion rows near this time are teleport " +
                  "artefacts, not physics.", this);
    }
}
