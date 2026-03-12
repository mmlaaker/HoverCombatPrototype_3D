using UnityEngine;
using Unity.Cinemachine;
using Unity.Cinemachine.TargetTracking;

/// <summary>
/// HoverCameraController v1.0
/// ------------------------------------------------
/// Manages all camera behavior for the hover vehicle using two Cinemachine
/// Virtual Cameras blended by a CinemachineBrain on the main camera.
///
/// Drive Mode (default):
///   • Third-person chase camera behind the vehicle.
///   • Right Stick Y pitches the camera up and down within clamped limits.
///   • During hard turns / drift, camera shifts to the opposite shoulder
///     and looks slightly ahead. Driven by Propulsion's driftLerp value.
///   • Camera springs back to neutral pitch when right stick is released.
///
/// Strafe Mode (Left Trigger held):
///   • Camera locks directly behind the vehicle, no shoulder shift.
///   • Slight zoom in (reduced follow distance + FOV reduction).
///   • Blends back to Drive Mode on trigger release.
///
/// Cinemachine 3.x Scene Setup (required):
///   1. Main Camera: add CinemachineBrain component.
///      • Default Blend: EaseInOut, 0.2s
///      • Update Method: SmartUpdate
///   2. Create two CinemachineCamera GameObjects as children of the
///      camera rig or scene root (NOT children of the vehicle).
///      Name them exactly: "VCam_Drive" and "VCam_Strafe"
///
///   VCam_Drive settings:
///      • Add component: CinemachineFollow (Body)
///        - Follow Offset: (0, 3, -8)      ← adjust per vehicle scale
///        - Damping: X 0.1, Y 0.2, Z 0.1
///      • Add component: CinemachineRotationComposer (Aim)
///        - Tracked Object Offset: (0, 1, 0)
///        - Dead Zone: 0, Soft Zone: 0.8
///      • Priority: 10
///
///   VCam_Strafe settings:
///      • Add component: CinemachineFollow (Body)
///        - Follow Offset: (0, 2.5, -6)    ← tighter than drive
///        - Damping: same as drive
///      • Add component: CinemachineRotationComposer (Aim, same as drive)
///      • Lens: FOV reduce by 5-8 degrees vs drive cam
///      • Priority: 10
///
///   3. Assign both VCams to this component in the Inspector.
///   4. Assign the vehicle Transform as vehicleTarget.
///   5. Assign inputProvider (same MonoBehaviour as Propulsion uses).
///   6. Assign propulsion reference for driftLerp access.
///
/// Priority management:
///   Both VCams sit at priority 10. The active one is bumped to 11.
///   The brain blends between them on priority changes.
/// </summary>
public class HoverCameraController : MonoBehaviour
{
    // ── References ────────────────────────────────────────────────────────

    [Header("Cinemachine Virtual Cameras")]
    [Tooltip("The drive-mode virtual camera. See class header for Inspector setup.")]
    [SerializeField] private CinemachineCamera vcamDrive;

    [Tooltip("The strafe-mode virtual camera. See class header for Inspector setup.")]
    [SerializeField] private CinemachineCamera vcamStrafe;

    [Header("Vehicle")]
    [Tooltip("The vehicle root transform. Both VCams follow this.")]
    [SerializeField] private Transform vehicleTarget;

    [Header("Input + Modules")]
    [Tooltip("MonoBehaviour implementing IHoverInputProvider.")]
    [SerializeField] private MonoBehaviour inputProvider;

    [Tooltip("Propulsion module — read driftLerp for shoulder shift.")]
    [SerializeField] private HoverController_Propulsion propulsion;

    // ── Drive Mode: Camera Pitch ──────────────────────────────────────────

    [Header("Drive Mode — Camera Pitch")]
    [Tooltip("Sensitivity of right stick Y axis for camera pitch control.")]
    [Range(0.5f, 5f)]
    [SerializeField] private float pitchSensitivity = 2f;

    [Tooltip("Minimum pitch angle (degrees). Negative = look down.")]
    [SerializeField] private float minPitch = -10f;

    [Tooltip("Maximum pitch angle (degrees). Positive = look up.")]
    [SerializeField] private float maxPitch = 35f;

    [Tooltip("Neutral pitch the camera returns to when stick is released.")]
    [SerializeField] private float neutralPitch = 12f;

    [Tooltip("Speed at which pitch springs back to neutral when stick released.")]
    [Range(1f, 10f)]
    [SerializeField] private float pitchRecenterSpeed = 4f;

    // ── Drive Mode: Shoulder Shift ────────────────────────────────────────

    [Header("Drive Mode — Shoulder Shift")]
    [Tooltip("Horizontal offset of the camera at full drift (opposite shoulder). " +
             "Positive = right shoulder offset when turning left.")]
    [SerializeField] private float shoulderShiftAmount = 1.8f;

    [Tooltip("Forward look-ahead offset added at full drift. " +
             "Shifts look target ahead of vehicle during turns.")]
    [SerializeField] private float shoulderLookAheadAmount = 2.5f;

    [Tooltip("Speed at which shoulder offset lerps in and out. " +
             "Matched to Propulsion's driftBlendSeconds for feel consistency.")]
    [Range(1f, 15f)]
    [SerializeField] private float shoulderShiftLerpSpeed = 8f;

    // ── Strafe Mode: Zoom ─────────────────────────────────────────────────

    [Header("Strafe Mode — Zoom")]
    [Tooltip("Follow distance reduction when entering strafe mode (metres). " +
             "Subtracted from VCam_Strafe's follow offset Z.")]
    [SerializeField] private float strafeZoomDistance = 1.5f;

    [Tooltip("FOV reduction (degrees) in strafe mode for subtle zoom feel.")]
    [Range(0f, 15f)]
    [SerializeField] private float strafeFovReduction = 5f;

    [Tooltip("Blend time (seconds) for strafe mode entry/exit. " +
             "Overrides CinemachineBrain default for this specific transition.")]
    [Range(0.05f, 0.5f)]
    [SerializeField] private float strafeModeBlendTime = 0.18f;

    // ── Private State ─────────────────────────────────────────────────────

    private IHoverInputProvider _input;

    // Drive mode camera pitch (degrees, clamped)
    private float _currentPitch;

    // Smoothed shoulder shift offset (X axis, world-relative to vehicle)
    private float _shoulderOffset;

    // Tracks previous strafe state for transition detection
    private bool _wasStrafingLastFrame;

    // Cached component references — avoid GetComponent per frame
    private CinemachineFollow _driveTransposer;
    private CinemachineFollow _strafeTransposer;

    // Cached rotation composer for drive cam look offset
    private CinemachineRotationComposer _driveComposer;

    // Base follow offsets read from Inspector at startup — never modified directly
    private Vector3 _driveBaseOffset;
    private Vector3 _strafeBaseOffset;

    // Base FOV values read at startup
    private float _driveFov;
    private float _strafeFov;

    // ── Unity Lifecycle ───────────────────────────────────────────────────

    private void Awake()
    {
        _input = inputProvider as IHoverInputProvider;

        if (_input == null)
        {
            Debug.LogError("[HoverCameraController] inputProvider is null or missing IHoverInputProvider.", this);
            enabled = false;
            return;
        }

        if (vcamDrive == null || vcamStrafe == null)
        {
            Debug.LogError("[HoverCameraController] One or both VCam references are unassigned.", this);
            enabled = false;
            return;
        }

        if (vehicleTarget == null)
        {
            Debug.LogError("[HoverCameraController] vehicleTarget is unassigned.", this);
            enabled = false;
            return;
        }

        if (propulsion == null)
        {
            Debug.LogWarning("[HoverCameraController] Propulsion reference unassigned. " +
                             "Shoulder shift via driftLerp will be disabled.", this);
        }

        // Assign follow/look targets on both VCams
        vcamDrive.Follow  = vehicleTarget;
        vcamDrive.LookAt  = vehicleTarget;
        vcamStrafe.Follow = vehicleTarget;
        vcamStrafe.LookAt = vehicleTarget;

        // Cache component references — v3 uses GetComponent, not GetCinemachineComponent
        _driveTransposer  = vcamDrive.GetComponent<CinemachineFollow>();
        _strafeTransposer = vcamStrafe.GetComponent<CinemachineFollow>();
        _driveComposer    = vcamDrive.GetComponent<CinemachineRotationComposer>();

        if (_driveTransposer == null || _strafeTransposer == null)
        {
            Debug.LogError("[HoverCameraController] VCams must have a CinemachineFollow component. " +
                           "Add CinemachineFollow to both VCam GameObjects.", this);
            enabled = false;
            return;
        }

        // Cache base values — these are the Inspector-authored offsets
        _driveBaseOffset  = _driveTransposer.FollowOffset;
        _strafeBaseOffset = _strafeTransposer.FollowOffset;
        _driveFov         = vcamDrive.Lens.FieldOfView;
        _strafeFov        = Mathf.Max(20f, _driveFov - strafeFovReduction);

        // Drive cam: LockToTargetWithWorldUp — follows yaw only, ignores pitch and roll.
        // Correct for a chase cam where the vehicle tilts but the camera stays level.
        var driveSettings  = _driveTransposer.TrackerSettings;
        driveSettings.BindingMode = BindingMode.LockToTargetWithWorldUp;
        _driveTransposer.TrackerSettings = driveSettings;

        // Strafe cam: LockToTargetNoRoll — follows yaw AND pitch, ignores roll only.
        // Required so the camera physically tilts with the vehicle nose in strafe mode.
        // LockToTargetWithWorldUp would ignore pitch entirely, leaving the camera flat.
        var strafeSettings = _strafeTransposer.TrackerSettings;
        strafeSettings.BindingMode = BindingMode.LockToTargetNoRoll;
        _strafeTransposer.TrackerSettings = strafeSettings;

        // Apply strafe FOV immediately (strafe cam starts inactive but set it now)
        var strafeLens = vcamStrafe.Lens;
        strafeLens.FieldOfView = _strafeFov;
        vcamStrafe.Lens = strafeLens;

        // Initialize pitch to neutral
        _currentPitch = neutralPitch;

        // Drive cam active at start
        SetDriveCamActive();
    }

    private void LateUpdate()
    {
        if (_input == null) return;

        bool strafing = _input.StrafeHeld;

        // ── Mode transitions ──────────────────────────────────────────────
        if (strafing != _wasStrafingLastFrame)
        {
            if (strafing) SetStrafeCamActive();
            else          SetDriveCamActive();
        }
        _wasStrafingLastFrame = strafing;

        // ── Per-frame updates ─────────────────────────────────────────────
        if (!strafing)
        {
            UpdateDriveCameraPitch();
            UpdateDriveShoulderShift();
        }
        // Strafe cam is fully static relative to vehicle — no per-frame adjustments needed
    }

    // ── Drive Mode: Camera Pitch ──────────────────────────────────────────

    /// <summary>
    /// Right Stick Y controls camera pitch in drive mode.
    /// When stick is released (near zero), pitch springs back to neutralPitch.
    /// Applied by adjusting the VCam_Drive follow offset Y component.
    /// </summary>
    private void UpdateDriveCameraPitch()
    {
        float lookY = _input.CameraLookY;

        if (Mathf.Abs(lookY) > 0.05f)
        {
            // Stick is active — accumulate pitch
            _currentPitch += lookY * pitchSensitivity * Time.deltaTime * 60f;
            _currentPitch  = Mathf.Clamp(_currentPitch, minPitch, maxPitch);
        }
        else
        {
            // Stick released — spring back to neutral
            _currentPitch = Mathf.Lerp(_currentPitch, neutralPitch,
                                        pitchRecenterSpeed * Time.deltaTime);
        }

        // Apply pitch by modifying the follow offset Y on the drive transposer
        // Pitch is expressed as a vertical offset change — higher Y = looking down more
        // We convert pitch degrees to a Y offset using the base distance as radius
        float baseDistance = Mathf.Abs(_driveBaseOffset.z);
        float pitchRad     = _currentPitch * Mathf.Deg2Rad;

        Vector3 pitchedOffset = _driveBaseOffset;
        pitchedOffset.y = baseDistance * Mathf.Sin(pitchRad);
        pitchedOffset.z = -baseDistance * Mathf.Cos(pitchRad);

        _driveTransposer.FollowOffset = new Vector3(
            _driveTransposer.FollowOffset.x, // preserve shoulder shift X
            pitchedOffset.y,
            pitchedOffset.z
        );
    }

    // ── Drive Mode: Shoulder Shift ────────────────────────────────────────

    /// <summary>
    /// During hard turns/drift, the camera shifts to the opposite shoulder
    /// and the look target moves slightly forward.
    ///
    /// Driven by Propulsion.DriftLerp — the same smooth 0..1 value that
    /// controls chassis bank. Negative turn input = drift right = shift left.
    ///
    /// Shoulder shift is applied as a local X offset on the drive transposer.
    /// Look-ahead is applied via the Composer's tracked object offset Z.
    /// </summary>
    private void UpdateDriveShoulderShift()
    {
        if (propulsion == null) return;

        float driftLerp = propulsion.DriftLerp;
        float turnSign  = Mathf.Sign(_input.TurnInput);

        // Opposite shoulder: turning right → camera shifts left (negative X)
        float targetShoulderX = -turnSign * shoulderShiftAmount * driftLerp;

        _shoulderOffset = Mathf.Lerp(
            _shoulderOffset,
            targetShoulderX,
            shoulderShiftLerpSpeed * Time.deltaTime
        );

        // Apply X offset — preserve Y and Z from pitch calculation
        var currentOffset = _driveTransposer.FollowOffset;
        _driveTransposer.FollowOffset = new Vector3(
            _shoulderOffset,
            currentOffset.y,
            currentOffset.z
        );

        // Look-ahead via rotation composer tracked object offset
        if (_driveComposer != null)
        {
            float lookAhead = shoulderLookAheadAmount * driftLerp;
            _driveComposer.TargetOffset = new Vector3(0f, 1f, lookAhead);
        }
    }

    // ── Mode Switching ────────────────────────────────────────────────────

    /// <summary>
    /// Activates the drive camera by giving it priority advantage.
    /// CinemachineBrain handles the blend automatically.
    /// </summary>
    private void SetDriveCamActive()
    {
        vcamDrive.Priority  = 11;
        vcamStrafe.Priority = 10;

        // Restore drive damping to loose chase feel when returning from strafe.
        var s = _driveTransposer.TrackerSettings;
        s.PositionDamping = new Vector3(0.1f, 0.2f, 0.1f);
        s.RotationDamping = Vector3.zero;
        _driveTransposer.TrackerSettings = s;
    }

    /// <summary>
    /// Activates the strafe camera. Zeros all damping so the camera is
    /// bolted to vehicle heading with zero lag — FPS-style lock.
    /// </summary>
    private void SetStrafeCamActive()
    {
        vcamStrafe.Priority = 11;
        vcamDrive.Priority  = 10;
        _shoulderOffset     = 0f;

        // Zero all damping — no lag, no float. Snaps instantly to vehicle heading.
        // LockToTargetNoRoll: follows yaw AND pitch, ignores roll only.
        // This is what lets the camera physically tilt with the vehicle nose.
        var s = _strafeTransposer.TrackerSettings;
        s.BindingMode     = BindingMode.LockToTargetNoRoll;
        s.PositionDamping = Vector3.zero;
        s.RotationDamping = Vector3.zero;
        _strafeTransposer.TrackerSettings = s;
    }

    // ── Public API ────────────────────────────────────────────────────────

    /// <summary>
    /// True when strafe camera is active. Read by UI or other systems
    /// that need to know current camera mode.
    /// </summary>
    public bool IsInStrafeMode => _input != null && _input.StrafeHeld;
}
