using UnityEngine;

/// <summary>
/// TuningTarget v1.0
/// -----------------
/// Turns a vehicle into a shooting-range target you do not have to chase.
///
/// The problem it solves: a craft hit by a full-force rocket travels about 100 metres, and a
/// five-missile volley threw the AI over 300. That is the knockback working correctly, and it
/// makes iterating on the numbers miserable — every shot is followed by a long drive to find
/// where the target ended up, and by then you have forgotten what the hit looked like.
///
/// So: let it take the hit and tumble, which is the thing being judged, then put it back.
///
/// Attach to a vehicle root. Does nothing at all in Free mode, which is the default, so this
/// is safe to leave on a prefab.
/// </summary>
[DefaultExecutionOrder(50)]
public class TuningTarget : MonoBehaviour
{
    public enum TargetMode
    {
        /// <summary>Untouched. Behaves exactly as if this component were not here.</summary>
        Free,

        /// <summary>Takes the hit, tumbles, then returns to its mark once it settles.</summary>
        AutoReturn,

        /// <summary>Pinned in place. Rotates and tilts, but never travels.</summary>
        Anchored
    }

    [Header("Mode")]
    [Tooltip("Free: normal vehicle, gets knocked across the map. This component does nothing.\n\n" +
             "AutoReturn: takes the hit and tumbles properly, then flies back to its mark once " +
             "it has settled. Use this for tuning FORCE, where watching the knockback is the " +
             "whole point but chasing it afterwards is not.\n\n" +
             "Anchored: still tips and spins, but never travels. Use this for tuning FLIGHT " +
             "PATHS and aim, where you want the target to stay exactly where it was so every " +
             "shot is the same shot.")]
    public TargetMode mode = TargetMode.Free;

    [Header("Auto return")]
    [Tooltip("Seconds to let the hit play out before returning. Long enough to watch the tumble " +
             "and the recovery; short enough that you are not waiting around between shots.")]
    [Min(0.5f)]
    public float watchSeconds = 4f;

    [Tooltip("Also restore full health on return, so a long tuning session never ends with a " +
             "dead target and a stubbed death path.")]
    public bool refillHealthOnReturn = true;

    [Header("Manual")]
    [Tooltip("Puts the target back immediately, in any mode. Handy when a shot sends it " +
             "somewhere unexpected and you do not want to wait out the timer.")]
    public KeyCode resetKey = KeyCode.T;

    private Rigidbody _body;
    private VehicleHealth _health;
    private Vector3 _homePosition;
    private Quaternion _homeRotation;
    private float _disturbedAt = -1f;

    private void Awake()
    {
        _body   = GetComponent<Rigidbody>();
        _health = GetComponent<VehicleHealth>();

        // Captured at Awake rather than serialized, so moving the target in the scene just
        // works and the mark is always wherever you last put it.
        _homePosition = transform.position;
        _homeRotation = transform.rotation;
    }

    private void Update()
    {
        if (Input.GetKeyDown(resetKey)) ReturnHome();
    }

    private void FixedUpdate()
    {
        if (mode == TargetMode.Free || _body == null) return;

        if (mode == TargetMode.Anchored)
        {
            // Position only. Angular velocity is deliberately untouched, so the target still
            // tips, rolls and goes down exactly as it would — which is most of what a knockback
            // pass is judging. It simply does not travel while doing it.
            _body.position        = _homePosition;
            _body.linearVelocity  = Vector3.zero;
            return;
        }

        // AutoReturn: notice a disturbance, let it play out, then go home.
        bool disturbed = _body.linearVelocity.sqrMagnitude > 4f
                      || _body.angularVelocity.sqrMagnitude > 1f;

        if (disturbed && _disturbedAt < 0f) _disturbedAt = Time.time;

        if (_disturbedAt >= 0f && Time.time - _disturbedAt >= watchSeconds)
            ReturnHome();
    }

    /// <summary>Puts the target back on its mark, upright, still, and optionally healed.</summary>
    public void ReturnHome()
    {
        _disturbedAt = -1f;
        if (_body == null) return;

        _body.linearVelocity  = Vector3.zero;
        _body.angularVelocity = Vector3.zero;
        _body.position        = _homePosition;
        _body.rotation        = _homeRotation;

        // VehicleHealth.Respawn is the existing public restore: full HP and the GameObject
        // re-enabled. Using it rather than adding anything, and it also recovers the target
        // if a long session ever did manage to kill it — VehicleHealth DISABLES the object on
        // death, so without this the target would simply vanish and look like a bug.
        if (refillHealthOnReturn && _health != null) _health.Respawn();
    }

    /// <summary>Re-marks the current spot as home. Call after deliberately repositioning.</summary>
    public void SetHomeHere()
    {
        _homePosition = transform.position;
        _homeRotation = transform.rotation;
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (mode == TargetMode.Free) return;
        Vector3 home = Application.isPlaying ? _homePosition : transform.position;

        Gizmos.color = mode == TargetMode.Anchored
            ? new Color(1f, 0.5f, 0f, 0.9f)
            : new Color(0.3f, 1f, 0.5f, 0.9f);
        Gizmos.DrawWireSphere(home, 1.5f);
        Gizmos.DrawLine(home, home + Vector3.up * 6f);

        if (Application.isPlaying && Vector3.Distance(transform.position, home) > 2f)
            Gizmos.DrawLine(transform.position, home);
    }
#endif
}
