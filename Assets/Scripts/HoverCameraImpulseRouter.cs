using System.Collections.Generic;
using Unity.Cinemachine;
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// Owns every camera punch on the vehicle. Subscribes to the gameplay events that
/// deserve one and fires the matching Cinemachine impulse channel.
///
/// Was HoverLandingCameraImpulse, which only knew about landings. Renamed in place
/// (file and .meta moved together, so the script GUID and every authored value
/// survive) once it took ownership of crashes, denied jumps, the EMP and recoil.
///
/// WHERE IT LIVES: on the Camera object, under HoverCameraController, with its
/// five sources as children of the same object. Everything that moves the camera
/// is therefore in one place in the hierarchy. It used to sit on the vehicle root
/// next to the controllers.
///
/// **Consequence of that move, and the reason VehicleCollisionRelay exists:**
/// Unity delivers OnCollisionEnter to the GameObject that owns the Rigidbody, and
/// nothing else. A component on a child never sees it. So crash detection cannot
/// live here any more; a one-method relay sits on the vehicle root and forwards
/// each impact to ReportCollision below. That is the only thing the move cost.
///
/// **And the reason vehicleRoot is a serialized field:** every gameplay signal and
/// every direction this class needs belongs to the VEHICLE, not to the camera rig.
/// `transform.forward` here would be the camera's heading, which is a different
/// vector and a silent one, so nothing reads `transform` at all.
///
/// WHY THIS IS SEPARATE FROM HoverCameraController, and must stay that way even
/// now that they share a GameObject: impulse and framing are different stages.
/// CinemachineImpulseListener runs post-body on the vcams, downstream of the
/// framing stage, so an impulse can never fight HoverCameraController's per-frame
/// FollowOffset writes. Framing needed a single-write-authority refactor precisely
/// because its contributors DID fight. Shake never had that problem, and folding
/// the two together would import the problem rather than solve it.
///
/// WHY FIVE SOURCES AND NOT ONE: the impulse shape and duration are authored on
/// the CinemachineImpulseSource component, not passed at fire time. Only the
/// velocity vector is a per-call argument. So a source IS a channel, and one
/// channel can only ever have one character. All five fire on impulse channel 1,
/// which is what both vcam listeners are masked to (drive gain 1, strafe gain
/// 0.7), so every punch is felt in both camera modes at the strafe camera's
/// reduced strength.
///
/// TUNING A CHANNEL: only three things do anything, and the inspector shows more
/// than three. On the source, it is IMPULSE SHAPE and IMPULSE DURATION. Here, it
/// is the velocity for that channel. That is the whole surface.
///
/// **The Time Envelope block on the source is dead, along with Raw Signal, Repeat
/// Mode and Randomize.** They belong to Cinemachine's legacy signal path, which
/// `LegacyCreateAndReturnEvent` abandons on its first line when `RawSignal` is
/// null, and it is null on all five sources. The live path builds its envelope as
/// `SustainTime = ImpulseDuration` and ignores the rest. Attack, sustain, decay
/// and Scale With Impact can be dragged all day with no effect, so do not spend an
/// afternoon there. Confirmed by measurement: every channel's tail matches its
/// Impulse Duration and not its authored envelope length.
/// </summary>
public class HoverCameraImpulseRouter : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 🚗 Vehicle
    // -------------------------------------------------------------------------
    [Header("🚗 Vehicle")]
    [Tooltip("The vehicle root. Every event this class listens to, and every direction it throws " +
             "the camera along, comes from here rather than from this GameObject.\n\n" +
             "This is not optional convenience wiring. Sitting on the camera rig means " +
             "transform.forward is the CAMERA's heading, so reading it would aim recoil down the " +
             "view axis instead of down the chassis, and would do it silently.")]
    [SerializeField] private Transform vehicleRoot;

    // -------------------------------------------------------------------------
    // 💥 Impulse Sources
    // -------------------------------------------------------------------------
    [Header("💥 Impulse Sources")]
    [Tooltip("Hard landing channel. Bump over 0.25s. This envelope is the punch the owner signed " +
             "off on; it was copied field for field off the original source and verified after the " +
             "move, so treat any change here as a change to something already approved.")]
    [SerializeField] private CinemachineImpulseSource hardLandingSource;

    [Tooltip("Crash channel. Started life as an exact copy of the hard landing envelope and is now " +
             "free to diverge, which is the point of splitting them: a wall you drove into and a " +
             "floor you fell onto have no reason to share a shape.")]
    [SerializeField] private CinemachineImpulseSource crashSource;

    [Tooltip("Denied jump channel. Bump over 0.18s. Wants to stay SHORT and small: this is a " +
             "failure tick, not an impact, and anything long enough to notice properly reads as " +
             "having hit something.")]
    [SerializeField] private CinemachineImpulseSource jumpDeniedSource;

    [Tooltip("Weapon recoil channel. Shared by every weapon that opts in; only the STRENGTH is per " +
             "weapon, authored on WeaponDefinition > Combat > Recoil Velocity. Its own source so " +
             "recoil can be shaped for repeat fire without dragging the denied-jump tick along.")]
    [SerializeField] private CinemachineImpulseSource weaponRecoilSource;

    [Tooltip("EMP channel. Recoil over 0.22s, a one-way shove rather than a bump, because a launch " +
             "goes one way.")]
    [SerializeField] private CinemachineImpulseSource empSource;

    // -------------------------------------------------------------------------
    // 🛬 Hard Landing
    // -------------------------------------------------------------------------
    [Header("🛬 Hard Landing")]
    [Tooltip("Impulse velocity magnitude at severity 1. The shape and duration are authored on the " +
             "hard landing source. Try 1.5 to 3.")]
    [FormerlySerializedAs("maxImpulseVelocity")]
    [Min(0f)]
    [SerializeField] private float landingMaxVelocity = 2f;

    // -------------------------------------------------------------------------
    // 🧱 Collisions
    // -------------------------------------------------------------------------
    [Header("🧱 Collisions")]
    [Tooltip("Master switch for crash shake. Off restores the pre-router behaviour, where wall and " +
             "vehicle impacts produced no camera feedback at all.")]
    [SerializeField] private bool collisionShakeEnabled = true;

    [Tooltip("Impact speed below which a collision is ignored, in m/s.\n\n" +
             "This is a genuine speed, not an impulse in N*s: the raw collision impulse is divided " +
             "by the chassis mass, which turns it into the velocity the impact actually removed. " +
             "That makes it the same unit as Hard Landing Min Speed and it stops meaning something " +
             "different the moment anyone edits the Rigidbody mass.\n\n" +
             "8 clears scrapes and kerb nudges while still catching a real shunt. Turn on Log " +
             "Collisions below to read live values off the console and pick this from data.")]
    [Min(0f)]
    [SerializeField] private float collisionMinSpeed = 8f;

    [Tooltip("Impact speed that reads at full severity, in m/s.\n\n" +
             "Picked from a measured crash rather than from top speed, because the two are not the " +
             "same number. A 45 m/s head-on into a static wall removes only 27.1 m/s: the rest goes " +
             "into bounce and slide. Impact speed lands around 0.6 of approach speed, so a top-speed " +
             "60 m/s head-on measures near 36. Setting this to the 60 it looks like it should be " +
             "would make full severity unreachable and bunch every real crash in the lower half.")]
    [Min(0f)]
    [SerializeField] private float collisionMaxSpeed = 35f;

    [Tooltip("Impulse velocity magnitude at collision severity 1. Kept a little above the landing " +
             "value by default: a wall you drove into should hit harder than a floor you fell onto.")]
    [Min(0f)]
    [SerializeField] private float collisionMaxVelocity = 2.5f;

    [Tooltip("Surfaces this flat or flatter are treated as floor and produce no crash shake, " +
             "because a landing on them is already the hard-landing path's job. Without this " +
             "filter every hard landing fires twice, once from OnHardLanding and once from the " +
             "collision that caused it.\n\n" +
             "Mirrors the idea behind Foundation's Unstick Max Surface Angle (also 60) but is " +
             "deliberately a separate number, so tuning the shake can never perturb unstick.")]
    [Range(0f, 90f)]
    [SerializeField] private float collisionFloorMaxAngle = 60f;

    [Tooltip("Minimum gap between crash shakes. Scraping along a wall re-establishes contact " +
             "repeatedly and would otherwise machine-gun the camera.")]
    [Min(0f)]
    [SerializeField] private float collisionCooldown = 0.2f;

    [Tooltip("Crash shake is suppressed for this long after a hard landing punch. The angle filter " +
             "above already catches the flat case; this is the backstop for the one it cannot, " +
             "which is slamming down onto a bank steeper than Collision Floor Max Angle. There the " +
             "contact is legitimately not floor-like, so only the timestamp separates the two.")]
    [Min(0f)]
    [SerializeField] private float landingSuppressWindow = 0.3f;

    // -------------------------------------------------------------------------
    // 🚫 Denied Jump
    // -------------------------------------------------------------------------
    [Header("🚫 Denied Jump")]
    [Tooltip("Impulse velocity when a grounded jump is refused for lack of energy.\n\n" +
             "Deliberately small and slightly unpleasant. Energy as Tempo is a design pillar and " +
             "running dry currently produces no feedback whatsoever: the button does nothing and " +
             "the player is left to infer why. With no audio system in the project this punch is " +
             "the entire failure signal, so it needs to be legible without being satisfying.")]
    [Min(0f)]
    [SerializeField] private float deniedJumpGroundedVelocity = 0.35f;

    [Tooltip("Impulse velocity when an AIR jump is refused. Smaller than the grounded value: you " +
             "are already committed to the arc and a hard jolt mid-flight reads as an impact that " +
             "did not happen. The air-jump token is not consumed on denial, so this also has to " +
             "avoid reading as final.")]
    [Min(0f)]
    [SerializeField] private float deniedJumpAirVelocity = 0.25f;

    // -------------------------------------------------------------------------
    // ⚡ EMP
    // -------------------------------------------------------------------------
    [Header("⚡ EMP")]
    [Tooltip("Impulse velocity on EMP launch, thrown backward along the chassis. At 70 of 100 " +
             "energy this is the largest single commitment in the game and currently the quietest.")]
    [Min(0f)]
    [SerializeField] private float empVelocity = 1.2f;

    // -------------------------------------------------------------------------
    // 🔫 Weapon Recoil
    // -------------------------------------------------------------------------
    [Header("🔫 Weapon Recoil")]
    [Tooltip("Master switch for per-weapon recoil. On is safe: the strength is authored per weapon " +
             "on WeaponDefinition > Combat > Recoil Velocity and defaults to 0, so nothing kicks " +
             "until a weapon opts in. This exists to kill every gun's recoil at once while judging " +
             "something else, not as the opt-in itself.")]
    [SerializeField] private bool weaponRecoilEnabled = true;

    [Tooltip("Multiplies every weapon's authored recoil. Lets the whole set move together during " +
             "tuning without editing six WeaponDefinition assets one at a time.")]
    [Min(0f)]
    [SerializeField] private float weaponRecoilScale = 1f;

    // -------------------------------------------------------------------------
    // 🛠 Debug
    // -------------------------------------------------------------------------
    [Header("🛠 Debug")]
    [Tooltip("Severity used by the F9 hard-landing and F10 crash test hotkeys, and by the " +
             "context-menu items. 0..1, the same scale Foundation passes on a real landing. " +
             "Editor only, play mode only.")]
    [Range(0f, 1f)]
    [SerializeField] private float testSeverity = 1f;

    [Tooltip("Logs the measured impact speed and the resulting severity for every collision, " +
             "including the ones filtered out and why. This is how Collision Min Speed and " +
             "Collision Max Speed get picked from real crashes instead of guessed.")]
    [SerializeField] private bool logCollisions = false;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private HoverController_Foundation foundation;
    private HoverController_Propulsion propulsion;
    private HoverController_EMP        emp;
    private HoverController_Weapons    weapons;
    private Rigidbody                  vehicleBody;

    // Pre-allocated contact buffer, matching Foundation's OnCollisionStay pattern.
    // Collision.GetContacts(List<T>) fills a caller-owned list; the ContactPoint[]
    // overload allocates on every impact.
    private readonly List<ContactPoint> _contactBuffer = new(8);

    // Timestamps rather than booleans, per the project convention: a boolean needs
    // a matching clear callback to be correct, a timestamp just goes stale.
    private float _lastCollisionShakeTime = float.NegativeInfinity;
    private float _lastLandingPunchTime   = float.NegativeInfinity;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        if (vehicleRoot == null)
        {
            Debug.LogError("[ImpulseRouter] Vehicle Root unassigned. Every channel is dead: this " +
                           "component sits on the camera rig and has no other way to reach the " +
                           "vehicle's events.", this);
            return;
        }

        foundation  = vehicleRoot.GetComponent<HoverController_Foundation>();
        propulsion  = vehicleRoot.GetComponent<HoverController_Propulsion>();
        emp         = vehicleRoot.GetComponent<HoverController_EMP>();
        weapons     = vehicleRoot.GetComponent<HoverController_Weapons>();
        vehicleBody = vehicleRoot.GetComponent<Rigidbody>();

        // Explicit refs, so a channel that was never wired says so once at startup
        // rather than silently swallowing every punch on it for the whole session.
        WarnIfUnwired(hardLandingSource,  "Hard landing");
        WarnIfUnwired(crashSource,        "Crash");
        WarnIfUnwired(jumpDeniedSource,   "Denied jump");
        WarnIfUnwired(weaponRecoilSource, "Weapon recoil");
        WarnIfUnwired(empSource,          "EMP");
    }

    private void WarnIfUnwired(CinemachineImpulseSource source, string channel)
    {
        if (source == null)
            Debug.LogWarning($"[ImpulseRouter] {channel} source unassigned. That channel will not " +
                             "shake the camera.", this);
    }

    private void OnEnable()
    {
        if (foundation != null) foundation.OnHardLanding += HandleHardLanding;
        if (propulsion != null) propulsion.OnJumpDenied  += HandleJumpDenied;
        if (emp        != null) emp.OnEmpFired           += HandleEmpFired;
        if (weapons    != null) weapons.OnWeaponFired    += HandleWeaponFired;
    }

    private void OnDisable()
    {
        if (foundation != null) foundation.OnHardLanding -= HandleHardLanding;
        if (propulsion != null) propulsion.OnJumpDenied  -= HandleJumpDenied;
        if (emp        != null) emp.OnEmpFired           -= HandleEmpFired;
        if (weapons    != null) weapons.OnWeaponFired    -= HandleWeaponFired;
    }

    // -------------------------------------------------------------------------
    // 🛬 Hard landing
    // -------------------------------------------------------------------------
    /// <summary>
    /// Called by Foundation.OnHardLanding. Punch along the impact axis, scaled by
    /// severity. Behaviourally identical to the pre-router version; the only added
    /// line stamps the suppression window that keeps the crash path off this event.
    ///
    /// **This one deliberately skips ToScreenSpace, and that inconsistency is the
    /// point.** The listener reads its vector as a screen direction, so passing a
    /// world down-vector raw produces a punch along the CAMERA's down axis, which
    /// with the camera sitting about 27 degrees above the craft is a down-and-back
    /// shove rather than a straight drop. That is the landing punch the owner likes
    /// and signed off on. Routing it through the converter would make it a true
    /// world-down punch, which is a different feel, so C7 forbids it. Every other
    /// channel converts, because for them the raw behaviour was simply wrong.
    /// </summary>
    private void HandleHardLanding(float severity)
    {
        _lastLandingPunchTime = Time.time;
        Fire(hardLandingSource, ImpactDirection * (landingMaxVelocity * severity));
    }

    /// <summary>
    /// Direction the camera is thrown on impact: into the ground, along the
    /// negated ground normal. This is the same axis Foundation measures impact
    /// speed on, so a landing on a bank punches along the bank instead of
    /// world-down. Foundation assigns the normal on the grounded edge one line
    /// before it fires OnHardLanding, so the read here is the landing frame's.
    ///
    /// Falls back to world-down without a Foundation; Foundation itself reports
    /// Vector3.up while airborne, so the two agree in the air anyway.
    /// </summary>
    private Vector3 ImpactDirection =>
        foundation != null ? -foundation.AverageGroundNormal : Vector3.down;

    // -------------------------------------------------------------------------
    // 🧱 Collisions
    // -------------------------------------------------------------------------
    /// <summary>
    /// Crash shake. Called by VehicleCollisionRelay on the vehicle root, because
    /// Unity delivers OnCollisionEnter only to the GameObject owning the Rigidbody
    /// and this component lives on the camera rig.
    ///
    /// Enter fires per COLLIDER PAIR, which is what makes the floor filter below
    /// workable: the arena is authored as many separate meshes (road, walls, loop,
    /// 8track, the mountains), so hitting a wall while driving on the road is its
    /// own callback carrying only wall contacts, and averaging them yields a clean
    /// horizontal normal rather than a floor-contaminated diagonal.
    ///
    /// Strength comes from Collision.impulse.magnitude only. Its DIRECTION is not
    /// used, deliberately: the vector is documented as running from the first body
    /// to the second, so which way it points depends on collider ordering rather
    /// than on anything about the crash. The contact normals say the same thing
    /// without the ambiguity, and they are already being read for the filter.
    /// </summary>
    public void ReportCollision(Collision collision)
    {
        if (!collisionShakeEnabled || vehicleBody == null || collision == null) return;

        if (Time.time - _lastCollisionShakeTime < collisionCooldown) return;
        if (Time.time - _lastLandingPunchTime   < landingSuppressWindow) return;

        int count = collision.GetContacts(_contactBuffer);
        if (count == 0) return;

        Vector3 normalSum = Vector3.zero;
        for (int i = 0; i < count; i++)
            normalSum += _contactBuffer[i].normal;

        Vector3 normal = normalSum.normalized;

        // Dividing the impulse by mass converts N*s into the m/s the impact
        // actually removed, which is both the unit the thresholds are authored in
        // and the one the hard-landing path already speaks.
        float impactSpeed = collision.impulse.magnitude / vehicleBody.mass;

        bool isFloor = normal.y >= Mathf.Cos(collisionFloorMaxAngle * Mathf.Deg2Rad);

        if (logCollisions)
            Debug.Log($"[ImpulseRouter] Collision with {collision.gameObject.name}: " +
                      $"{impactSpeed:F1} m/s, normal.y {normal.y:F2}, " +
                      (isFloor      ? "IGNORED (floor-like, hard landing owns it)"
                     : impactSpeed < collisionMinSpeed ? "IGNORED (below Collision Min Speed)"
                                                       : "shaking"), this);

        if (isFloor) return;
        if (impactSpeed < collisionMinSpeed) return;

        float severity = Mathf.InverseLerp(collisionMinSpeed, collisionMaxSpeed, impactSpeed);

        _lastCollisionShakeTime = Time.time;
        Fire(crashSource, ToScreenSpace(-normal) * (collisionMaxVelocity * severity));
    }

    // -------------------------------------------------------------------------
    // 🚫 Denied jump
    // -------------------------------------------------------------------------
    /// <summary>
    /// Called by Propulsion.OnJumpDenied. The bool separates the two denials, which
    /// are different failures: grounded means the charge was spent for nothing, air
    /// means the token survives and you may retry.
    ///
    /// Thrown down the CHASSIS axis rather than world-down, so a denial while banked
    /// still reads as the craft failing to rise rather than as a sideways nudge.
    /// </summary>
    private void HandleJumpDenied(bool grounded)
    {
        float velocity = grounded ? deniedJumpGroundedVelocity : deniedJumpAirVelocity;
        Fire(jumpDeniedSource, ToScreenSpace(-VehicleUp) * velocity);
    }

    // -------------------------------------------------------------------------
    // ⚡ EMP
    // -------------------------------------------------------------------------
    /// <summary>
    /// Called by EMP.TryActivate on success only, after the energy debit and the
    /// projectile spawn, so this never fires on a refused shot. Thrown backward
    /// along the chassis: the projectile leaves forward, the camera recoils.
    /// </summary>
    private void HandleEmpFired()
    {
        Fire(empSource, ToScreenSpace(-VehicleForward) * empVelocity);
    }

    // -------------------------------------------------------------------------
    // 🔫 Weapon recoil
    // -------------------------------------------------------------------------
    /// <summary>
    /// Called by Weapons.OnWeaponFired. Both the projectile path and the
    /// ParticleSystem path raise it, so opting a weapon in works for either kind.
    ///
    /// **Strength comes from the weapon, not from here.** A rocket and a chain gun
    /// want completely different kicks, and the thing that knows which is which is
    /// the WeaponDefinition. Authoring it here would mean a slot-index lookup that
    /// silently means something different the moment the loadout is reordered.
    ///
    /// The event always carries `ActiveSlotIndex`, so `ActiveSlot` is by definition
    /// the slot that just fired and no index lookup is needed at all.
    /// </summary>
    private void HandleWeaponFired(int slotIndex, int ammoRemaining)
    {
        if (!weaponRecoilEnabled || weapons == null) return;

        HoverController_Weapons.WeaponSlot slot = weapons.ActiveSlot;
        if (slot == null || slot.definition == null) return;

        float velocity = slot.definition.combat.recoilVelocity * weaponRecoilScale;
        if (velocity <= 0f) return;   // the opt-in: 0 means this weapon has no recoil

        Fire(weaponRecoilSource, ToScreenSpace(-VehicleForward) * velocity);
    }

    // -------------------------------------------------------------------------
    // Firing
    // -------------------------------------------------------------------------
    /// <summary>Chassis axes. Never `transform`, which here is the camera rig.</summary>
    private Vector3 VehicleForward => vehicleRoot != null ? vehicleRoot.forward : transform.forward;
    private Vector3 VehicleUp      => vehicleRoot != null ? vehicleRoot.up      : transform.up;

    /// <summary>
    /// Single choke point. Null-tolerant so one unwired channel cannot take the
    /// others down with it; Awake has already logged which one it is.
    /// </summary>
    private static void Fire(CinemachineImpulseSource source, Vector3 velocity)
    {
        if (source == null) return;
        source.GenerateImpulseWithVelocity(velocity);
    }

    /// <summary>
    /// Converts a WORLD direction into the space the listeners actually read.
    ///
    /// Both vcam listeners have Use Camera Space on, and that setting is not a
    /// preference, it changes the meaning of the vector: `CinemachineImpulseListener`
    /// does `impulsePos = state.RawOrientation * impulsePos`, so whatever is handed
    /// over is treated as a SCREEN direction and then rotated out into the world.
    ///
    /// Hand it a world vector raw and the craft's compass heading silently becomes
    /// a screen direction. A crash while facing world -X arrived as a shove to
    /// screen LEFT, which is exactly how it was reported: the craft slid to the
    /// side of the frame while sitting still. Facing world +Z it would have looked
    /// correct, which is what made it survive the first pass.
    ///
    /// The landing punch does NOT go through here, deliberately. See HandleHardLanding.
    /// </summary>
    private static Vector3 ToScreenSpace(Vector3 worldDirection)
    {
        Camera cam = Camera.main;
        return cam != null ? cam.transform.InverseTransformDirection(worldDirection)
                           : worldDirection;
    }

    // -------------------------------------------------------------------------
    // 🛠 Play-mode test hotkeys
    // -------------------------------------------------------------------------
    /// <summary>
    /// Press with the Game view focused, in play mode. Each runs the exact handler
    /// the real event would.
    ///
    ///   F9   hard landing at testSeverity
    ///   F10  crash at testSeverity, thrown forward as though into a wall ahead
    ///   F11  denied grounded jump      (hold Shift for the air variant)
    ///   F12  EMP launch                (hold Shift for weapon recoil)
    ///
    /// Hotkeys rather than the context-menu items below: menu navigation stalls the
    /// player loop, and the post-menu frame's deltaTime can step past the entire
    /// impulse envelope, so the shake resolves without ever rendering.
    /// </summary>
#if UNITY_EDITOR
    private void Update()
    {
        var kb = UnityEngine.InputSystem.Keyboard.current;
        if (kb == null) return;

        bool shift = kb.leftShiftKey.isPressed || kb.rightShiftKey.isPressed;

        if (kb.f9Key.wasPressedThisFrame)
        {
            Debug.Log($"[ImpulseRouter] F9 hard landing at severity {testSeverity:F2}.", this);
            HandleHardLanding(testSeverity);
        }
        else if (kb.f10Key.wasPressedThisFrame)
        {
            Debug.Log($"[ImpulseRouter] F10 crash at severity {testSeverity:F2}.", this);
            _lastCollisionShakeTime = Time.time;
            FireTestCrash();
        }
        else if (kb.f11Key.wasPressedThisFrame)
        {
            Debug.Log($"[ImpulseRouter] F11 denied {(shift ? "air" : "grounded")} jump.", this);
            HandleJumpDenied(!shift);
        }
        else if (kb.f12Key.wasPressedThisFrame)
        {
            if (shift)
            {
                var slot = weapons != null ? weapons.ActiveSlot : null;
                string weapon = slot?.definition != null
                    ? $"{slot.definition.name} recoil {slot.definition.combat.recoilVelocity:F2}"
                    : "no active weapon";
                Debug.Log($"[ImpulseRouter] F12+Shift weapon recoil: {weapon}, " +
                          $"master={weaponRecoilEnabled}, scale={weaponRecoilScale:F2}.", this);
                HandleWeaponFired(weapons != null ? weapons.ActiveSlotIndex : 0, 0);
            }
            else
            {
                Debug.Log("[ImpulseRouter] F12 EMP launch.", this);
                HandleEmpFired();
            }
        }
    }
#endif

    /// <summary>
    /// Synthesised head-on crash. A wall ahead faces back at us, so its negated
    /// normal runs forward down the chassis, which is the same geometry
    /// ReportCollision would produce off a real impact.
    /// </summary>
    private void FireTestCrash() =>
        Fire(crashSource, ToScreenSpace(VehicleForward) * (collisionMaxVelocity * testSeverity));

    [ContextMenu("Test Hard Landing Impulse")]
    private void TestHardLanding()
    {
        if (!RequirePlayMode()) return;
        HandleHardLanding(testSeverity);
    }

    [ContextMenu("Test Crash Impulse")]
    private void TestCrash()
    {
        if (!RequirePlayMode()) return;
        FireTestCrash();
    }

    [ContextMenu("Test Denied Jump Impulse")]
    private void TestDeniedJump()
    {
        if (!RequirePlayMode()) return;
        HandleJumpDenied(true);
    }

    [ContextMenu("Test EMP Impulse")]
    private void TestEmp()
    {
        if (!RequirePlayMode()) return;
        HandleEmpFired();
    }

    private bool RequirePlayMode()
    {
        if (Application.isPlaying) return true;
        Debug.LogWarning("[ImpulseRouter] Enter play mode to test impulses.", this);
        return false;
    }
}
