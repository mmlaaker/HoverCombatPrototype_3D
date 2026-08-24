#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

/// <summary>
/// RocketForceProbe — TEMPORARY measurement rig for TODO 5.16.
/// -----------------------------------------------------------
/// Re-measures what a Dumbfire rocket actually does to a craft now that topSpeed is 105.
/// The last measurement (~100 m/s, "1.67x top speed") was taken against topSpeed 60.
///
/// Fires through the REAL path: the private FireAllMuzzles, so the projectile is
/// instantiated from the definition's own prefab, flies, arms, sweeps and explodes
/// exactly as it does in play. Nothing here applies an impulse by hand — the point is
/// to find out whether the code does what WeaponImpact's comments claim, which no
/// amount of dividing impactForce by mass can tell you (Measuring.md trap 4).
///
/// The projectile is TRACKED to detonation. A first run reported all zeros, which is
/// worthless unless you can tell a weak weapon from a clean miss (trap 3). The report
/// now states where the rocket died and how far that was from the target.
///
/// Delete this file and the MeasureRig GameObject when 5.16 is authored.
/// </summary>
public class RocketForceProbe : MonoBehaviour
{
    [Header("Arming")]
    [Tooltip("OFF by default and it must stay that way. This probe freezes the player craft and " +
             "fires a rocket three seconds into play, which from the driver's seat looks exactly " +
             "like the car locking up and shooting on its own. Tick it only for the run you want, " +
             "and untick it afterwards.")]
    public bool armProbe = false;

    [Header("Aim")]
    [Tooltip("Metres above the target's centre of mass to aim at. 0 = centred hit. " +
             "Positive, with the shooter placed off to one side, gives the high flank hit.")]
    public float aimHeightOffset = 0f;

    [Tooltip("Non-zero fires a near miss into the ground at this offset from the target instead " +
             "of hitting it. Splash is the ONLY thing a craft can take here, because a direct " +
             "victim is excluded from the splash pass.")]
    public Vector3 nearMissOffset = Vector3.zero;

    [Tooltip("Non-zero runs a ROCKET JUMP test instead: the firer shoots the ground at this many " +
             "degrees below its own nose and we measure what the blast does to the FIRER. " +
             "A diagonal blast is the worst case for self-spin -- an axis-aligned one puts the " +
             "contact point on the push axis through the centre of mass and produces no rotation " +
             "at all. The craft is left dynamic for this, obviously.")]
    public float rocketJumpAngleDeg = 0f;

    [Header("Timing")]
    [Tooltip("Seconds to let both craft settle onto their hover before firing. Trap 1.")]
    public float settleSeconds = 3f;
    [Tooltip("Seconds to sample after the shot is fired.")]
    public float sampleSeconds = 5f;

    private Rigidbody _aiBody, _playerBody;
    private HoverController_Foundation _aiFoundation;
    private HoverController_Weapons _playerWeapons;
    private readonly List<Rigidbody> _cubes = new List<Rigidbody>();
    private readonly List<string> _cubeNames = new List<string>();

    private IEnumerator Start()
    {
        // Hijacking play mode is this thing's whole job, so it does not get to decide for
        // itself when to do it. Left armed once already and it read as the car locking up
        // and firing a missile on its own.
        if (!armProbe) yield break;

        GameObject player = null, ai = null, rig = null;
        foreach (var r in UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects())
        {
            if (r.name == "HoverCar_Prototype") player = r;
            if (r.name == "HoverCar_AI")        ai     = r;
            if (r.name == "MeasureRig")         rig    = r;
        }
        if (player == null || ai == null) { Debug.LogError("[Probe] ABORT: craft not found."); yield break; }

        _playerBody    = player.GetComponent<Rigidbody>();
        _playerWeapons = player.GetComponent<HoverController_Weapons>();
        _aiBody        = ai.GetComponent<Rigidbody>();
        _aiFoundation  = ai.GetComponent<HoverController_Foundation>();

        if (rig != null)
            foreach (Transform t in rig.transform)
            {
                var rb = t.GetComponent<Rigidbody>();
                if (rb != null) { _cubes.Add(rb); _cubeNames.Add(t.name); }
            }

        // ---- Settle -----------------------------------------------------------
        float t0 = Time.time;
        while (Time.time - t0 < settleSeconds) yield return new WaitForFixedUpdate();

        Vector3 aiRest     = _aiBody.position;
        Vector3 aiCentre   = _aiBody.worldCenterOfMass;
        float   aiRestTilt = Vector3.Angle(_aiBody.transform.up, Vector3.up);

        Debug.Log($"[Probe] SETTLED  ai.speed={_aiBody.linearVelocity.magnitude:F3}  " +
                  $"player.speed={_playerBody.linearVelocity.magnitude:F3}  " +
                  $"ai.tilt={aiRestTilt:F2}deg  ai.downed={_aiFoundation?.IsDowned}");

        // ---- Ensure a Dumbfire slot exists ------------------------------------
        // The SCENE INSTANCE overrides weaponSlots.Array.size to 1, so the playable craft
        // carries only the Machine Gun though both prefabs author six. Inject at runtime
        // rather than editing the authored scene; play mode discards this on exit.
        var slotsField = typeof(HoverController_Weapons)
            .GetField("weaponSlots", BindingFlags.NonPublic | BindingFlags.Instance);
        var slots = slotsField.GetValue(_playerWeapons) as System.Collections.IList;

        int fireIndex = -1;
        for (int i = 0; i < slots.Count; i++)
        {
            var d = slots[i].GetType().GetField("definition").GetValue(slots[i]) as WeaponDefinition;
            if (d != null && d.displayName == "Dumbfire Missile") { fireIndex = i; break; }
        }

        Transform muzzle = player.transform.Find("Weapons/Missile");
        if (muzzle == null) { Debug.LogError("[Probe] ABORT: no Weapons/Missile muzzle."); yield break; }

        if (fireIndex < 0)
        {
            var def = UnityEditor.AssetDatabase.LoadAssetAtPath<WeaponDefinition>("Assets/Data/WD_Missile.asset");
            if (def == null) { Debug.LogError("[Probe] ABORT: WD_Missile not found."); yield break; }

            var slotType = typeof(HoverController_Weapons).GetNestedType("WeaponSlot");
            var newSlot  = System.Activator.CreateInstance(slotType);
            slotType.GetField("definition").SetValue(newSlot, def);
            slotType.GetField("muzzlePoints").SetValue(newSlot, new List<Transform> { muzzle });
            slotType.GetField("particleEmitters").SetValue(newSlot, new List<ParticleSystem>());
            slotType.GetMethod("Initialize").Invoke(newSlot, null);
            slots.Add(newSlot);
            fireIndex = slots.Count - 1;
            Debug.LogWarning($"[Probe] Scene had no Dumbfire slot. Injected at index {fireIndex} for this session only.");
        }

        // ---- Hold the shooter still and aim it --------------------------------
        // The first run drifted at 1.07 m/s and the muzzle sat 4 deg off, which at 60m
        // is a ~4m lateral error — wider than the craft. Freeze, then point the muzzle
        // at the aim point directly. FireAllMuzzles spawns at muzzle.position/rotation,
        // so this guarantees the geometry under test rather than hoping for it.
        bool jumpTest = rocketJumpAngleDeg > 0.01f;

        // The rocket jump test measures what the blast does to the FIRER, so the firer
        // obviously cannot be kinematic. Every other mode freezes it for aim stability.
        if (!jumpTest)
        {
            _playerBody.linearVelocity  = Vector3.zero;
            _playerBody.angularVelocity = Vector3.zero;
            _playerBody.isKinematic     = true;
        }

        // Direct hit, or a deliberate near miss into the ground beside the target.
        // The near miss is the only way a craft takes splash at all: a direct victim is
        // excluded from splash, so it never sees the biased direction at all.
        Vector3 aimPoint;
        if (jumpTest)
        {
            // Angle the nose down and fire. Where it lands is wherever that ray meets the
            // ground, which is the honest version of what a player does.
            Vector3 dir = Quaternion.AngleAxis(rocketJumpAngleDeg, player.transform.right)
                          * player.transform.forward;
            aimPoint = Physics.Raycast(muzzle.position, dir, out RaycastHit gnd, 300f)
                ? gnd.point
                : muzzle.position + dir * 50f;

            float toGround = Vector3.Distance(muzzle.position, aimPoint);
            Debug.Log($"[Probe] ROCKET JUMP  angle={rocketJumpAngleDeg:F0}deg below nose  " +
                      $"ground hit {toGround:F1}m away  muzzleHeight={muzzle.position.y:F2}");
        }
        else if (nearMissOffset.sqrMagnitude > 0.01f)
        {
            Vector3 probeFrom = aiRest + nearMissOffset + Vector3.up * 50f;
            aimPoint = Physics.Raycast(probeFrom, Vector3.down, out RaycastHit gh, 200f)
                ? gh.point
                : aiRest + nearMissOffset;
            Debug.Log($"[Probe] NEAR MISS aiming at ground {aimPoint.ToString("F2")}, " +
                      $"{Vector3.Distance(aimPoint, aiCentre):F1}m from target centre");
        }
        else
        {
            aimPoint = aiCentre + Vector3.up * aimHeightOffset;
        }
        muzzle.rotation = Quaternion.LookRotation((aimPoint - muzzle.position).normalized, Vector3.up);

        float range = Vector3.Distance(muzzle.position, aimPoint);
        Debug.Log($"[Probe] AIM  muzzle={muzzle.position.ToString("F2")}  aimPoint={aimPoint.ToString("F2")}  " +
                  $"range={range:F1}m  heightOffset={aimHeightOffset:F1}m");

        object slot = slots[fireIndex];
        var slotDef = slot.GetType().GetField("definition").GetValue(slot) as WeaponDefinition;
        Debug.Log($"[Probe] FIRING '{slotDef.displayName}'  impactForce={slotDef.impact.impactForce}  " +
                  $"splash={slotDef.impact.splashImpactForce}  radius={slotDef.blast.splashRadius}  " +
                  $"destabilize={slotDef.impact.destabilizeFraction}  speed={slotDef.flight.speed}");

        var before = Object.FindObjectsByType<RocketProjectile>(FindObjectsSortMode.None);
        var fire = typeof(HoverController_Weapons)
            .GetMethod("FireAllMuzzles", BindingFlags.NonPublic | BindingFlags.Instance);
        fire.Invoke(_playerWeapons, new object[] { slot, true });

        // Grab the rocket we just made and turn its splash logging on.
        RocketProjectile rocket = null;
        foreach (var r in Object.FindObjectsByType<RocketProjectile>(FindObjectsSortMode.None))
        {
            bool isNew = true;
            foreach (var b in before) if (b == r) { isNew = false; break; }
            if (isNew) { rocket = r; break; }
        }
        if (rocket == null) Debug.LogError("[Probe] No rocket was spawned. Everything below is meaningless.");
        else typeof(RocketProjectile).GetField("logSplash", BindingFlags.NonPublic | BindingFlags.Instance)
                                     ?.SetValue(rocket, true);

        // ---- Sample -----------------------------------------------------------
        float peakAiSpeed = 0f, peakAiTilt = aiRestTilt, peakAiSpin = 0f, peakPlayerSpeed = 0f;
        float peakAiVertical = 0f, peakAiHeightGain = 0f;

        // Firer-side, for the rocket jump test.
        var   playerFoundation = player.GetComponent<HoverController_Foundation>();
        float playerRestTilt   = Vector3.Angle(player.transform.up, Vector3.up);
        float peakSelfSpin = 0f, peakSelfTilt = playerRestTilt, peakSelfLift = 0f;
        Vector3 playerStartVel = _playerBody.isKinematic ? Vector3.zero : _playerBody.linearVelocity;
        Vector3 playerStartPos = _playerBody.position;
        bool    selfDowned = false;
        float timeToPeak = 0f;
        bool  downedSeen = false;
        var   peakCube = new float[_cubes.Count];

        float   tFire = Time.time;
        float   deathTime = -1f;
        Vector3 deathPos = Vector3.zero, lastRocketPos = muzzle.position;
        Vector3 firerComAtBlast = _playerBody.worldCenterOfMass;

        while (Time.time - tFire < sampleSeconds)
        {
            yield return new WaitForFixedUpdate();

            if (rocket != null) lastRocketPos = rocket.transform.position;
            else if (deathTime < 0f)
            {
                deathTime = Time.time - tFire;
                deathPos  = lastRocketPos;
                // Capture the firer's position NOW. Reading it in the report instead measured
                // from the blast to wherever the craft had been thrown by the time the sample
                // window closed, which read as "out of range" on a shot that plainly connected.
                firerComAtBlast = _playerBody.worldCenterOfMass;
            }

            float s = _aiBody.linearVelocity.magnitude;
            if (s > peakAiSpeed) { peakAiSpeed = s; timeToPeak = Time.time - tFire; }

            float up = _aiBody.linearVelocity.y;
            if (up > peakAiVertical) peakAiVertical = up;

            float gain = _aiBody.position.y - aiRest.y;
            if (gain > peakAiHeightGain) peakAiHeightGain = gain;

            float tilt = Vector3.Angle(_aiBody.transform.up, Vector3.up);
            if (tilt > peakAiTilt) peakAiTilt = tilt;

            float spin = _aiBody.angularVelocity.magnitude;
            if (spin > peakAiSpin) peakAiSpin = spin;

            if (_aiFoundation != null && _aiFoundation.IsDowned) downedSeen = true;

            if (!_playerBody.isKinematic)
            {
                float ss = _playerBody.angularVelocity.magnitude;
                if (ss > peakSelfSpin) peakSelfSpin = ss;

                float st = Vector3.Angle(player.transform.up, Vector3.up);
                if (st > peakSelfTilt) peakSelfTilt = st;

                float sl = (_playerBody.linearVelocity - playerStartVel).magnitude;
                if (sl > peakSelfLift) peakSelfLift = sl;

                if (playerFoundation != null && playerFoundation.IsDowned) selfDowned = true;
            }

            for (int i = 0; i < _cubes.Count; i++)
            {
                if (_cubes[i] == null) continue;
                float cs = _cubes[i].linearVelocity.magnitude;
                if (cs > peakCube[i]) peakCube[i] = cs;
            }
        }

        // The firer is held kinematic for aim stability, so measure the self-shove as the
        // force the blast WOULD have applied: is the firer even inside the blast's layer mask?
        int  playerLayer = player.layer;
        bool firerInMask = (slotDef.blast.damageLayers.value & (1 << playerLayer)) != 0;

        float displaced   = Vector3.Distance(_aiBody.position, aiRest);
        float missDistance = deathTime >= 0f ? Vector3.Distance(deathPos, aiCentre) : -1f;
        const float topSpeed = 105f, boosted = 131.25f;

        var sb = new System.Text.StringBuilder();
        sb.AppendLine("========== [Probe] ROCKET FORCE RESULT ==========");
        sb.AppendLine($"shot               : {(nearMissOffset.sqrMagnitude > 0.01f ? "NEAR MISS " + nearMissOffset.ToString("F1") : "DIRECT, aim height " + aimHeightOffset.ToString("F1") + "m")}   range {range:F1} m");
        sb.AppendLine($"propUpwardBias     : {slotDef.blast.propUpwardBias:F1} m  (props only; craft exempt)   splashRadius {slotDef.blast.splashRadius:F1} m");
        sb.AppendLine(deathTime >= 0f
            ? $"rocket detonated   : t+{deathTime:F3}s at {deathPos.ToString("F1")}, {missDistance:F1} m from target centre"
            : $"rocket STILL ALIVE at end of window — it missed and is coasting to its {slotDef.flight.lifetime}s fuse");
        sb.AppendLine($"AI peak speed      : {peakAiSpeed:F1} m/s   ({peakAiSpeed / topSpeed:F2}x topSpeed 105, {peakAiSpeed / boosted:F2}x boosted 131)");
        sb.AppendLine($"  time to peak     : {timeToPeak:F3}s");
        sb.AppendLine($"AI peak LIFT       : {peakAiVertical:F1} m/s up, gained {peakAiHeightGain:F1} m of height");
        sb.AppendLine($"AI peak tilt       : {peakAiTilt:F1} deg   (80 deg = downed ~1.6s)");
        sb.AppendLine($"AI peak spin       : {peakAiSpin:F2} rad/s");
        sb.AppendLine($"AI went DOWNED     : {downedSeen}");
        sb.AppendLine($"AI displaced       : {displaced:F1} m over {sampleSeconds}s");
        sb.AppendLine($"firer layer        : {LayerMask.LayerToName(playerLayer)} ({playerLayer}), inside blast mask: {firerInMask}");
        sb.AppendLine($"  -> rocket-jump self-shove is {(firerInMask ? "REACHABLE" : "UNREACHABLE: OverlapSphere can never return the firer")}");
        for (int i = 0; i < _cubes.Count; i++)
            sb.AppendLine($"  {_cubeNames[i],-10} peak {peakCube[i]:F1} m/s");
        if (jumpTest)
        {
            float armingDistance = slotDef.flight.speed * slotDef.flight.armingDelay;
            float blastToFirer   = deathTime >= 0f
                ? Vector3.Distance(deathPos, firerComAtBlast) : -1f;

            sb.AppendLine("---------- ROCKET JUMP (the firer) ----------");
            sb.AppendLine($"arming distance    : {armingDistance:F1} m  (speed {slotDef.flight.speed} x armingDelay {slotDef.flight.armingDelay})");
            sb.AppendLine($"muzzle height      : {muzzle.position.y - 6.54f:F1} m above ground");
            sb.AppendLine($"blast to firer     : {blastToFirer:F1} m   (splashRadius {slotDef.blast.splashRadius:F1} m)");
            sb.AppendLine($"  -> {(blastToFirer < 0f ? "ROCKET NEVER DETONATED" : blastToFirer > slotDef.blast.splashRadius ? "OUT OF RANGE: firer took nothing" : "in range")}");
            sb.AppendLine($"self shove         : {peakSelfLift:F1} m/s   (air jump 25, charged jump max 40)");
            sb.AppendLine($"self SPIN          : {peakSelfSpin:F2} rad/s   (~10-14 = commits to a flip)");
            sb.AppendLine($"self peak tilt     : {peakSelfTilt:F1} deg   (80 = downed ~1.6s)");
            sb.AppendLine($"firer went DOWNED  : {selfDowned}");
            sb.AppendLine($"destabilizeFraction: {slotDef.impact.destabilizeFraction:F2}   selfImpactScale {slotDef.impact.selfImpactScale:F2}");
        }
        sb.AppendLine("================================================");
        Debug.Log(sb.ToString());

        _playerBody.isKinematic = false;
    }
}
#endif
