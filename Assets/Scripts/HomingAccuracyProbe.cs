#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.Text;
using UnityEngine;

/// <summary>
/// HomingAccuracyProbe — measurement rig for the Soft Homing missile.
/// -------------------------------------------------------------------
/// The design spec for this weapon is a HIT RATE: "about 85% if the target is in view".
/// A percentage is not something you can eyeball, and it is not something you can derive
/// from turnRate on paper either, because the guidance law is pure pursuit and pure
/// pursuit's miss distance depends on the whole engagement geometry rather than on any
/// single authored number. So: fire a lot of missiles at a target moving in a controlled
/// way, and count.
///
/// WHAT IT MEASURES, and why each one is here:
///   damage dealt  - the only unambiguous hit classifier. A direct victim is excluded from
///                   the splash pass (RocketProjectile.Explode), so damage == combat.damage
///                   means a direct hit, 0 &lt; damage &lt; that means splash only, and 0 is a
///                   clean miss. No distance threshold for me to pick and then argue about.
///   min approach  - closest the missile ever got, sampled on the physics tick. Diagnostic:
///                   it separates "guidance was fine and the hit box is small" from "guidance
///                   never had a chance", which the hit/miss bit alone cannot.
///   detonation    - where it actually died, so a missile that expired on its lifetime fuse
///                   out in the weeds cannot be filed as a near miss (Measuring.md trap 3).
///
/// CONTROLLED, not realistic. The target is driven KINEMATICALLY along a scripted path: the
/// same path every trial, and no knockback feeding into the next shot. That is deliberate. A
/// hit rate measured against a target being thrown around by the previous missile measures
/// the knockback, not the guidance.
///
/// The shooter is frozen and aimed at where the target is at the instant of firing, which is
/// what a player does. It is deliberately NOT given a lead solution — that would measure a
/// weapon nobody is holding.
///
/// Delete this file and the MeasureRig GameObject once the weapon is authored.
/// </summary>
public class HomingAccuracyProbe : MonoBehaviour
{
    [Header("Arming")]
    [Tooltip("OFF by default and it must stay that way. This probe seizes both craft and fires " +
             "dozens of missiles unprompted, which from the driver's seat is indistinguishable " +
             "from the game having lost its mind. Tick it for the run you want; untick it after.")]
    public bool armProbe = false;

    [Header("What to fire")]
    [Tooltip("displayName on the WeaponDefinition, e.g. Soft Homing Missile.")]
    public string weaponDisplayName = "Soft Homing Missile";

    [Tooltip("Asset path, used only if the craft carries no slot for that weapon and one has to be injected.")]
    public string weaponAssetPath = "Assets/Data/WD_SoftHomingMissile.asset";

    [Header("Trial matrix")]
    [Tooltip("Firing ranges in metres. Keep the longest at or under the weapon's lockRange — past " +
             "that the player cannot get a lock at all, so its hit rate is not part of the spec.")]
    public float[] ranges = { 40f, 70f, 100f };

    [Tooltip("Target speeds in m/s. 0 is the floor case. topSpeed (105) is the worst case a player " +
             "can actually present. In between is the common case.")]
    public float[] targetSpeeds = { 0f, 52f, 105f };

    [Tooltip("How the target moves. Crossing is the hardest honest case for pure pursuit: all of " +
             "the target's speed is perpendicular to the missile, so none of it is spent closing " +
             "or opening the range.")]
    public MotionMode motion = MotionMode.Crossing;

    [Tooltip("Shots per (range, speed) cell. 8 is enough to tell 50% from 85%. It is NOT enough to " +
             "tell 85% from 90%, so do not read the last digit.")]
    [Min(1)] public int shotsPerCell = 8;

    [Tooltip("Seconds to wait for a missile to resolve before calling the shot a miss.")]
    public float maxFlightSeconds = 3f;

    [Tooltip("Seconds to let both craft settle onto their hover before the first shot. Trap 1.")]
    public float settleSeconds = 3f;

    public enum MotionMode
    {
        /// <summary>Parked. The floor case: if this is not near 100%, something is broken.</summary>
        Static,

        /// <summary>Straight line, perpendicular to the line of fire.</summary>
        Crossing,

        /// <summary>Straight line, directly away from the shooter.</summary>
        Receding,

        /// <summary>Sinusoidal side to side. The "player actively juking" case.</summary>
        Juking
    }

    private Rigidbody _targetBody, _playerBody;
    private VehicleHealth _targetHealth;
    private HoverController_Weapons _playerWeapons;
    private Transform _muzzle;
    private object _slot;

    private struct Shot
    {
        public float range, targetSpeed, damage, minApproach, flightTime, detonationDist;
        public bool  resolved;

        /// <summary>
        /// Peak height above the straight line from muzzle to target. This is the number that
        /// says whether a lofted profile is actually lofting: "it arcs" is a claim about
        /// geometry, and a missile that climbs 2 m over a 60 m shot is not arcing no matter
        /// what the flare is set to.
        /// </summary>
        public float apex;
    }

    private void Start() { StartCoroutine(Run()); }

    private IEnumerator Run()
    {
        // Hijacking play mode is this thing's whole job, so it does not get to decide when.
        if (!armProbe) yield break;

        // ---- Find the two craft ------------------------------------------------
        // GameObject.Find cannot see an inactive object, and the AI craft ships disabled, so
        // walk the scene roots instead.
        //
        // Keyed off the COMPONENT rather than the name. An earlier probe matched on "Player"
        // and the playable craft is called HoverCar_Prototype, so it silently found nothing.
        // Every craft has a Foundation; only the AI is named for it.
        GameObject player = null, ai = null;
        foreach (var r in UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects())
        {
            if (r.GetComponent<HoverController_Foundation>() == null) continue;
            if (r.name.IndexOf("AI", System.StringComparison.OrdinalIgnoreCase) >= 0) ai = r;
            else player = r;
        }

        if (player == null || ai == null)
        {
            Debug.LogError("[Accuracy] ABORT: player=" + (player == null ? "null" : player.name) +
                           " ai=" + (ai == null ? "null" : ai.name));
            yield break;
        }

        if (!ai.activeSelf) ai.SetActive(true);

        _playerBody    = player.GetComponent<Rigidbody>();
        _playerWeapons = player.GetComponent<HoverController_Weapons>();
        _targetBody    = ai.GetComponent<Rigidbody>();
        _targetHealth  = ai.GetComponent<VehicleHealth>();

        if (_playerWeapons == null || _targetBody == null || _targetHealth == null)
        {
            Debug.LogError("[Accuracy] ABORT: missing Weapons / Rigidbody / VehicleHealth.");
            yield break;
        }

        // A TuningTarget on the AI would fight us for its position every tick.
        var tuning = ai.GetComponent<TuningTarget>();
        if (tuning != null && tuning.mode != TuningTarget.TargetMode.Free)
        {
            Debug.LogWarning("[Accuracy] AI has TuningTarget in " + tuning.mode + "; forcing Free for this run.");
            tuning.mode = TuningTarget.TargetMode.Free;
        }

        yield return new WaitForSeconds(settleSeconds);

        if (!ResolveSlot(player)) yield break;

        // Hull size, so the hit rates below can be read against something real rather than
        // against a threshold I picked out of the air. Measured, not assumed.
        Bounds hull = new Bounds(_targetBody.worldCenterOfMass, Vector3.zero);
        foreach (var c in ai.GetComponentsInChildren<Collider>())
            if (!c.isTrigger) hull.Encapsulate(c.bounds);

        float halfWidth = (hull.extents.x + hull.extents.z) * 0.5f;
        Debug.Log("[Accuracy] Target hull extents = " + hull.extents +
                  " (mean half-width " + halfWidth.ToString("F2") + " m).");

        // Both craft go kinematic: the shooter so its aim is exact, the target so the previous
        // missile's knockback cannot contaminate the next trial.
        _playerBody.isKinematic = true;
        _targetBody.isKinematic = true;

        Vector3 origin = _playerBody.position;
        var results = new List<Shot>();

        foreach (float range in ranges)
            foreach (float speed in targetSpeeds)
                for (int i = 0; i < shotsPerCell; i++)
                    yield return RunShot(origin, range, speed, results);

        _playerBody.isKinematic = false;
        _targetBody.isKinematic = false;

        Report(results, halfWidth);
    }

    /// <summary>Finds the weapon slot, injecting one if the scene craft does not carry it.</summary>
    private bool ResolveSlot(GameObject player)
    {
        var slotsField = typeof(HoverController_Weapons)
            .GetField("weaponSlots", BindingFlags.NonPublic | BindingFlags.Instance);
        var slots = slotsField.GetValue(_playerWeapons) as System.Collections.IList;

        for (int i = 0; i < slots.Count; i++)
        {
            var d = slots[i].GetType().GetField("definition").GetValue(slots[i]) as WeaponDefinition;
            if (d != null && d.displayName == weaponDisplayName) { _slot = slots[i]; break; }
        }

        _muzzle = player.transform.Find("Weapons/Missile");
        if (_muzzle == null) { Debug.LogError("[Accuracy] ABORT: no Weapons/Missile muzzle."); return false; }

        if (_slot == null)
        {
            var def = UnityEditor.AssetDatabase.LoadAssetAtPath<WeaponDefinition>(weaponAssetPath);
            if (def == null) { Debug.LogError("[Accuracy] ABORT: not found: " + weaponAssetPath); return false; }

            var slotType = typeof(HoverController_Weapons).GetNestedType("WeaponSlot");
            _slot = System.Activator.CreateInstance(slotType);
            slotType.GetField("definition").SetValue(_slot, def);
            slotType.GetField("muzzlePoints").SetValue(_slot, new List<Transform> { _muzzle });
            slotType.GetField("particleEmitters").SetValue(_slot, new List<ParticleSystem>());
            slotType.GetMethod("Initialize").Invoke(_slot, null);
            slots.Add(_slot);
            Debug.LogWarning("[Accuracy] Scene had no " + weaponDisplayName + " slot. Injected for this session only.");
        }

        return true;
    }

    private IEnumerator RunShot(Vector3 origin, float range, float speed, List<Shot> results)
    {
        // ---- Place the target --------------------------------------------------
        // Straight ahead of the shooter at `range`, at the shooter's own height, so the shot is
        // level and a gravity-free projectile is not quietly doing some of the aiming.
        Vector3 lineOfFire = Vector3.forward;
        Vector3 across     = Vector3.right;
        Vector3 start      = origin + lineOfFire * range;

        _targetBody.position        = start;
        _targetBody.rotation        = Quaternion.LookRotation(across);
        _targetBody.linearVelocity  = Vector3.zero;
        _targetBody.angularVelocity = Vector3.zero;
        _targetHealth.Respawn();                      // full HP, so the classifier below stays clean

        // Aim the muzzle where a player would: at the target, right now. No lead.
        Quaternion aim = Quaternion.LookRotation((start - origin).normalized);
        _playerBody.rotation = aim;
        _muzzle.rotation     = aim;

        yield return new WaitForFixedUpdate();

        float healthBefore = _targetHealth.Health;

        // ---- Fire through the real path ---------------------------------------
        var before = new HashSet<RocketProjectile>(
            Object.FindObjectsByType<RocketProjectile>(FindObjectsSortMode.None));

        var fire = typeof(HoverController_Weapons)
            .GetMethod("FireAllMuzzles", BindingFlags.NonPublic | BindingFlags.Instance);
        fire.Invoke(_playerWeapons, new object[] { _slot, true, _targetBody.transform });

        yield return new WaitForFixedUpdate();

        RocketProjectile missile = null;
        foreach (var p in Object.FindObjectsByType<RocketProjectile>(FindObjectsSortMode.None))
            if (!before.Contains(p)) { missile = p; break; }

        var shot = new Shot { range = range, targetSpeed = speed, minApproach = float.MaxValue };

        if (missile == null)
        {
            Debug.LogError("[Accuracy] No missile spawned — a slot or prefab problem, not a miss.");
            results.Add(shot);
            yield break;
        }

        // ---- Fly it, moving the target underneath it --------------------------
        float t = 0f;
        Vector3 lastPos = missile.transform.position;

        while (missile != null && t < maxFlightSeconds)
        {
            t += Time.fixedDeltaTime;

            Vector3 offset;
            switch (motion)
            {
                case MotionMode.Crossing: offset = across * (speed * t); break;
                case MotionMode.Receding: offset = lineOfFire * (speed * t); break;
                // 1.5 s period: fast enough to read as a juke, slow enough to be drivable.
                case MotionMode.Juking:   offset = across * (speed * 0.24f * Mathf.Sin(t * 4.19f)); break;
                default:                  offset = Vector3.zero; break;
            }

            _targetBody.MovePosition(start + offset);

            lastPos = missile.transform.position;
            float d = Vector3.Distance(lastPos, _targetBody.position);
            if (d < shot.minApproach) shot.minApproach = d;

            // Height above the muzzle-to-target line, not above the ground: the ground is
            // irrelevant and the launch height would otherwise be counted as arc.
            float climb = lastPos.y - Mathf.Lerp(origin.y, start.y,
                Mathf.Clamp01(Vector3.Dot(lastPos - origin, (start - origin).normalized) / Mathf.Max(1f, range)));
            if (climb > shot.apex) shot.apex = climb;

            yield return new WaitForFixedUpdate();
        }

        shot.flightTime     = t;
        shot.detonationDist = Vector3.Distance(lastPos, _targetBody.position);
        shot.damage         = healthBefore - _targetHealth.Health;
        shot.resolved       = missile == null;

        if (missile != null) Destroy(missile.gameObject);   // timed out; do not leave it flying

        results.Add(shot);
        yield return new WaitForSeconds(0.15f);
    }

    private void Report(List<Shot> all, float halfWidth)
    {
        var def = _slot.GetType().GetField("definition").GetValue(_slot) as WeaponDefinition;
        float full = def.combat.damage;

        var sb = new StringBuilder();
        sb.AppendLine("===== HOMING ACCURACY: " + def.displayName + " =====");
        sb.AppendLine("speed=" + def.flight.speed + "  turnRate=" + def.homing.turnRate +
                      " deg/s  turn radius=" +
                      (def.flight.speed / (def.homing.turnRate * Mathf.Deg2Rad)).ToString("F1") + " m");
        sb.AppendLine("flare=" + def.homing.flareOffset + " x range over " + def.homing.flareFlightFraction +
                      "s   homingDelay=" + def.homing.homingDelay + "s   splashRadius=" +
                      def.blast.splashRadius + " m");
        sb.AppendLine("motion=" + motion + "  shots/cell=" + shotsPerCell +
                      "  target hull half-width " + halfWidth.ToString("F2") + " m");
        sb.AppendLine();
        sb.AppendLine("flareDir=" + def.homing.flareDirection + "  flareOffset=" + def.homing.flareOffset +
                      " (climb " + (Mathf.Atan(def.homing.flareOffset) * Mathf.Rad2Deg).ToString("F0") + " deg)");
        sb.AppendLine();
        sb.AppendLine("range  tgtSpd  direct  anyDmg  avgDmg  minAppr(med)   worst    apex");

        foreach (float range in ranges)
            foreach (float speed in targetSpeeds)
            {
                var cell = all.FindAll(s => Mathf.Approximately(s.range, range) &&
                                            Mathf.Approximately(s.targetSpeed, speed));
                if (cell.Count == 0) continue;

                int direct = 0, any = 0;
                float dmgSum = 0f, worst = 0f, apexSum = 0f;
                var approaches = new List<float>();

                foreach (var s in cell)
                {
                    if (s.damage >= full - 0.01f) direct++;
                    if (s.damage > 0.01f) any++;
                    dmgSum += s.damage;
                    apexSum += s.apex;
                    approaches.Add(s.minApproach);
                    if (s.minApproach > worst) worst = s.minApproach;
                }

                approaches.Sort();

                sb.AppendLine(range.ToString("F0").PadLeft(5) + "  " +
                              speed.ToString("F0").PadLeft(6) + "  " +
                              (100f * direct / cell.Count).ToString("F0").PadLeft(5) + "%  " +
                              (100f * any / cell.Count).ToString("F0").PadLeft(5) + "%  " +
                              (dmgSum / cell.Count).ToString("F1").PadLeft(6) + "  " +
                              approaches[approaches.Count / 2].ToString("F1").PadLeft(11) + "  " +
                              worst.ToString("F1").PadLeft(6) + "  " +
                              (apexSum / cell.Count).ToString("F1").PadLeft(6) + " m");
            }

        int totalDirect = all.FindAll(s => s.damage >= full - 0.01f).Count;
        int totalAny    = all.FindAll(s => s.damage > 0.01f).Count;

        sb.AppendLine();
        sb.AppendLine("OVERALL  direct " + (100f * totalDirect / all.Count).ToString("F0") + "%   " +
                      "any damage " + (100f * totalAny / all.Count).ToString("F0") + "%   " +
                      "(n=" + all.Count + ")");

        Debug.Log(sb.ToString());
    }
}
#endif
