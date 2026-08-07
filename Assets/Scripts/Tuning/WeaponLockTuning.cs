using System;
using UnityEngine;

/// <summary>
/// Target acquisition. Missile type only.
/// Pure designer-facing values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class WeaponLockTuning
{
    // -------------------------------------------------------------------------
    // 🔒 Lock
    // -------------------------------------------------------------------------
    [Header("🔒 Lock")]
    [Tooltip("Dumbfire: launches the instant you press. Flies straight, tracks nothing.\n" +
             "SoftHoming: also launches instantly, then grabs whoever is in the cone and chases " +
             "them. No waiting around, which is what makes it the harassment tool.\n" +
             "HardLock: hold fire to build a lock, let go to launch. Slow to set up, but it will " +
             "not miss.")]
    public MissileFireMode missileFireMode = MissileFireMode.HardLock;

    [Tooltip("How long you have to keep the target inside the cone before the lock confirms. " +
             "HardLock only.")]
    [Min(0.01f)]
    public float lockAcquireTime = 1.5f;

    [Tooltip("How far out the weapon can see targets, in metres. Used by SoftHoming and HardLock.")]
    [Min(1f)]
    public float lockRange = 80f;

    [Tooltip("How wide the targeting cone is, measured from the centre outward, in degrees. Small " +
             "means you have to be pointed almost straight at them; large is forgiving but will " +
             "happily grab the wrong target in a crowd. Used by SoftHoming and HardLock.")]
    [Range(1f, 45f)]
    public float lockConeAngle = 15f;
}
