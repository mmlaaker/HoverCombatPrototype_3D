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

    [Tooltip("How many locks one hold can stack up, and therefore how many missiles the volley " +
             "fires. HardLock only. 1 is the single-missile weapon.\n\n" +
             "Each lock costs another Lock Acquire Time, so this and that value together decide " +
             "how long a full volley takes to set up: at 4 locks and 0.4s each you are holding for " +
             "1.6 seconds before release. That hold is the whole cost of the weapon, so tune the " +
             "two as a pair rather than separately.\n\n" +
             "One missile and one ammo is spent per lock, so a volley that outruns your ammo is " +
             "truncated to what you can actually pay for rather than firing on credit.")]
    [Min(1)]
    public int maxLocks = 1;

    [Tooltip("When there is nobody new left to lock, keep stacking locks onto targets you have " +
             "already acquired.\n\n" +
             "ON is the RYNO: point at one victim and the whole volley converges on them. This is " +
             "what makes the weapon do anything at all in a one-on-one fight, where there IS no " +
             "second target.\n" +
             "OFF caps the volley at the number of distinct targets in the cone, so it is a " +
             "crowd-clearing weapon that goes quiet against a single opponent.")]
    public bool allowRepeatLocks = true;

    [Tooltip("How many locks you must bank before releasing fires anything. Below this the volley " +
             "is DROPPED and you get nothing.\n\n" +
             "This is what stops the weapon collapsing into cheap homing spam. Without it, holding " +
             "for one lock and letting go hands you a guaranteed-tracking missile in a fraction of " +
             "a second — strictly better than the Soft Homing Missile, which is supposed to own " +
             "that job, and available for the same trigger pull.\n\n" +
             "Set it equal to Max Locks for full commitment: you either pay the whole hold or you " +
             "waste the attempt. Set it lower to allow a partial salvo when a target escapes " +
             "part-way through.\n\n" +
             "It is clamped to Max Locks at runtime, so a value above that means 'always require " +
             "a full volley' rather than becoming impossible to fire.")]
    [Min(1)]
    public int minLocksToFire = 1;

    [Tooltip("Seconds between each missile leaving the tube. 0 fires the whole volley on one " +
             "frame, which is the toggle for this being off.\n\n" +
             "One value rather than a separate on/off switch, so the two can never disagree — " +
             "a checkbox that is ticked while the interval sits at 0 does nothing and looks broken.\n\n" +
             "Where to start: 0.06 to 0.12. Below about 0.05 the eye reads it as simultaneous " +
             "anyway; above about 0.2 a five-missile volley takes a full second to leave and the " +
             "last ones launch at a target that has already been hit by the first.\n\n" +
             "Note this delays the LAUNCH, not the flight. Missiles fired later also arrive " +
             "later, which is what turns a single thump into a rolling series of hits.")]
    [Min(0f)]
    public float volleyLaunchInterval = 0f;
}
