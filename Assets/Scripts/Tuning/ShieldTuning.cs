using System;
using UnityEngine;

/// <summary>
/// Tuning fields consumed by HoverController_Shield. Pure designer-facing
/// values. No scene refs, no runtime state.
/// </summary>
[Serializable]
public class ShieldTuning
{
    // -------------------------------------------------------------------------
    // 🛡 Shield
    // -------------------------------------------------------------------------
    [Header("🛡 Shield")]
    [Tooltip("Master toggle. When false, the shield component is inert and TryActivate always fails.")]
    public bool enableShield = true;

    [Tooltip("How long the shield stays up once raised.\n\n" +
             "You cannot drop it early, so this is also how long you are COMMITTED. Longer is safer " +
             "against sustained fire but easier for an opponent to simply wait out. An EMP hit " +
             "cancels it immediately, which is the intended counter.")]
    [Min(0.05f)]
    public float shieldDuration = 2f;

    [Tooltip("Energy paid once on activation.\n\n" +
             "Read it against Max Energy and against the EMP cost: shield is the only thing that " +
             "grants full immunity, so it should cost enough that raising it means giving up your " +
             "next boost or dodge.")]
    [Min(0f)]
    public float shieldEnergyCost = 35f;
}
