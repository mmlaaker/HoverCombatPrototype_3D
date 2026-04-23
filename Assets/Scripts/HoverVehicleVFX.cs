using UnityEngine;

/// <summary>
/// Drives all cosmetic particle VFX on the hover vehicle.
///
/// All particle systems run continuously from Start. Boost modulates
/// emission rate by lerping between an idle rate and a boost rate using
/// HoverController_Propulsion.BoostLerp. No inspector wiring to other
/// HoverController scripts is needed -- acquired via GetComponent in Awake.
///
/// Slot layout (inspector-assigned):
///   hoverPoints  -- one VFXSlot per thruster corner (expect 4)
///   rearExhaust  -- rear exhaust mounts (expect 2)
///   leftSide     -- left side boost mounts (expect 2)
///   rightSide    -- right side boost mounts (expect 2)
///
/// Each VFXSlot holds a ParticleSystem array, allowing multiple layers
/// (e.g. base glow + spark) per physical mount without hardcoding counts.
/// </summary>
public class HoverVehicleVFX : MonoBehaviour
{
    [System.Serializable]
    public class VFXSlot
    {
        public ParticleSystem[] particles;
    }

    [Header("Slots")]
    [SerializeField] private VFXSlot[] hoverPoints;
    [SerializeField] private VFXSlot[] rearExhaust;
    [SerializeField] private VFXSlot[] leftSide;
    [SerializeField] private VFXSlot[] rightSide;

    [Header("Hover Point Emission")]
    [SerializeField] private float hoverIdleRate  = 10f;
    [SerializeField] private float hoverBoostRate = 30f;
    [SerializeField] private float hoverIdleSize  = 1f;
    [SerializeField] private float hoverBoostSize = 1f;

    [Header("Rear Exhaust Emission")]
    [SerializeField] private float exhaustIdleRate  = 15f;
    [SerializeField] private float exhaustBoostRate = 60f;
    [SerializeField] private float exhaustIdleSize  = 1f;
    [SerializeField] private float exhaustBoostSize = 1f;

    [Header("Side Boost Emission")]
    [SerializeField] private float sideIdleRate  = 5f;
    [SerializeField] private float sideBoostRate = 40f;
    [SerializeField] private float sideIdleSize  = 1f;
    [SerializeField] private float sideBoostSize = 1f;

    private HoverController_Propulsion propulsion;

    private void Awake()
    {
        propulsion = GetComponent<HoverController_Propulsion>();
    }

    private void Start()
    {
        PlayAll(hoverPoints);
        PlayAll(rearExhaust);
        PlayAll(leftSide);
        PlayAll(rightSide);
    }

    private void Update()
    {
        float boost = propulsion.BoostLerp;

        SetSlot(hoverPoints, Mathf.Lerp(hoverIdleRate,   hoverBoostRate,   boost), Mathf.Lerp(hoverIdleSize,   hoverBoostSize,   boost));
        SetSlot(rearExhaust, Mathf.Lerp(exhaustIdleRate, exhaustBoostRate, boost), Mathf.Lerp(exhaustIdleSize, exhaustBoostSize, boost));
        SetSlot(leftSide,    Mathf.Lerp(sideIdleRate,    sideBoostRate,    boost), Mathf.Lerp(sideIdleSize,    sideBoostSize,    boost));
        SetSlot(rightSide,   Mathf.Lerp(sideIdleRate,    sideBoostRate,    boost), Mathf.Lerp(sideIdleSize,    sideBoostSize,    boost));
    }

    private static void PlayAll(VFXSlot[] slots)
    {
        if (slots == null) return;
        foreach (var slot in slots)
        {
            if (slot?.particles == null) continue;
            foreach (var ps in slot.particles)
                if (ps != null) ps.Play();
        }
    }

    private static void SetSlot(VFXSlot[] slots, float rate, float size)
    {
        if (slots == null) return;
        foreach (var slot in slots)
        {
            if (slot?.particles == null) continue;
            foreach (var ps in slot.particles)
            {
                if (ps == null) continue;
                var em = ps.emission;
                em.rateOverTime = rate;
                var main = ps.main;
                main.startSize = size;
            }
        }
    }
}
