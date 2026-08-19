using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

// GamepadButton lives in .LowLevel while Key and Gamepad do not. Aliased rather
// than pulling the whole LowLevel namespace in for one enum. Same reason as
// PlaytestReset, which reads its hotkey the same way.
using GamepadButton = UnityEngine.InputSystem.LowLevel.GamepadButton;

/// <summary>
/// PlaytestSession v1.0
/// ------------------------------------------------
/// Runs one tester through one session: a splash card, a countdown, and a
/// controls overlay that stays LOCKED until enough play has elapsed. The
/// session ENDS ITSELF at zero and re-arms for the next person.
///
/// THE POINT IS THAT THE FACILITATOR NEVER TOUCHES THE LAPTOP
///
///   Four testers, one at a time, one machine. Between them the craft has to go
///   home, the meter has to be full, the gun has to be back to slot 0 and the
///   blind gate has to re-lock. Every one of those is something to forget while
///   also watching a person and taking notes. So the session does all of it on
///   expiry, and the splash coming back up IS the "pad down, let's talk" cue:
///   the seated questions happen against an idle game, and the next tester only
///   has to press Options. Hold P remains, for ending early or redoing a start
///   that got fumbled.
///
///   THE CLOCK COUNTS DOWN rather than up, so remaining time reads at a glance.
///   A visible countdown does apply mild pressure and will make people hurry
///   near zero, which is why the run sheet puts the task list last and free
///   play earlier: tasks are already timed challenges and absorb it, while
///   "show me what feels best" is exactly what must not be rushed.
///
/// WHY THE GATE EXISTS
///
///   The first two minutes a person spends with this build are the only blind
///   first contact they will ever have, and there is one of those per friend.
///   Timing that by hand, four times, late in the evening, produces four
///   different blind periods and therefore four results that cannot be compared.
///   The gate makes the blind period identical by construction and takes the
///   facilitator out of the loop, which is the entire point: they should be
///   watching the player, not a stopwatch.
///
///   The clock starts when the SPLASH IS DISMISSED, not on scene load, so the
///   threshold measures time spent playing rather than time spent reading.
///
/// WHY IT DOES NOT PAUSE
///
///   Toggling the overlay deliberately leaves the game running. Foundation and
///   Propulsion carry cross-boundary state on Time.unscaledTime timestamps, and
///   parking timeScale at zero risks those reading a discontinuity on resume
///   that would present as a physics bug in the middle of a playtest. The
///   overlay is a centred card rather than a full screen wash for the same
///   reason: the player keeps peripheral view and can see they are about to hit
///   something. If they want to stop, they can stop.
///
/// WHY THE UI IS BUILT IN CODE
///
///   Nothing to wire, so nothing can be half wired. This ships the night before
///   a session with four testers and a scene reference that silently came
///   undone would cost the evening. The scene gains one component and no
///   canvas. Both text blocks are [TextArea] fields so the wording can be
///   edited in the inspector without a recompile.
///
/// WHAT IT DOES NOT OWN
///
///   The craft warp belongs to PlaytestReset and is called, not reimplemented.
///   That routine toggles interpolation across the move and calls
///   NotifyVehicleWarped, and both are load bearing. A second copy would be a
///   second place for the camera warp to be forgotten.
///
///   It DOES top up energy on restart, which is the one place it deliberately
///   parts company with PlaytestReset. That component leaves resources alone so
///   that runs after an escape hatch stay readable, which is correct for an
///   escape hatch. A session restart is the opposite case: it hands the craft
///   to a NEW PERSON, and starting their blind two minutes on tester A's empty
///   meter would deny them boost and jump for exactly the window the session is
///   built to measure.
///
/// THE LOG IS THE SESSION INDEX
///
///   Every phase change is stamped with Time.unscaledTime, MotionTrace's clock,
///   so one continuous Player.log across four testers can be cut back into four
///   sessions afterwards. This is why the facilitator restart is a hotkey and
///   not a relaunch of the player: Unity keeps only Player.log and
///   Player-prev.log, so relaunching four times destroys the first two testers.
///
///   The pre-unlock press is logged too, and it is real data rather than noise.
///   A tester reaching for the controls button at 40 seconds is telling you when
///   they gave up guessing.
///
/// Usage: drop on its own scene object. Nothing to assign.
/// Options dismisses the splash and, once unlocked, toggles the overlay.
/// The session ends itself at zero; hold P only to end one early.
///
/// OPTIONS DOES BOTH JOBS ON PURPOSE. Dismissing on "any button" would hand
/// that same press to the game, so starting with Cross would launch a jump on
/// frame one. Options is bound to nothing in HoverControls.inputactions, so it
/// reaches no gameplay system, and using one button for every meta action
/// teaches it once rather than twice.
/// </summary>
[DisallowMultipleComponent]
public class PlaytestSession : MonoBehaviour
{
    private enum Phase { Splash, Running }

    // ── Inspector ────────────────────────────────────────────────────────

    [Header("Timing")]
    [Tooltip("Total length of one tester's session. At zero the session ends BY ITSELF: craft " +
             "home, energy full, splash back up, gate re-locked, ready for the next person. " +
             "The facilitator never has to touch the laptop between testers.")]
    [SerializeField, Range(60f, 3600f)] private float sessionLength = 600f;

    [Tooltip("Seconds of PLAY before the controls overlay unlocks. The clock starts when the " +
             "splash is dismissed, so this is time holding the pad, not time reading. Raise it " +
             "for a stricter blind period; drop it to 0 to disable the gate entirely.")]
    [SerializeField, Range(0f, 600f)] private float controlsUnlockTime = 120f;

    [Tooltip("Small dim elapsed clock in the corner. Kept visible so the facilitator can run to " +
             "schedule without a stopwatch. Most players stop noticing it inside a minute.")]
    [SerializeField] private bool showClock = true;

    [Header("Tester input")]
    [Tooltip("Dismisses the splash, and toggles the controls overlay once unlocked. Options / " +
             "Start. Verified unbound in " +
             "HoverControls.inputactions, which uses only leftStick, leftStickPress, both " +
             "shoulders, both triggers, rightStick, buttonSouth and the d-pad. Read off the " +
             "device directly so a playtest utility never requires editing the shipped asset.")]
    [SerializeField] private GamepadButton overlayButton = GamepadButton.Start;

    [Tooltip("Keyboard equivalent. The input asset binds no keyboard controls at all, so any " +
             "plain key is free. Avoid M and R, which MotionTrace and PlaytestReset already own.")]
    [SerializeField] private Key overlayKey = Key.Tab;

    [Header("Facilitator input")]
    [Tooltip("Held, not tapped, so a tester leaning on the keyboard cannot wipe a session. " +
             "Keyboard only and deliberately so: the pad belongs to the tester.")]
    [SerializeField] private Key restartKey = Key.P;

    [SerializeField, Range(0.2f, 3f)] private float restartHoldSeconds = 1f;

    [Header("Copy")]
    [TextArea(6, 20)]
    [SerializeField] private string splashText =
        "MOVEMENT TEST\n\n" +
        "One craft, one gun, nothing to shoot at.\n" +
        "No enemies, no score, no goal, no end.\n" +
        "The art and the level are placeholder.\n\n" +
        "Please say everything out loud, including\n" +
        "the boring parts and especially the bad parts.\n\n" +
        "Stuck or fallen out of the world?\n" +
        "Hold SELECT for one second.\n\n" +
        "Press OPTIONS to begin.";

    [TextArea(6, 24)]
    [SerializeField] private string controlsText =
        "CONTROLS\n\n" +
        "Left stick Y      Throttle and reverse (reverse is also the brake)\n" +
        "Right stick X     Steer\n" +
        "Right stick Y     Camera. In aim mode, the nose\n\n" +
        "L2 (hold)         Aim mode. Adds strafe on left stick X, and free aim\n" +
        "L1 or R1          On the ground: drift. In the air: pitch and roll\n\n" +
        "Cross             Jump. Hold to charge. One extra jump in the air\n" +
        "L3                Boost. In aim mode with no throttle, a dodge\n" +
        "R2                Fire\n\n" +
        "Select (hold)     Un-stick\n\n" +
        "Press OPTIONS to close.";

    [Header("Optional")]
    [Tooltip("Leave empty to use the TMP default font. Only set this if the default renders wrong " +
             "in a build.")]
    [SerializeField] private TMP_FontAsset font;

    // ── Private ──────────────────────────────────────────────────────────

    private Phase _phase = Phase.Splash;

    private float _sessionTime;
    private int   _sessionIndex;
    private bool  _unlocked;
    private bool  _overlayOpen;
    private bool  _splashArmed;      // input released at least once since the splash appeared
    private bool  _prePressLogged;   // only log the first pre-unlock press per session
    private float _restartHeld;

    private PlaytestReset          _reset;
    private HoverController_Energy _energy;
    private HoverController_Weapons _weapons;

    private CanvasGroup     _splashGroup;
    private CanvasGroup     _overlayGroup;
    private TextMeshProUGUI _clockLabel;
    private TextMeshProUGUI _promptLabel;
    private TextMeshProUGUI _splashLabel;
    private TextMeshProUGUI _overlayLabel;

    private const float PromptFadeSeconds = 0.6f;

    // ── Unity Lifecycle ──────────────────────────────────────────────────

    private void Awake()
    {
        BuildUi();
    }

    private void Start()
    {
        _reset = Object.FindFirstObjectByType<PlaytestReset>();
        if (_reset == null)
        {
            Debug.LogWarning("[PlaytestSession] No PlaytestReset in the scene. Restart will reset " +
                             "the clock and the splash but will NOT move the craft, so the next " +
                             "tester starts wherever the last one stopped.", this);
        }

        var input = Object.FindFirstObjectByType<PlayerHoverInput>();
        if (input != null)
        {
            _energy  = input.GetComponent<HoverController_Energy>();
            _weapons = input.GetComponent<HoverController_Weapons>();
        }

        EnterSplash(first: true);
    }

    private void Update()
    {
        TickRestart();

        if (_phase == Phase.Splash)
        {
            TickSplash();
        }
        else
        {
            _sessionTime += Time.unscaledDeltaTime;

            if (!_unlocked && (controlsUnlockTime <= 0f || _sessionTime >= controlsUnlockTime))
                Unlock();

            // Ends itself. The splash coming back up IS the "pad down, let's
            // talk" signal, so the seated questions happen against an idle game
            // and the next tester only has to press Options.
            if (_sessionTime >= sessionLength)
            {
                RestartSession($"AUTO END after {sessionLength:F0}s");
                TickLabels();
                return;
            }

            TickOverlayToggle();
        }

        // Runs in BOTH phases, and derives every alpha from state rather than
        // being written at the transition. See TickLabels.
        TickLabels();
    }

    // ── Phases ───────────────────────────────────────────────────────────

    private void EnterSplash(bool first, string reason = null)
    {
        float ranFor    = _sessionTime;   // captured before the reset below

        _phase          = Phase.Splash;
        _sessionTime    = 0f;
        _unlocked       = controlsUnlockTime <= 0f;
        _overlayOpen    = false;
        _prePressLogged = false;
        _splashArmed    = false;   // require a release before a press counts

        _splashLabel.text  = splashText;
        _overlayLabel.text = controlsText;

        // Only the splash is snapped, so the very first frame of a session is
        // already covered rather than fading up from a visible arena. Every
        // other alpha is left to TickLabels, which will drive it from state.
        _splashGroup.alpha = 1f;

        if (!first)
        {
            // The reason is recorded because it separates a tester who ran the
            // full length from one cut short, and those are not comparable.
            Debug.Log($"[PlaytestSession] SESSION {_sessionIndex} END at t={Time.unscaledTime:F2}s, " +
                      $"ran {ranFor:F1}s, {reason}. " +
                      "Rows above this line belong to that tester.", this);
        }
    }

    private void TickSplash()
    {
        // A press is only accepted after the button has been released once.
        // Without this, a thumb still resting on Options skips the new splash
        // the instant a restart builds it.
        if (!_splashArmed)
        {
            if (!OverlayToggleHeld()) _splashArmed = true;
            return;
        }

        if (!OverlayTogglePressed()) return;

        _sessionIndex++;
        _phase       = Phase.Running;
        _sessionTime = 0f;

        Debug.Log($"[PlaytestSession] SESSION {_sessionIndex} START at t={Time.unscaledTime:F2}s. " +
                  $"Controls unlock after {controlsUnlockTime:F0}s of play.", this);
    }

    private void Unlock()
    {
        _unlocked = true;
        Debug.Log($"[PlaytestSession] CONTROLS UNLOCKED, session {_sessionIndex}, " +
                  $"session t={_sessionTime:F1}s (t={Time.unscaledTime:F2}s).", this);
    }

    // ── Input ────────────────────────────────────────────────────────────

    private void TickOverlayToggle()
    {
        if (!OverlayTogglePressed()) return;

        if (!_unlocked)
        {
            // Not a misfire. This is the moment the tester stopped guessing and
            // went looking for help, which is a discoverability reading and the
            // only place it gets recorded.
            if (!_prePressLogged)
            {
                _prePressLogged = true;
                Debug.Log($"[PlaytestSession] CONTROLS REQUESTED BEFORE UNLOCK, " +
                          $"session {_sessionIndex}, session t={_sessionTime:F1}s. " +
                          "This is when they gave up guessing.", this);
            }
            return;
        }

        _overlayOpen = !_overlayOpen;
        Debug.Log($"[PlaytestSession] OVERLAY {(_overlayOpen ? "ON" : "OFF")}, " +
                  $"session {_sessionIndex}, session t={_sessionTime:F1}s.", this);
    }

    private void TickRestart()
    {
        var kb = Keyboard.current;
        if (kb == null || !kb[restartKey].isPressed)
        {
            _restartHeld = 0f;
            return;
        }

        _restartHeld += Time.unscaledDeltaTime;
        if (_restartHeld < restartHoldSeconds) return;

        _restartHeld = 0f;
        RestartSession("MANUAL, P held");
    }

    private bool OverlayTogglePressed()
    {
        var pad = Gamepad.current;
        if (pad != null && pad[overlayButton].wasPressedThisFrame) return true;

        var kb = Keyboard.current;
        return kb != null && kb[overlayKey].wasPressedThisFrame;
    }

    private bool OverlayToggleHeld()
    {
        var pad = Gamepad.current;
        if (pad != null && pad[overlayButton].isPressed) return true;

        var kb = Keyboard.current;
        return kb != null && kb[overlayKey].isPressed;
    }

    // ── Session restart ──────────────────────────────────────────────────

    private void RestartSession(string reason)
    {
        // Order matters only in that the craft should be home before the next
        // tester sees the splash over the top of it.
        if (_reset != null) _reset.ResetNow();

        // Kept even though PlaytestReset now refills energy too, because the two
        // are answering different questions and only one of them is optional.
        // There, restoring is a playtest convenience with a tickbox to turn it
        // off for measurement runs. Here it is a correctness requirement: the
        // craft is being handed to a NEW PERSON, and starting their blind
        // ninety seconds on the last tester's drained meter would deny them
        // boost and jump for exactly the window this exists to measure. If that
        // tickbox is ever unticked, this line is what keeps handover correct.
        //
        // Grant refuses while EMP-frozen, which cannot arise in this build (EMP
        // is disabled) but would leave the meter short if it ever did.
        if (_energy != null) _energy.Grant(999999f);

        // Cheap insurance: if a tester cycled off the machine gun, the next one
        // should not start holding something else.
        if (_weapons != null && _weapons.ActiveSlotIndex != 0) _weapons.SetActiveSlot(0);

        EnterSplash(first: false, reason);
    }

    // ── Presentation ─────────────────────────────────────────────────────

    /// <summary>
    /// Every alpha on screen is SOLVED FROM STATE here, once per frame, and
    /// nothing else writes one. The first draft set them at the phase
    /// transitions instead, which meant any path that reached a phase without
    /// going through its transition left the UI describing the wrong state. The
    /// failure that matters is the opaque splash surviving into a live session,
    /// which would end a tester's block. Deriving costs nothing and cannot
    /// desynchronise.
    /// </summary>
    private void TickLabels()
    {
        bool  splash = _phase == Phase.Splash;
        float dt     = Time.unscaledDeltaTime;

        _splashGroup.alpha = Mathf.MoveTowards(_splashGroup.alpha, splash ? 1f : 0f, dt / 0.18f);

        if (showClock)
        {
            // Counts DOWN, so the facilitator reads time remaining at a glance
            // instead of doing arithmetic. Ceil rather than truncate, or the
            // last whole second displays as 00:00 while play continues.
            int left = Mathf.CeilToInt(Mathf.Max(0f, sessionLength - _sessionTime));
            _clockLabel.text = $"{left / 60:00}:{left % 60:00}";
        }
        _clockLabel.alpha = Mathf.MoveTowards(_clockLabel.alpha,
                                              (showClock && !splash) ? 0.32f : 0f, dt / 0.4f);

        float promptTarget = (!splash && _unlocked && !_overlayOpen) ? 0.55f : 0f;
        _promptLabel.alpha = Mathf.MoveTowards(_promptLabel.alpha, promptTarget,
                                               dt / PromptFadeSeconds);

        _overlayGroup.alpha = Mathf.MoveTowards(_overlayGroup.alpha,
                                                (!splash && _overlayOpen) ? 1f : 0f, dt / 0.18f);
    }

    // ── UI construction ──────────────────────────────────────────────────

    private void BuildUi()
    {
        var root = new GameObject("PlaytestUI",
                                  typeof(Canvas), typeof(CanvasScaler));
        root.transform.SetParent(transform, false);

        var canvas = root.GetComponent<Canvas>();
        canvas.renderMode  = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = 100;   // every canvas already in the scene sits at 0

        var scaler = root.GetComponent<CanvasScaler>();
        scaler.uiScaleMode         = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.matchWidthOrHeight  = 0.5f;

        // No GraphicRaycaster on purpose: nothing here is clickable, and adding
        // one puts a second raycaster in a scene that already has an EventSystem.

        // Splash: full bleed, because it is the only thing on screen.
        _splashGroup = MakePanel(root.transform, "Splash", new Color(0.03f, 0.04f, 0.05f, 0.96f),
                                 stretch: true, size: default);
        _splashLabel = MakeText(_splashGroup.transform, "SplashText", 34f,
                                TextAlignmentOptions.Center, new Vector2(180f, 70f));

        // Overlay: a centred card, so peripheral vision survives. See class notes.
        _overlayGroup = MakePanel(root.transform, "Overlay", new Color(0.03f, 0.04f, 0.05f, 0.90f),
                                  stretch: false, size: new Vector2(1080f, 700f));
        _overlayLabel = MakeText(_overlayGroup.transform, "OverlayText", 26f,
                                 TextAlignmentOptions.Left, new Vector2(56f, 40f));

        _clockLabel  = MakeCorner(root.transform, "Clock", 26f, TextAlignmentOptions.TopRight,
                                  new Vector2(1f, 1f), new Vector2(-40f, -28f));
        _promptLabel = MakeCorner(root.transform, "Prompt", 26f, TextAlignmentOptions.Bottom,
                                  new Vector2(0.5f, 0f), new Vector2(0f, 52f));
        _promptLabel.text = "OPTIONS  for controls";
    }

    private CanvasGroup MakePanel(Transform parent, string name, Color color,
                                  bool stretch, Vector2 size)
    {
        var go = new GameObject(name, typeof(RectTransform), typeof(CanvasGroup), typeof(Image));
        go.transform.SetParent(parent, false);

        var rt = (RectTransform)go.transform;
        if (stretch)
        {
            rt.anchorMin = Vector2.zero;
            rt.anchorMax = Vector2.one;
            rt.offsetMin = Vector2.zero;
            rt.offsetMax = Vector2.zero;
        }
        else
        {
            rt.anchorMin = rt.anchorMax = new Vector2(0.5f, 0.5f);
            rt.pivot     = new Vector2(0.5f, 0.5f);
            rt.sizeDelta = size;
            rt.anchoredPosition = Vector2.zero;
        }

        go.GetComponent<Image>().color = color;
        go.GetComponent<Image>().raycastTarget = false;

        return go.GetComponent<CanvasGroup>();
    }

    /// <summary>Stretched to its parent, inset by <paramref name="pad"/> on each axis.</summary>
    private TextMeshProUGUI MakeText(Transform parent, string name, float size,
                                     TextAlignmentOptions align, Vector2 pad)
    {
        var go = new GameObject(name, typeof(RectTransform), typeof(TextMeshProUGUI));
        go.transform.SetParent(parent, false);

        var rt = (RectTransform)go.transform;
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = new Vector2( pad.x,  pad.y);
        rt.offsetMax = new Vector2(-pad.x, -pad.y);

        var t = go.GetComponent<TextMeshProUGUI>();
        if (font != null) t.font = font;
        t.fontSize      = size;
        t.alignment     = align;
        t.color         = new Color(0.90f, 0.93f, 0.92f, 1f);
        t.raycastTarget = false;
        t.richText      = true;
        return t;
    }

    private TextMeshProUGUI MakeCorner(Transform parent, string name, float size,
                                       TextAlignmentOptions align, Vector2 anchor,
                                       Vector2 offset)
    {
        var go = new GameObject(name, typeof(RectTransform), typeof(TextMeshProUGUI));
        go.transform.SetParent(parent, false);

        var rt = (RectTransform)go.transform;
        rt.anchorMin = rt.anchorMax = anchor;
        rt.pivot     = anchor;
        rt.sizeDelta = new Vector2(520f, 60f);
        rt.anchoredPosition = offset;

        var t = go.GetComponent<TextMeshProUGUI>();
        if (font != null) t.font = font;
        t.fontSize      = size;
        t.alignment     = align;
        t.color         = new Color(0.90f, 0.93f, 0.92f, 1f);
        t.raycastTarget = false;
        return t;
    }
}
