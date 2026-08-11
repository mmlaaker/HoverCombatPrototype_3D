using UnityEditor;
using UnityEngine;

/// <summary>
/// TuningBaseline v1.0
/// -------------------
/// Prefab-style override marking for ScriptableObject tuning assets: changed fields draw BOLD with
/// their previous value beside them, and right-clicking one offers a revert.
///
/// Unity gives this away free on prefab instances, and deliberately not here. Override bolding
/// works by diffing an instance against its prefab parent, and an asset like VTP_Default has no
/// parent to diff against. So the diff needs a baseline, and the whole design turns on which one.
///
/// **The class defaults are the wrong baseline, and it is not close.** Measured 2026-08-10:
/// VTP_Default differs from its own field initialisers in 40 of 93 fields. Bolding those lights up
/// nearly half the inspector permanently, which trains you to ignore the bold within a day and
/// leaves the feature worse than useless, because now there is a signal that means nothing.
///
/// The baseline is therefore a SNAPSHOT taken at the start of a tuning session, which answers the
/// question actually being asked mid-pass: what have I moved, and what was it before? It lives in
/// SessionState, so it survives recompiles and domain reloads and dies when Unity closes. That is
/// exactly a session's worth of lifetime, and it means a baseline can never go stale across days
/// or get committed to disk.
///
/// Showing the OLD VALUE is the part prefabs do not do, and during tuning it is the most useful
/// half: bold tells you that you moved something, the suffix tells you how far.
///
/// Two implementation notes worth knowing before editing this file:
///
///   1. **The bold is done by mutating EditorStyles.label, not by SetBoldDefaultFont.**
///      EditorGUIUtility.SetBoldDefaultFont is the method Unity uses internally for prefab
///      overrides, and it is NOT public (verified by reflection, 2026-08-10). Reaching it needs
///      reflection into a private API that can be renamed in any Unity update, and a silent
///      failure there would look exactly like "nothing is bold", which is also what "nothing has
///      changed" looks like. The public path is coarser: it catches ordinary float, slider and
///      toggle labels, and may not bold a foldout header. That is an acceptable trade for not
///      depending on a private method. The style is restored in a finally, because EditorStyles is
///      global and a leaked bold would follow you into every other inspector.
///
///   2. **Undo and dirty are NOT this file's job and never were.** SerializedObject
///      .ApplyModifiedProperties already registers an undo step and flags the target dirty; that is
///      the entire reason ApplyModifiedPropertiesWithoutUndo exists as a separate method. Reverts
///      here go through the same call, so they land on the undo stack like any other edit.
/// </summary>
public static class TuningBaseline
{
    private const string KeyPrefix = "TuningBaseline.";

    // Static because only one inspector is drawing at a time: every PropertyField call happens
    // between a Begin and the end of that same OnInspectorGUI. Two inspector windows locked to
    // two different profiles still work, because Begin re-resolves on every repaint.
    private static string           _key;
    private static string           _json;
    private static ScriptableObject _clone;
    private static SerializedObject _baseline;
    private static SerializedObject _live;
    private static bool             _active;

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /// <summary>
    /// Resolves the baseline for this target, capturing one if the session has none yet. Pass a
    /// null target to draw normally, which is what a multi-object selection should do: the diff
    /// would have to pick one object's values and then report them as though they described all
    /// of them.
    /// </summary>
    public static void Begin(SerializedObject live, Object target)
    {
        _active = false;
        _live   = live;

        if (live == null || target is not ScriptableObject)
            return;

        string path = AssetDatabase.GetAssetPath(target);

        // An unsaved in-memory instance has no stable identity to key a session snapshot against.
        if (string.IsNullOrEmpty(path))
            return;

        string key  = KeyPrefix + AssetDatabase.AssetPathToGUID(path);
        string json = SessionState.GetString(key, null);

        if (string.IsNullOrEmpty(json))
        {
            json = EditorJsonUtility.ToJson(target);
            SessionState.SetString(key, json);
        }

        // The clone is a real instance of the same type holding the snapshot's values, so the
        // comparison runs through SerializedProperty rather than through parsed JSON and therefore
        // agrees with the inspector about what a "field" is. Rebuilt only when the snapshot
        // changes, or when a domain reload has taken the previous one with it.
        if (_clone == null || _key != key || _json != json)
        {
            if (_clone != null)
                Object.DestroyImmediate(_clone);

            _clone = ScriptableObject.CreateInstance(target.GetType());
            _clone.hideFlags = HideFlags.HideAndDontSave;
            EditorJsonUtility.FromJsonOverwrite(json, _clone);

            _baseline = new SerializedObject(_clone);
            _key      = key;
            _json     = json;
        }

        _baseline.Update();
        _active = true;
    }

    // -------------------------------------------------------------------------
    // Drawing
    // -------------------------------------------------------------------------

    /// <summary>
    /// The session bar. Draws nothing when there is no baseline to talk about, so a multi-object
    /// selection or an unsaved instance simply gets the ordinary inspector.
    /// </summary>
    public static void DrawBar()
    {
        if (!_active) return;

        CountChanged(out int changed, out int total);

        using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
        {
            using (new EditorGUILayout.HorizontalScope())
            {
                string summary = changed == 0
                    ? $"Baseline captured. No changes yet ({total} fields)."
                    : $"{changed} of {total} fields changed since baseline.";

                using (new EditorGUI.DisabledScope(true))
                    EditorGUILayout.LabelField(summary, EditorStyles.miniLabel);

                if (GUILayout.Button(new GUIContent("Capture",
                        "Make the current values the new baseline. Everything stops being bold " +
                        "until you change something again."), EditorStyles.miniButton,
                        GUILayout.Width(64)))
                {
                    Capture();
                }

                using (new EditorGUI.DisabledScope(changed == 0))
                {
                    if (GUILayout.Button(new GUIContent("Revert All",
                            "Put every changed field back to its baseline value. Undoable."),
                            EditorStyles.miniButton, GUILayout.Width(76)))
                    {
                        RevertAll(changed);
                    }
                }
            }

            if (changed > 0)
                using (new EditorGUI.DisabledScope(true))
                    EditorGUILayout.LabelField(
                        "Right-click a bold field to revert just that one.",
                        EditorStyles.wordWrappedMiniLabel);
        }

        EditorGUILayout.Space(4);
    }

    /// <summary>
    /// Drop-in replacement for EditorGUILayout.PropertyField. Falls back to the plain call when no
    /// baseline is active, so call sites never have to branch.
    /// </summary>
    public static void PropertyField(SerializedProperty prop, bool includeChildren = true)
    {
        SerializedProperty baseProp = _active ? _baseline.FindProperty(prop.propertyPath) : null;
        bool changed = baseProp != null && !SerializedProperty.DataEquals(prop, baseProp);

        if (!changed)
        {
            EditorGUILayout.PropertyField(prop, includeChildren);
            return;
        }

        // The old value rides in the label rather than on its own row, so a changed field does not
        // reflow the inspector around it. The tooltip is carried across explicitly: these tuning
        // tooltips are the documentation, and dropping them to gain a suffix would be a bad trade.
        string was     = Describe(baseProp);
        string label   = was != null
            ? $"{prop.displayName}  (was {was})"
            : prop.displayName;

        FontStyle previous = EditorStyles.label.fontStyle;

        try
        {
            EditorStyles.label.fontStyle = FontStyle.Bold;
            EditorGUILayout.PropertyField(prop, new GUIContent(label, prop.tooltip), includeChildren);
        }
        finally
        {
            EditorStyles.label.fontStyle = previous;
        }

        HandleRevertMenu(prop.propertyPath, GUILayoutUtility.GetLastRect());
    }

    // -------------------------------------------------------------------------
    // Actions
    // -------------------------------------------------------------------------

    private static void Capture()
    {
        if (_live == null || _live.targetObject == null) return;

        string path = AssetDatabase.GetAssetPath(_live.targetObject);
        if (string.IsNullOrEmpty(path)) return;

        SessionState.SetString(KeyPrefix + AssetDatabase.AssetPathToGUID(path),
                               EditorJsonUtility.ToJson(_live.targetObject));

        // Forces Begin to rebuild the clone on the next repaint rather than trusting the cache.
        _json = null;
    }

    private static void RevertAll(int changed)
    {
        if (!EditorUtility.DisplayDialog(
                "Revert to baseline",
                $"Put all {changed} changed fields back to their baseline values?\n\n" +
                "This is a single undo step.",
                "Revert All", "Cancel"))
            return;

        SerializedProperty it = _baseline.GetIterator();
        bool enterChildren = true;

        while (it.NextVisible(enterChildren))
        {
            enterChildren = true;

            if (it.propertyPath == "m_Script") continue;
            if (it.hasVisibleChildren)         continue;

            _live.CopyFromSerializedProperty(it);
        }

        _live.ApplyModifiedProperties();
    }

    /// <summary>
    /// Right-click revert for one field. The property PATH is captured rather than the
    /// SerializedProperty, because the menu callback fires on a later event by which time the
    /// iterator that produced it has moved on.
    /// </summary>
    private static void HandleRevertMenu(string propertyPath, Rect rect)
    {
        Event e = Event.current;

        if (e.type != EventType.ContextClick || !rect.Contains(e.mousePosition))
            return;

        SerializedObject live     = _live;
        SerializedObject baseline = _baseline;

        var menu = new GenericMenu();
        menu.AddItem(new GUIContent("Revert to baseline"), false, () =>
        {
            SerializedProperty b = baseline.FindProperty(propertyPath);
            if (b == null) return;

            live.Update();
            live.CopyFromSerializedProperty(b);
            live.ApplyModifiedProperties();
        });
        menu.ShowAsContext();

        e.Use();
    }

    // -------------------------------------------------------------------------
    // Plumbing
    // -------------------------------------------------------------------------

    private static void CountChanged(out int changed, out int total)
    {
        changed = 0;
        total   = 0;

        SerializedProperty it = _live.GetIterator();
        bool enterChildren = true;

        while (it.NextVisible(enterChildren))
        {
            enterChildren = true;

            if (it.propertyPath == "m_Script") continue;
            if (it.hasVisibleChildren)         continue;

            total++;

            SerializedProperty b = _baseline.FindProperty(it.propertyPath);

            if (b != null && !SerializedProperty.DataEquals(it, b))
                changed++;
        }
    }

    /// <summary>
    /// Renders a baseline value compactly enough to sit inside a field label. Returns null for
    /// anything that has no short form, in which case the field still bolds and still reverts; it
    /// just does not advertise what it was.
    /// </summary>
    private static string Describe(SerializedProperty p)
    {
        switch (p.propertyType)
        {
            case SerializedPropertyType.Float:
                return p.floatValue.ToString("0.###");

            case SerializedPropertyType.Integer:
                return p.intValue.ToString();

            case SerializedPropertyType.Boolean:
                return p.boolValue ? "on" : "off";

            case SerializedPropertyType.Enum:
                return p.enumValueIndex >= 0 && p.enumValueIndex < p.enumDisplayNames.Length
                    ? p.enumDisplayNames[p.enumValueIndex]
                    : null;

            case SerializedPropertyType.String:
                return string.IsNullOrEmpty(p.stringValue) ? "empty" : $"\"{p.stringValue}\"";

            case SerializedPropertyType.ObjectReference:
                return p.objectReferenceValue != null ? p.objectReferenceValue.name : "none";

            case SerializedPropertyType.Vector2:
                return p.vector2Value.ToString("0.##");

            case SerializedPropertyType.Vector3:
                return p.vector3Value.ToString("0.##");

            case SerializedPropertyType.LayerMask:
                return p.intValue == 0 ? "nothing" : p.intValue == -1 ? "everything" : $"mask {p.intValue}";

            default:
                return null;
        }
    }
}
