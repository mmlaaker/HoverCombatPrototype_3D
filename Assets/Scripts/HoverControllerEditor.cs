#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System.IO;
using System.Collections.Generic;

[CustomEditor(typeof(HoverController))]
public class HoverControllerEditor : Editor
{
    private Vector2 _overviewScroll;
    private bool _showOverview = true;

    private string _presetName = "";
    private string _selectedPreset = "";
    private bool _liveApply = false;

    private Dictionary<string, HoverPreset> _presets = new();

    private string PresetsFolder => Path.Combine(Application.dataPath, "Presets");
    private string PresetsPath   => Path.Combine(PresetsFolder, "HoverPresets.json");

    private void OnEnable()
    {
        LoadPresets();
    }

    public override void OnInspectorGUI()
    {
        var controller = (HoverController)target;

        // ─────────────────────────────
        // SYSTEM OVERVIEW (Foldout + Scroll)
        // ─────────────────────────────
        _showOverview = EditorGUILayout.Foldout(_showOverview, "System Overview", true, EditorStyles.foldoutHeader);
        if (_showOverview)
        {
            var textStyle = new GUIStyle(EditorStyles.textArea)
            {
                wordWrap = true,
                richText = true,
                fontSize = 10,
                padding = new RectOffset(10, 10, 10, 10)
            };
            // Keep readable color while still disabling editing
            textStyle.normal.textColor = EditorStyles.label.normal.textColor;

            _overviewScroll = EditorGUILayout.BeginScrollView(_overviewScroll, GUILayout.Height(240));
            bool prev = GUI.enabled;
            GUI.enabled = false; // disable editing only (not the scroll view)
            EditorGUILayout.TextArea(controller.GetOverviewText(), textStyle);
            GUI.enabled = prev;
            EditorGUILayout.EndScrollView();
        }

        EditorGUILayout.Space(10);
        DrawDefaultInspector();   // your regular fields

        EditorGUILayout.Space(12);
        DrawPresetUI(controller);
    }

    // ─────────────────────────────
    // PRESET UI
    // ─────────────────────────────
    private void DrawPresetUI(HoverController c)
    {
        EditorGUILayout.LabelField("Presets", EditorStyles.boldLabel);
        EditorGUILayout.HelpBox(
            "Save/load named hover presets (stored at Assets/Presets/HoverPresets.json). " +
            "Enable Live Apply to preview instantly when selecting a preset.",
            MessageType.Info);

        // Save row
        EditorGUILayout.BeginHorizontal();
        _presetName = EditorGUILayout.TextField("Preset Name", _presetName);
        if (GUILayout.Button("Save", GUILayout.Width(60)))
            SavePreset(c, _presetName);
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.Space(4);

        _liveApply = EditorGUILayout.ToggleLeft("Live Apply Selected Preset", _liveApply);

        if (_presets.Count == 0)
        {
            EditorGUILayout.HelpBox("No presets saved yet.", MessageType.None);
            return;
        }

        // Dropdown
        var keys = new List<string>(_presets.Keys);
        int currentIndex = Mathf.Max(0, keys.IndexOf(_selectedPreset));
        int newIndex = EditorGUILayout.Popup("Select Preset", currentIndex, keys.ToArray());

        if (newIndex >= 0 && newIndex < keys.Count)
        {
            string newSel = keys[newIndex];
            if (newSel != _selectedPreset)
            {
                _selectedPreset = newSel;
                if (_liveApply)
                    ApplyPreset(c, _presets[_selectedPreset]);
            }
        }

        EditorGUILayout.Space(6);

        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Load", GUILayout.Width(70)) && _presets.ContainsKey(_selectedPreset))
            ApplyPreset(c, _presets[_selectedPreset]);

        if (GUILayout.Button("Delete", GUILayout.Width(70)) && _presets.ContainsKey(_selectedPreset))
        {
            _presets.Remove(_selectedPreset);
            SavePresetFile();
            _selectedPreset = "";
        }
        EditorGUILayout.EndHorizontal();
    }

    // ─────────────────────────────
    // PRESET LOGIC
    // ─────────────────────────────
    private void SavePreset(HoverController c, string name)
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            EditorUtility.DisplayDialog("Invalid Name", "Please enter a valid preset name.", "OK");
            return;
        }

        if (!Directory.Exists(PresetsFolder))
            Directory.CreateDirectory(PresetsFolder);

        _presets[name] = new HoverPreset(c);
        SavePresetFile();

        _selectedPreset = name;
        EditorUtility.DisplayDialog("Preset Saved", $"Saved preset: {name}", "OK");
    }

    private void ApplyPreset(HoverController c, HoverPreset p)
    {
        Undo.RecordObject(c, "Apply Hover Preset");

        c.hoverHeight = p.hoverHeight;
        c.hoverForce = p.hoverForce;
        c.hoverDamp = p.hoverDamp;
        c.preloadBias = p.preloadBias;

        c.thrustAccel = p.thrustAccel;
        c.turnTorque = p.turnTorque;
        c.maxSpeed   = p.maxSpeed;

        c.driftDamp        = p.driftDamp;
        c.driftBoostFactor = p.driftBoostFactor;
        c.verticalDamp     = p.verticalDamp;
        c.alignStrength    = p.alignStrength;

        c.minGroundedRatio   = p.minGroundedRatio;
        c.groundedBlendSpeed = p.groundedBlendSpeed;
        c.airAngularDrag     = p.airAngularDrag;

        c.linearDrag        = p.linearDrag;
        c.groundAngularDrag = p.groundAngularDrag;
        c.levelingStrength  = p.levelingStrength;
        c.angularDampFactor = p.angularDampFactor;

        c.pitchResponse = p.pitchResponse;
        c.maxPitchAngle = p.maxPitchAngle;
        c.tiltSmooth    = p.tiltSmooth;

        EditorUtility.SetDirty(c);
        if (!_liveApply)
            EditorUtility.DisplayDialog("Preset Loaded", $"Applied preset: {_selectedPreset}", "OK");
    }

    private void LoadPresets()
    {
        _presets.Clear();
        if (!File.Exists(PresetsPath)) return;

        try
        {
            string json = File.ReadAllText(PresetsPath);
            var loaded = JsonUtility.FromJson<HoverPresetCollection>(json);
            if (loaded != null && loaded.keys != null && loaded.values != null)
                _presets = loaded.ToDictionary();
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"Error loading presets: {ex.Message}");
        }
    }

    private void SavePresetFile()
    {
        var wrapper = new HoverPresetCollection(_presets);
        string json = JsonUtility.ToJson(wrapper, true);
        File.WriteAllText(PresetsPath, json);
        AssetDatabase.Refresh();
    }
}

// ─────────────────────────────
// DATA STRUCTURES
// ─────────────────────────────
[System.Serializable]
public class HoverPreset
{
    public float hoverHeight, hoverForce, hoverDamp, preloadBias;
    public float thrustAccel, turnTorque, maxSpeed;
    public float driftDamp, driftBoostFactor, verticalDamp, alignStrength;
    public float minGroundedRatio, groundedBlendSpeed, airAngularDrag;
    public float linearDrag, groundAngularDrag, levelingStrength, angularDampFactor;
    public float pitchResponse, maxPitchAngle, tiltSmooth;

    public HoverPreset(HoverController c)
    {
        hoverHeight = c.hoverHeight;
        hoverForce  = c.hoverForce;
        hoverDamp   = c.hoverDamp;
        preloadBias = c.preloadBias;

        thrustAccel = c.thrustAccel;
        turnTorque  = c.turnTorque;
        maxSpeed    = c.maxSpeed;

        driftDamp        = c.driftDamp;
        driftBoostFactor = c.driftBoostFactor;
        verticalDamp     = c.verticalDamp;
        alignStrength    = c.alignStrength;

        minGroundedRatio   = c.minGroundedRatio;
        groundedBlendSpeed = c.groundedBlendSpeed;
        airAngularDrag     = c.airAngularDrag;

        linearDrag        = c.linearDrag;
        groundAngularDrag = c.groundAngularDrag;
        levelingStrength  = c.levelingStrength;
        angularDampFactor = c.angularDampFactor;

        pitchResponse = c.pitchResponse;
        maxPitchAngle = c.maxPitchAngle;
        tiltSmooth    = c.tiltSmooth;
    }
}

[System.Serializable]
public class HoverPresetCollection
{
    public List<string> keys = new();
    public List<HoverPreset> values = new();

    public HoverPresetCollection() { }

    public HoverPresetCollection(Dictionary<string, HoverPreset> dict)
    {
        foreach (var kv in dict)
        {
            keys.Add(kv.Key);
            values.Add(kv.Value);
        }
    }

    public Dictionary<string, HoverPreset> ToDictionary()
    {
        var result = new Dictionary<string, HoverPreset>();
        for (int i = 0; i < keys.Count; i++)
        {
            if (i < values.Count)
                result[keys[i]] = values[i];
        }
        return result;
    }
}
#endif
