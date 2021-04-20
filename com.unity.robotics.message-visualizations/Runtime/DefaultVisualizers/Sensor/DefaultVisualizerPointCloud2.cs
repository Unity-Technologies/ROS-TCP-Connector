using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif // UNITY_EDITOR

public enum ColorMode
{
    HSV, RGB
}

public class DefaultVisualizerPointCloud2 : BasicVisualizer<MPointCloud2>
{
    public Pcl2Channels m_ChannelConfigs;
    
    public override void Draw(BasicDrawing drawing, MPointCloud2 message, MessageMetadata meta, Color color, string label)
    {
        if (m_ChannelConfigs.channels == null)
            m_ChannelConfigs.channels = message.fields;
        
        message.Draw<FLU>(drawing, color, m_ChannelConfigs);
    }

    public override Action CreateGUI(MPointCloud2 message, MessageMetadata meta, BasicDrawing drawing)
    {
        var formatDict = new Dictionary<PointFieldFormat, List<string>>();

        foreach (MPointField field in message.fields)
        {
            if (formatDict.ContainsKey((PointFieldFormat)field.datatype))
                formatDict[(PointFieldFormat)field.datatype].Add(field.name);
            else 
                formatDict.Add((PointFieldFormat)field.datatype, new List<string>() { field.name });
        }

        string formats = "";
        foreach (var f in formatDict)
        {
            if (f.Value.Count > 0)
                formats += $"{f.Key}: {String.Join(", ", f.Value)}\n";
        }

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nData length: {message.data.Length}\nPoint step: {message.point_step}\nRow step: {message.row_step}\nIs dense: {message.is_dense}");
            GUILayout.Label($"Channels:\n{formats}");
        };
    }
}

[Serializable]
public class Pcl2Channels
{
    public ColorMode colorMode;
    public MPointField[] channels;

    public string m_XChannel = "x";
    public string m_YChannel = "y";
    public string m_ZChannel = "z";
    public string m_RgbChannel = "ring";
    public string m_RChannel = "x";
    public string m_GChannel = "y";
    public string m_BChannel = "z";
    public string m_SizeChannel = "intensity";

    public float[] m_RgbRange = new float[] { 0, 31 };
    public float[] m_RRange = new float[] { -100, 100 };
    public float[] m_GRange = new float[] { -100, 100 };
    public float[] m_BRange = new float[] { -100, 100 };
    public float[] m_SizeRange = new float[] { 0, 100 };
    public float m_Size = 0.01f;

    public bool m_UseRgbChannel = false;
    public bool m_UseSizeChannel = false;

}

#if UNITY_EDITOR

[CustomEditor(typeof(DefaultVisualizerPointCloud2))]
public class PointCloud2Editor : Editor
{
    Pcl2Channels pcl2Config;
    string colorMin = "0";
    string colorMax = "1000";
    float colorMinVal = 0;
    float colorMaxVal = 1000;
    string sizeMin = "0";
    string sizeMax = "1000";
    float sizeMinVal = 0;
    float sizeMaxVal = 1000;

    void OnEnable()
    {
        pcl2Config = ((DefaultVisualizerPointCloud2)target).m_ChannelConfigs;
    }

    void CreateNewDropdown(string label, string channel, Action<string> action)
    {
        if (pcl2Config.channels == null)
        {
            Debug.LogWarning("Channels were null!");
            return;
        }

        GUILayout.BeginHorizontal();
        GUILayout.Label(label);
        if (EditorGUILayout.DropdownButton(new GUIContent(channel), FocusType.Keyboard))
        {
            GenericMenu menu = new GenericMenu();
            foreach (var c in pcl2Config.channels)
            {
                menu.AddItem(new GUIContent(c.name), c.name == channel, () => { 
                    action(c.name);
                });
            }
            menu.DropDown(new Rect(Event.current.mousePosition.x, Event.current.mousePosition.y, 0f, 0f));
        }
        GUILayout.EndHorizontal();
    }

    void CreateMinMaxSlider(ref float[] range, float min, float max)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(range[0].ToString());
        EditorGUILayout.MinMaxSlider(ref range[0], ref range[1], min, max);
        GUILayout.Label(range[1].ToString());
        GUILayout.EndHorizontal();
    }

    void MinMaxText(string label, ref float minVal, ref string minS, ref float maxVal, ref string maxS)
    {
        minVal = float.Parse(EditorGUILayout.TextField($"Min {label} range:", minS));
        minS = minVal.ToString();

        maxVal = float.Parse(EditorGUILayout.TextField($"Max {label} range:", maxS));
        maxS = maxVal.ToString();
    }

    public override void OnInspectorGUI()
    {
        pcl2Config.m_XChannel = EditorGUILayout.TextField("X channel name:", pcl2Config.m_XChannel);
        pcl2Config.m_YChannel = EditorGUILayout.TextField("Y channel name:", pcl2Config.m_YChannel);
        pcl2Config.m_ZChannel = EditorGUILayout.TextField("Z channel name:", pcl2Config.m_ZChannel);

        pcl2Config.m_UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", pcl2Config.m_UseSizeChannel);

        if (pcl2Config.m_UseSizeChannel)
        {
            MinMaxText("size", ref sizeMinVal, ref sizeMin, ref sizeMaxVal, ref sizeMax);
            CreateNewDropdown("Size channel name:", pcl2Config.m_SizeChannel, (newChannel) => { pcl2Config.m_SizeChannel = newChannel; });
            CreateMinMaxSlider(ref pcl2Config.m_SizeRange, sizeMinVal, sizeMaxVal);
        }

        pcl2Config.m_UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", pcl2Config.m_UseRgbChannel);

        if (pcl2Config.m_UseRgbChannel)
        {
            pcl2Config.colorMode = (ColorMode)EditorGUILayout.EnumPopup("Color mode", pcl2Config.colorMode);

            MinMaxText("color", ref colorMinVal, ref colorMin, ref colorMaxVal, ref colorMax);

            switch (pcl2Config.colorMode)
            {
                case ColorMode.HSV:
                    CreateNewDropdown("RGB channel name:", pcl2Config.m_RgbChannel, (newChannel) => { pcl2Config.m_RgbChannel = newChannel; });
                    CreateMinMaxSlider(ref pcl2Config.m_RgbRange, colorMinVal, colorMaxVal);
                    break;
                case ColorMode.RGB:
                    CreateNewDropdown("R channel name:", pcl2Config.m_RChannel, (newChannel) => { pcl2Config.m_RChannel = newChannel; });
                    CreateMinMaxSlider(ref pcl2Config.m_RRange, colorMinVal, colorMaxVal);

                    CreateNewDropdown("G channel name:", pcl2Config.m_GChannel, (newChannel) => { pcl2Config.m_GChannel = newChannel; });
                    CreateMinMaxSlider(ref pcl2Config.m_GRange, colorMinVal, colorMaxVal);

                    CreateNewDropdown("B channel name:", pcl2Config.m_BChannel, (newChannel) => { pcl2Config.m_BChannel = newChannel; });
                    CreateMinMaxSlider(ref pcl2Config.m_BRange, colorMinVal, colorMaxVal);
                    break;
            }
        }
   }
}
#endif //UNITY_EDITOR