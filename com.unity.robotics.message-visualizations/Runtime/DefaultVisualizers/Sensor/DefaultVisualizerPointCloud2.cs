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
    string colorMin = "0";
    string colorMax = "1000";
    float colorMinVal = 0;
    float colorMaxVal = 1000;
    string sizeMin = "0";
    string sizeMax = "1000";
    float sizeMinVal = 0;
    float sizeMaxVal = 1000;

    public override void OnInspectorGUI()
    {
        DefaultVisualizerPointCloud2 pcl2Script = (DefaultVisualizerPointCloud2)target;

        pcl2Script.m_ChannelConfigs.m_XChannel = EditorGUILayout.TextField("X channel name:", pcl2Script.m_ChannelConfigs.m_XChannel);
        pcl2Script.m_ChannelConfigs.m_YChannel = EditorGUILayout.TextField("Y channel name:", pcl2Script.m_ChannelConfigs.m_YChannel);
        pcl2Script.m_ChannelConfigs.m_ZChannel = EditorGUILayout.TextField("Z channel name:", pcl2Script.m_ChannelConfigs.m_ZChannel);

        pcl2Script.m_ChannelConfigs.m_UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", pcl2Script.m_ChannelConfigs.m_UseSizeChannel);

        if (pcl2Script.m_ChannelConfigs.m_UseSizeChannel)
        {
            sizeMin = EditorGUILayout.TextField("Min color range:", sizeMin);
            sizeMinVal = float.Parse(sizeMin);

            sizeMax = EditorGUILayout.TextField("Max color range:", sizeMax);
            sizeMaxVal = float.Parse(sizeMax);

            pcl2Script.m_ChannelConfigs.m_SizeChannel = EditorGUILayout.TextField("Size channel name:", pcl2Script.m_ChannelConfigs.m_SizeChannel);
            GUILayout.BeginHorizontal();
            GUILayout.Label(pcl2Script.m_ChannelConfigs.m_SizeRange[0].ToString());
            EditorGUILayout.MinMaxSlider(ref pcl2Script.m_ChannelConfigs.m_SizeRange[0], ref pcl2Script.m_ChannelConfigs.m_SizeRange[1], sizeMinVal, sizeMaxVal);
            GUILayout.Label(pcl2Script.m_ChannelConfigs.m_SizeRange[1].ToString());
            GUILayout.EndHorizontal();
        }

        pcl2Script.m_ChannelConfigs.m_UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", pcl2Script.m_ChannelConfigs.m_UseRgbChannel);

        if (pcl2Script.m_ChannelConfigs.m_UseRgbChannel)
        {
            pcl2Script.m_ChannelConfigs.colorMode = (ColorMode)EditorGUILayout.EnumPopup("Color mode", pcl2Script.m_ChannelConfigs.colorMode);

            colorMin = EditorGUILayout.TextField("Min color range:", colorMin);
            colorMinVal = float.Parse(colorMin);

            colorMax = EditorGUILayout.TextField("Max color range:", colorMax);
            colorMaxVal = float.Parse(colorMax);

            switch (pcl2Script.m_ChannelConfigs.colorMode)
            {
                case ColorMode.HSV:
                    pcl2Script.m_ChannelConfigs.m_RgbChannel = EditorGUILayout.TextField("RGB channel name:", pcl2Script.m_ChannelConfigs.m_RgbChannel);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(pcl2Script.m_ChannelConfigs.m_RgbRange[0].ToString());
                    EditorGUILayout.MinMaxSlider(ref pcl2Script.m_ChannelConfigs.m_RgbRange[0], ref pcl2Script.m_ChannelConfigs.m_RgbRange[1], colorMinVal, colorMaxVal);
                    GUILayout.Label(pcl2Script.m_ChannelConfigs.m_RgbRange[1].ToString());
                    GUILayout.EndHorizontal();
                    break;
                case ColorMode.RGB:
                    pcl2Script.m_ChannelConfigs.m_RChannel = EditorGUILayout.TextField("R channel name:", pcl2Script.m_ChannelConfigs.m_RChannel);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(pcl2Script.m_ChannelConfigs.m_RRange[0].ToString());
                    EditorGUILayout.MinMaxSlider(ref pcl2Script.m_ChannelConfigs.m_RRange[0], ref pcl2Script.m_ChannelConfigs.m_RRange[1], colorMinVal, colorMaxVal);
                    GUILayout.Label(pcl2Script.m_ChannelConfigs.m_RRange[1].ToString());
                    GUILayout.EndHorizontal();

                    pcl2Script.m_ChannelConfigs.m_GChannel = EditorGUILayout.TextField("G channel name:", pcl2Script.m_ChannelConfigs.m_GChannel);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(pcl2Script.m_ChannelConfigs.m_GRange[0].ToString());
                    EditorGUILayout.MinMaxSlider(ref pcl2Script.m_ChannelConfigs.m_GRange[0], ref pcl2Script.m_ChannelConfigs.m_GRange[1], colorMinVal, colorMaxVal);
                    GUILayout.Label(pcl2Script.m_ChannelConfigs.m_GRange[1].ToString());
                    GUILayout.EndHorizontal();

                    pcl2Script.m_ChannelConfigs.m_BChannel = EditorGUILayout.TextField("B channel name:", pcl2Script.m_ChannelConfigs.m_BChannel);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(pcl2Script.m_ChannelConfigs.m_BRange[0].ToString());
                    EditorGUILayout.MinMaxSlider(ref pcl2Script.m_ChannelConfigs.m_BRange[0], ref pcl2Script.m_ChannelConfigs.m_BRange[1], colorMinVal, colorMaxVal);
                    GUILayout.Label(pcl2Script.m_ChannelConfigs.m_BRange[1].ToString());
                    GUILayout.EndHorizontal();
                    break;
            }
        }
   }
}
#endif //UNITY_EDITOR