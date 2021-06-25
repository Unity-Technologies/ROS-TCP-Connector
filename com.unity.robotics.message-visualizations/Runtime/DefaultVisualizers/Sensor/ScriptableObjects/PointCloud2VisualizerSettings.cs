using System;
using RosMessageTypes.Sensor;
using UnityEngine;

[CreateAssetMenu(fileName = "PointCloud2VisualizerSettings", menuName = "MessageVisualizations/Sensor/PointCloud2", order = 1)]
public class PointCloud2VisualizerSettings : ScriptableObject
{
    public string topic;
    public ColorMode colorMode;

    public string xChannel = "x";
    public string yChannel = "y";
    public string zChannel = "z";
    public string rgbChannel = "x";
    public string rChannel = "x";
    public string gChannel = "y";
    public string bChannel = "z";
    public string sizeChannel = "x";

    public float[] rgbRange = { 0, 31 };
    public float[] rRange = { -100, 100 };
    public float[] gRange = { -100, 100 };
    public float[] bRange = { -100, 100 };
    public float[] sizeRange = { 0, 100 };
    public float size = 0.01f;

    public bool useRgbChannel;
    public bool useSizeChannel;
    public MPointField[] channels;
}
