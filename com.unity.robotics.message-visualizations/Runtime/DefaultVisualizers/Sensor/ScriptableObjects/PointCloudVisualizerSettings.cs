using System;
using RosMessageTypes.Sensor;
using UnityEngine;

[CreateAssetMenu(fileName = "PointCloudVisualizerSettings", menuName = "MessageVisualizations/Sensor/PointCloud", order = 1)]
public class PointCloudVisualizerSettings : ScriptableObject
{
    public string topic;
    public ColorMode colorMode;
    public string rgbChannel = "";
    public string rChannel = "";
    public string gChannel = "";
    public string bChannel = "";
    public string sizeChannel = "";

    public float[] rgbRange = { 0, 100 };
    public float[] rRange = { 0, 100 };
    public float[] gRange = { 0, 100 };
    public float[] bRange = { 0, 100 };
    public float[] sizeRange = { 0, 100 };
    public float size = 0.05f;

    public bool useRgbChannel;
    public bool useSeparateRgb = true;
    public bool useSizeChannel;
    public MChannelFloat32[] channels;
}
