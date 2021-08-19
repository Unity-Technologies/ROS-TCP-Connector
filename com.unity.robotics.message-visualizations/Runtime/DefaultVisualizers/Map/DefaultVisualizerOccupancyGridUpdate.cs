using System;
using RosMessageTypes.Map;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOccupancyGridUpdate : TexturedDrawingVisualizer<OccupancyGridUpdateMsg>
{
    [SerializeField]
    public string OccupancyGridTopic;
    RosTopicVisualizationState visState;
    IDrawingTextureVisual m_Visual;

    IDrawingTextureVisual GetVisual(string topic)
    {
        if (m_Visual == null)
        {
            RosTopicState state = ROSConnection.GetOrCreateInstance().GetTopic(topic);
            if (state == null)
                return null;

            visState = RosTopicVisualizationState.GetOrCreate(state);
            if (visState == null)
                return null;

            m_Visual = visState.Visual as IDrawingTextureVisual;
        }
        return m_Visual;
    }

    public override void Draw(BasicDrawing drawing, OccupancyGridUpdateMsg message, MessageMetadata meta)
    {
        IDrawingTextureVisual visual = GetVisual(meta.Topic);

        int width = (int)message.width;
        int height = (int)message.height;
        var updateTexture = new Texture2D(width, height, TextureFormat.R8, true);
        updateTexture.wrapMode = TextureWrapMode.Clamp;
        updateTexture.filterMode = FilterMode.Point;
        updateTexture.SetPixelData(message.data, 0);
        updateTexture.Apply();

        if (visual.texture2D == null)
        {
            visual.texture2D = new Texture2D(width, height, TextureFormat.R8, true);
            visual.texture2D.wrapMode = TextureWrapMode.Clamp;
            visual.texture2D.filterMode = FilterMode.Point;
        }
        visual.texture2D.SetPixels(message.x, message.y, width, height, updateTexture.GetPixels(0, 0, width, height));
        visual.texture2D.Apply();

        var gridMaterial = new Material(visual.shaderMaterial);
        gridMaterial.mainTexture = visual.texture2D;

        visual.drawingObject = drawing.DrawMesh(visual.mesh, visual.drawingObject.transform.position, visual.drawingObject.transform.rotation, visual.drawingObject.transform.localScale, gridMaterial);
    }

    public override Action CreateGUI(OccupancyGridUpdateMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"(x, y): ({message.x}, {message.y})");
            GUILayout.Label($"Width x height: {message.width} x {message.height}");
        };
    }

    public override Texture2D CreateTexture(OccupancyGridUpdateMsg message, MessageMetadata meta)
    {
        IDrawingTextureVisual visual = GetVisual(meta.Topic);
        if (visual == null)
        {
            Debug.LogError($"Did not find a valid visualization window for {OccupancyGridTopic}!");
            return null;
        }

        int width = (int)message.width;
        int height = (int)message.height;
        var updateTexture = new Texture2D(width, height, TextureFormat.R8, true);
        updateTexture.wrapMode = TextureWrapMode.Clamp;
        updateTexture.filterMode = FilterMode.Point;
        updateTexture.SetPixelData(message.data, 0);
        updateTexture.Apply();
        var tex = visual.GetTexture();
        tex.SetPixels(message.x, message.y, width, height, updateTexture.GetPixels(0, 0, width, height));
        tex.Apply();
        return tex;
    }
}
