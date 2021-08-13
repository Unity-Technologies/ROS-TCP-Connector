using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOccupancyGrid : DrawingVisualFactory<OccupancyGridMsg>
{
    public override void Draw(BasicDrawing drawing, OccupancyGridMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing);
    }

    static Mesh s_OccupancyGridMesh;
    static Material s_OccupancyGridMaterial;

    public static void Draw<C>(OccupancyGridMsg message, BasicDrawing drawing) where C : ICoordinateSpace, new()
    {
        Vector3 origin = message.info.origin.position.From<C>();
        Quaternion rotation = message.info.origin.orientation.From<C>();
        int width = (int)message.info.width;
        int height = (int)message.info.height;
        float scale = message.info.resolution;

        if (s_OccupancyGridMesh == null)
        {
            s_OccupancyGridMesh = new Mesh();
            s_OccupancyGridMesh.vertices = new Vector3[] { Vector3.zero, new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(1, 0, 0) };
            s_OccupancyGridMesh.uv = new Vector2[] { Vector2.zero, Vector2.up, Vector2.one, Vector2.right };
            s_OccupancyGridMesh.triangles = new int[] { 0, 1, 2, 2, 3, 0, };
            s_OccupancyGridMaterial = new Material(Shader.Find("Unlit/OccupancyGrid"));
        }

        if (s_OccupancyGridMaterial == null)
            s_OccupancyGridMaterial = new Material(Shader.Find("Unlit/Color"));

        Texture2D gridTexture = new Texture2D(width, height, TextureFormat.R8, true);
        gridTexture.wrapMode = TextureWrapMode.Clamp;
        gridTexture.filterMode = FilterMode.Point;
        gridTexture.SetPixelData(message.data, 0);
        gridTexture.Apply();

        Material gridMaterial = new Material(s_OccupancyGridMaterial);
        gridMaterial.mainTexture = gridTexture;

        drawing.DrawMesh(s_OccupancyGridMesh, origin - rotation * new Vector3(scale * 0.5f, 0, scale * 0.5f), rotation, new Vector3(width * scale, 1, height * scale), gridMaterial);
    }

    public override Action CreateGUI(OccupancyGridMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.info.GUI();
        };
    }
}
