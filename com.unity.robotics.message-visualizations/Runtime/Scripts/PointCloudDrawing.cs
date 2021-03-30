using System.Collections;
using System.Collections.Generic;

using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class PointCloudDrawing : MonoBehaviour
    {
        Mesh m_Mesh;
        List<Vector3> m_Vertices = new List<Vector3>();
        List<Vector2> m_UVs = new List<Vector2>();
        List<Color32> m_Colors32 = new List<Color32>();
        List<int> m_Triangles = new List<int>();

        public static PointCloudDrawing Create(float radius, GameObject parent = null, int numPoints = 0, Material material = null)
        {
            GameObject newDrawingObj = new GameObject("PointCloud");
            if (parent != null)
                newDrawingObj.transform.parent = parent.transform;
            PointCloudDrawing newDrawing = newDrawingObj.AddComponent<PointCloudDrawing>();
            newDrawing.Init(radius, numPoints, material);
            return newDrawing;
        }

        public void Init(float radius, int numPoints = 0, Material material = null)
        {
            m_Mesh = new Mesh();
            m_Vertices.Capacity = numPoints * 4;
            m_UVs.Capacity = numPoints * 4;
            m_Colors32.Capacity = numPoints * 4;

            if (material == null)
                material = BasicDrawingManager.instance.UnlitPointCloudMaterial;

            MeshFilter mfilter = gameObject.AddComponent<MeshFilter>();
            mfilter.sharedMesh = m_Mesh;

            MeshRenderer mrenderer = gameObject.AddComponent<MeshRenderer>();
            mrenderer.material = material;
            mrenderer.material.SetFloat("_Radius", radius);
        }

        public void AddPoint(Vector3 point, Color32 color)
        {
            int start = m_Vertices.Count;

            for (int Idx = 0; Idx < 4; ++Idx)
            {
                // memory saving idea: forget the separate UV channel, just record the vertex index
                // in the (unused) color alpha channel, and have the shader deduce UVs from it
                // color.a = (byte)Idx;

                m_Vertices.Add(point);
                m_Colors32.Add(color);
            }

            m_UVs.Add(Vector2.zero);
            m_UVs.Add(Vector2.up);
            m_UVs.Add(Vector2.right);
            m_UVs.Add(Vector2.one);

            m_Triangles.Add(start + 0);
            m_Triangles.Add(start + 1);
            m_Triangles.Add(start + 2);
            m_Triangles.Add(start + 3);
            m_Triangles.Add(start + 2);
            m_Triangles.Add(start + 1);
            SetDirty();
        }

        void ClearBuffers()
        {
            m_Vertices.Clear();
            m_Colors32.Clear();
            m_UVs.Clear();
            m_Triangles.Clear();
        }

        public void Clear()
        {
            ClearBuffers();
            SetDirty();
        }

        // Bake all buffered data into a mesh. Clear the buffers.
        public void Bake()
        {
            GenerateMesh();
            ClearBuffers();
            enabled = false;
        }

        void GenerateMesh()
        {
            m_Mesh.Clear();
            m_Mesh.indexFormat = m_Vertices.Count < 65536 ? UnityEngine.Rendering.IndexFormat.UInt16 : UnityEngine.Rendering.IndexFormat.UInt32;
            m_Mesh.vertices = m_Vertices.ToArray();
            m_Mesh.colors32 = m_Colors32.ToArray();
            m_Mesh.uv = m_UVs.ToArray();
            m_Mesh.triangles = m_Triangles.ToArray();
        }

        void SetDirty()
        {
            enabled = true;
        }

        public void Update()
        {
            GenerateMesh();
            enabled = false;
        }
    }
}