using System.Collections;
using System.Collections.Generic;

using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class PointCloudDrawing : MonoBehaviour
    {
        static readonly float k_SqrtPointFive = Mathf.Sqrt(0.5f);

        Mesh m_Mesh;
        List<Vector3> m_Vertices = new List<Vector3>();
        List<Vector3> m_UVRs = new List<Vector3>(); // texture UV and point radius
        List<Color32> m_Colors32 = new List<Color32>();
        List<int> m_Triangles = new List<int>();
        MeshRenderer m_MeshRenderer;

        public static PointCloudDrawing Create(GameObject parent = null, int numPoints = 0, Material material = null)
        {
            GameObject newDrawingObj = new GameObject("PointCloud");
            if (parent != null)
            {
                newDrawingObj.transform.parent = parent.transform;
                newDrawingObj.transform.localPosition = Vector3.zero;
                newDrawingObj.transform.localRotation = Quaternion.identity;
            }
            PointCloudDrawing newDrawing = newDrawingObj.AddComponent<PointCloudDrawing>();
            newDrawing.SetCapacity(numPoints);
            newDrawing.SetMaterial(material);
            return newDrawing;
        }

        public void Awake()
        {
            m_Mesh = new Mesh();

            MeshFilter mfilter = gameObject.AddComponent<MeshFilter>();
            mfilter.sharedMesh = m_Mesh;

            m_MeshRenderer = gameObject.AddComponent<MeshRenderer>();
        }

        public void SetCapacity(int numPoints)
        {
            m_Vertices.Capacity = numPoints * 4;
            m_UVRs.Capacity = numPoints * 4;
            m_Colors32.Capacity = numPoints * 4;
        }

        public void SetMaterial(Material material)
        {
            if (material == null)
                material = Drawing3dManager.instance.UnlitPointCloudMaterial;

            m_MeshRenderer.sharedMaterial = material;
        }

        public void AddPoint(Vector3 point, Color32 color, float radius)
        {
            int start = m_Vertices.Count;

            for (int Idx = 0; Idx < 4; ++Idx)
            {
                m_Vertices.Add(point);
                m_Colors32.Add(color);
            }

            m_UVRs.Add(new Vector3(0, 0, radius));
            m_UVRs.Add(new Vector3(0, 1, radius));
            m_UVRs.Add(new Vector3(1, 0, radius));
            m_UVRs.Add(new Vector3(1, 1, radius));

            m_Triangles.Add(start + 0);
            m_Triangles.Add(start + 1);
            m_Triangles.Add(start + 2);
            m_Triangles.Add(start + 3);
            m_Triangles.Add(start + 2);
            m_Triangles.Add(start + 1);
            SetDirty();
        }

        public void AddTriangle(Vector3 point, Color32 color, float radius)
        {
            int start = m_Vertices.Count;

            for (int Idx = 0; Idx < 3; ++Idx)
            {
                m_Vertices.Add(point);
                m_Colors32.Add(color);
            }

            m_UVRs.Add(new Vector3(0.5f - k_SqrtPointFive, 0.5f, radius));
            m_UVRs.Add(new Vector3(1, 1.5f + k_SqrtPointFive, radius));
            m_UVRs.Add(new Vector3(1, -0.5f - k_SqrtPointFive, radius));

            m_Triangles.Add(start + 0);
            m_Triangles.Add(start + 1);
            m_Triangles.Add(start + 2);
            SetDirty();
        }

        void ClearBuffers()
        {
            m_Vertices.Clear();
            m_Colors32.Clear();
            m_UVRs.Clear();
            m_Triangles.Clear();
        }

        public void Clear()
        {
            ClearBuffers();
            SetDirty();
        }

        // Bake all buffered data into a mesh. Clear the buffers.
        // In most cases you don't need to call this - the mesh will be generated for you in the Update function.
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
            m_Mesh.SetUVs(0, m_UVRs.ToArray());
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
