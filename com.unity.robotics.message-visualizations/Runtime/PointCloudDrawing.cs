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
        bool m_isDirty = false;

        public static PointCloudDrawing Create(float duration = -1, Material material = null)
        {
            return null;// BasicDrawingManager.CreateDrawing(duration, material);
        }

        public Material material;

        public void Awake()
        {
            Init(null, material);
            float area = 10.0f;
            for (int Idx = 0; Idx < 10000000; ++Idx)
            {
                AddPoint(
                    new Vector3(Random.Range(-area, area), Random.Range(-area, area), Random.Range(-area, area)),
                    new Color32((byte)Random.Range(0, 255), (byte)Random.Range(0, 255), (byte)Random.Range(0, 255), 255)
                );
            }
        }

        public void Init(BasicDrawingManager parent, Material material, float duration = -1)
        {
            m_Mesh = new Mesh();

            //transform.parent = parent.transform;

            MeshFilter mfilter = gameObject.AddComponent<MeshFilter>();
            mfilter.sharedMesh = m_Mesh;
            MeshRenderer mrenderer = gameObject.AddComponent<MeshRenderer>();
            mrenderer.sharedMaterial = material;

            if (duration >= 0)
            {
                StartCoroutine(DestroyAfterDelay(duration));
            }
        }

        public void AddPoint(Vector3 point, Color32 color)
        {
            int start = m_Vertices.Count;
            m_UVs.Add(Vector3.zero);
            m_UVs.Add(Vector3.up);
            m_UVs.Add(Vector3.one);
            m_UVs.Add(Vector3.right);

            for (int Idx = 0; Idx < 4; ++Idx)
            {
                m_Vertices.Add(point);
                m_Colors32.Add(color);
            }

            m_Triangles.Add(start);
            m_Triangles.Add(start + 1);
            m_Triangles.Add(start + 2);
            m_Triangles.Add(start + 2);
            m_Triangles.Add(start + 3);
            m_Triangles.Add(start);
            SetDirty();
        }

        public IEnumerator DestroyAfterDelay(float duration)
        {
            yield return new WaitForSeconds(duration);
            Destroy();
        }

        public void Clear()
        {
            m_Vertices.Clear();
            m_Colors32.Clear();
            m_Triangles.Clear();
            SetDirty();
        }

        public void Destroy()
        {
            GameObject.Destroy(gameObject);
        }

        public void Update()
        {
            if (!m_isDirty)
                return;

            m_isDirty = false;
            m_Mesh.Clear();
            m_Mesh.indexFormat = m_Vertices.Count < 65536 ? UnityEngine.Rendering.IndexFormat.UInt16 : UnityEngine.Rendering.IndexFormat.UInt32;
            m_Mesh.vertices = m_Vertices.ToArray();
            m_Mesh.uv = m_UVs.ToArray();
            m_Mesh.colors32 = m_Colors32.ToArray();
            m_Mesh.triangles = m_Triangles.ToArray();
        }

        void SetDirty()
        {
            if (!m_isDirty)
            {
                m_isDirty = true;
                //BasicDrawingManager.instance.AddDirty(this);
            }
        }
    }
}