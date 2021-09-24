using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class Drawing3dManager : MonoBehaviour
    {
        static Drawing3dManager s_Instance;
        public static Drawing3dManager instance
        {
            get
            {
#if UNITY_EDITOR
                if (s_Instance == null)
                {
                    s_Instance = FindObjectOfType<Drawing3dManager>();
                    if (s_Instance == null)
                    {
                        GameObject newDebugDrawObj = new GameObject("DrawingManager");
                        s_Instance = newDebugDrawObj.AddComponent<Drawing3dManager>();
                        s_Instance.m_UnlitVertexColorMaterial = new Material(Shader.Find("Unlit/VertexColor"));
                        s_Instance.m_UnlitColorMaterial = new Material(Shader.Find("Unlit/Color"));
                        s_Instance.m_UnlitColorAlphaMaterial = new Material(Shader.Find("Unlit/ColorAlpha"));
                        s_Instance.m_UnlitPointCloudMaterial = new Material(Shader.Find("Unlit/PointCloudCutout"));
                    }
                }
#endif
                return s_Instance;
            }
        }

        Camera m_Camera;
        List<Drawing3d> m_Drawings = new List<Drawing3d>();
        List<Drawing3d> m_Dirty = new List<Drawing3d>();

        [SerializeField]
        Material m_UnlitVertexColorMaterial;
        [SerializeField]
        Material m_UnlitColorMaterial;
        [SerializeField]
        Material m_UnlitColorAlphaMaterial;
        [SerializeField]
        Material m_UnlitPointCloudMaterial;

        public Material UnlitVertexColorMaterial => m_UnlitVertexColorMaterial;
        public Material UnlitColorMaterial => m_UnlitColorMaterial;
        public Material UnlitColorAlphaMaterial => m_UnlitColorAlphaMaterial;
        public Material UnlitPointCloudMaterial => m_UnlitPointCloudMaterial;

        void Awake()
        {
            s_Instance = this;
            m_Camera = Camera.main;
        }

        public void AddDirty(Drawing3d drawing)
        {
            m_Dirty.Add(drawing);
        }

        public void DestroyDrawing(Drawing3d drawing)
        {
            m_Drawings.Remove(drawing);
            GameObject.Destroy(drawing.gameObject);
        }

        public static Drawing3d CreateDrawing(float duration = -1, Material material = null)
        {
            GameObject newDrawingObj = new GameObject("Drawing");
            Drawing3d newDrawing = newDrawingObj.AddComponent<Drawing3d>();
            newDrawing.Init(instance, material != null ? material : instance.UnlitVertexColorMaterial, duration);
            instance.m_Drawings.Add(newDrawing);
            return newDrawing;
        }


        void LateUpdate()
        {
            foreach (Drawing3d drawing in m_Dirty)
            {
                if (drawing != null)
                    drawing.Refresh();
            }
            m_Dirty.Clear();
        }

        public void OnGUI()
        {
            Color oldColor = GUI.color;

            int Idx = 0;
            while (Idx < m_Drawings.Count)
            {
                if (m_Drawings[Idx] == null)
                    m_Drawings.RemoveAt(Idx);
                else
                {
                    m_Drawings[Idx].OnDrawingGUI(m_Camera);
                    ++Idx;
                }
            }

            GUI.color = oldColor;
        }
    }
}
