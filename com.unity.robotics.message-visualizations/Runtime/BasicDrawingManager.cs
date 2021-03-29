using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class BasicDrawingManager : MonoBehaviour
    {
        static BasicDrawingManager s_Instance;
        public static BasicDrawingManager instance
        {
            get
            {
                if (s_Instance == null)
                {
                    GameObject newDebugDrawObj = new GameObject("DrawingManager");
                    s_Instance = newDebugDrawObj.AddComponent<BasicDrawingManager>();
                    s_Instance.Material = new Material(Shader.Find("Unlit/VertexColor"));
                    s_Instance.UnlitColorMaterial = new Material(Shader.Find("Unlit/Color"));
                    s_Instance.UnlitColorAlphaMaterial = new Material(Shader.Find("Unlit/ColorAlpha"));
                }
                return s_Instance;
            }
        }

        Camera m_Camera;
        List<BasicDrawing> m_Drawings = new List<BasicDrawing>();
        List<BasicDrawing> m_Dirty = new List<BasicDrawing>();
        public Material Material { get; private set; }
        public Material UnlitColorMaterial { get; private set; }
        public Material UnlitColorAlphaMaterial { get; private set; }

        void Awake()
        {
            s_Instance = this;
            m_Camera = Camera.main;
        }

        public void AddDirty(BasicDrawing drawing)
        {
            m_Dirty.Add(drawing);
        }

        public void DestroyDrawing(BasicDrawing drawing)
        {
            m_Drawings.Remove(drawing);
            GameObject.Destroy(drawing.gameObject);
        }

        public static BasicDrawing CreateDrawing(float duration = -1, Material material = null)
        {
            GameObject newDrawingObj = new GameObject("Drawing");
            BasicDrawing newDrawing = newDrawingObj.AddComponent<BasicDrawing>();
            newDrawing.Init(instance, material != null ? material : instance.Material, duration);
            instance.m_Drawings.Add(newDrawing);
            return newDrawing;
        }

        
        void LateUpdate()
        {
            foreach (BasicDrawing drawing in m_Dirty)
            {
                if(drawing != null)
                    drawing.Refresh();
            }
            m_Dirty.Clear();
        }

        public void OnGUI()
        {
            Color oldColor = GUI.color;

            int Idx = 0; 
            while(Idx < m_Drawings.Count)
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