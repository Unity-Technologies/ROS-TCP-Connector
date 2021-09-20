using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class VisualizationEditorWindow : EditorWindow
    {
        IVisualFactory m_VisualFactory;
        [SerializeField]
        string m_Topic;
        [SerializeField]
        string m_RosMessageName;
        [SerializeField]
        string m_FactoryID;
        IVisual m_Visual;
        Vector2 m_ScrollPos;
        public VisualizationEditorWindow(IVisualFactory factory, string topic, string rosMessageName)
        {
            m_VisualFactory = factory;
            m_Topic = topic;
            m_RosMessageName = rosMessageName;
            m_VisualFactory = factory;
            m_FactoryID = factory.ID;
            EditorApplication.playModeStateChanged += OnPlayModeState;
            titleContent = new GUIContent(m_Topic);
        }

        void OnPlayModeState(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredEditMode)
            {
                m_VisualFactory = null;
                m_Visual = null;
            }
        }

        private void OnGUI()
        {
            if (!EditorApplication.isPlaying)
            {
                GUILayout.Label("Waiting for play mode...");
                return;
            }

            if (m_Visual == null && EditorApplication.isPlaying)
            {
                if (m_FactoryID != null && m_VisualFactory == null)
                {
                    foreach (IVisualFactory factory in VisualFactoryRegistry.GetAllVisualFactories(m_Topic, m_RosMessageName))
                    {
                        if (factory.ID == m_FactoryID)
                        {
                            m_VisualFactory = factory;
                        }
                    }
                    if (m_VisualFactory == null)
                        m_FactoryID = null;
                }

                if (m_VisualFactory != null)
                    m_Visual = m_VisualFactory.GetOrCreateVisual(m_Topic);
            }

            if (m_Visual != null)
            {
                GUILayout.BeginHorizontal(GUI.skin.box);
                EditorGUI.BeginDisabledGroup(m_VisualFactory == null || !m_VisualFactory.CanShowDrawing);
                bool drawingEnable = EditorGUILayout.Toggle("Show 3d drawings", m_Visual.IsDrawingEnabled);
                EditorGUI.EndDisabledGroup();
                if (GUILayout.Button("Select in Editor"))
                {
                    Object factoryObject = (Object)m_VisualFactory;
                    Selection.activeObject = factoryObject;
                }
                GUILayout.EndHorizontal();
                if (m_Visual.IsDrawingEnabled != drawingEnable)
                    m_Visual.SetDrawingEnabled(drawingEnable);

                m_ScrollPos = GUILayout.BeginScrollView(m_ScrollPos);
                m_Visual.OnGUI();
                GUILayout.EndScrollView();
            }
        }

        [InitializeOnLoadMethod]
        static void Initialize()
        {
            VisualizationTopicsTabEntry.s_OpenVisualizationWindow = CreateWindow;
        }

        static void CreateWindow(IVisualFactory visualizer, string topic, string rosMessageName)
        {
            VisualizationEditorWindow window = new VisualizationEditorWindow(visualizer, topic, rosMessageName);
            window.Show();
        }
    }
}
