using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Unity.Robotics.Visualizations
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
            SetFactory(factory);
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

        public void SetFactory(IVisualFactory factory)
        {
            m_VisualFactory = factory;
            m_FactoryID = factory.ID;
            m_Visual = null;
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
                if (m_FactoryID == null)
                {
                    GUIContent buttonContent = new GUIContent("Select Visualizer");
                    Rect selectBtnRect = GUILayoutUtility.GetRect(buttonContent, EditorStyles.toolbarDropDown, GUILayout.ExpandWidth(false));
                    if (EditorGUI.DropdownButton(selectBtnRect, buttonContent, FocusType.Keyboard))
                    {
                        GenericMenu menu = new GenericMenu();
                        foreach (IVisualFactory factory in VisualFactoryRegistry.GetAllVisualFactories(m_Topic, m_RosMessageName))
                        {
                            menu.AddItem(new GUIContent(factory.Name), false, () => SetFactory(factory));
                        }
                        menu.DropDown(selectBtnRect);
                    }
                }
                else if (m_VisualFactory == null)
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
                bool drawingEnable = EditorGUILayout.ToggleLeft("Show 3d drawings", m_Visual.IsDrawingEnabled);
                EditorGUI.EndDisabledGroup();
                EditorGUI.BeginDisabledGroup(!(m_VisualFactory is Object));
                if (GUILayout.Button("Select in Editor"))
                {
                    Object factoryObject = (Object)m_VisualFactory;
                    Selection.activeObject = factoryObject;
                }
                EditorGUI.EndDisabledGroup();
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
            VisualizationTopicsTabEntry.SetOpenWindowCallback(OpenWindow);
        }

        static void OpenWindow(IVisualFactory visualizer, string topic, string rosMessageName)
        {
            VisualizationEditorWindow window = new VisualizationEditorWindow(visualizer, topic, rosMessageName);
            window.Show();
        }
    }
}
