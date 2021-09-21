using System;
using UnityEditor;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    [CustomEditor(typeof(InteractionManager))]
    public class InteractionManagerEditor : Editor
    {
        bool m_HasInteraction;
        bool m_HasTrigger;
        InteractionType m_InteractionType;
        TriggerType m_TriggerType;

        public override void OnInspectorGUI()
        {
            var interactManager = target as InteractionManager;
            if (interactManager != null)
            {
                m_HasTrigger = interactManager.GetComponent<VisualizationTrigger>() != null;
                if (!m_HasTrigger)
                {
                    GUILayout.BeginHorizontal();
                    m_TriggerType = (TriggerType)EditorGUILayout.EnumPopup("Trigger Type", m_TriggerType);
                    if (GUILayout.Button("Add Viz Trigger"))
                    {
                        switch (m_TriggerType)
                        {
                            case TriggerType.HudButton:
                                ObjectFactory.AddComponent<HudButton>(interactManager.gameObject);
                                break;
                        }
                    }
                    GUILayout.EndHorizontal();
                }

                m_HasInteraction = interactManager.GetComponent<Interaction>() != null;
                if (!m_HasInteraction)
                {
                    GUILayout.BeginHorizontal();
                    m_InteractionType = (InteractionType)EditorGUILayout.EnumPopup("Interaction Type", m_InteractionType);
                    if (GUILayout.Button("Add Interaction"))
                        switch (m_InteractionType)
                        {
                            case InteractionType.RaycastPoint:
                                ObjectFactory.AddComponent<RaycastPointInteraction>(interactManager.gameObject);
                                break;
                            case InteractionType.RaycastPose:
                                ObjectFactory.AddComponent<RaycastPoseInteraction>(interactManager.gameObject);
                                break;
                            case InteractionType.Field:
                                ObjectFactory.AddComponent<FieldInteraction>(interactManager.gameObject);
                                break;
                        }
                    GUILayout.EndHorizontal();
                }
            }
        }

        enum TriggerType
        {
            HudButton
        }

        enum InteractionType
        {
            RaycastPoint, RaycastPose, Field
        }
    }
}
