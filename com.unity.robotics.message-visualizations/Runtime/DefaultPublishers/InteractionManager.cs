using System;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    [RequireComponent(typeof(RosPublish))]
    public class InteractionManager : MonoBehaviour
    {
        VisualizationTrigger m_HudButton;
        Interaction[] m_Interactions;

        // Start is called before the first frame update
        void Start()
        {
            m_HudButton = GetComponent<VisualizationTrigger>();
            if (m_HudButton == null)
            {
                Debug.LogError("Requires component of type VisualizationTrigger!");
            }
            m_Interactions = GetComponents<Interaction>();
            if (m_Interactions.Length == 0)
            {
                Debug.LogError("Requires component of type Interaction!");
            }

            foreach (var interaction in m_Interactions)
            {
                m_HudButton.Subscribe(interaction);
            }
        }
    }
}
