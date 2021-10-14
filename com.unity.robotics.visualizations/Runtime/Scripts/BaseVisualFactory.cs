using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace Unity.Robotics.Visualizations
{
    public abstract class BaseVisualFactory<T> : MonoBehaviour, IVisualFactory, IPriority
        where T : Message
    {
        [SerializeField]
        protected string m_Topic;
        public string Topic { get => m_Topic; set => m_Topic = value; }

        [SerializeField]
        [HideInInspector]
        string m_ID;
        public string ID => m_ID;

#if UNITY_EDITOR
        // this could be any string that's not blank or null; it just needs to not test equal to any "real" deserialized ID
        string m_LastSetID = "~~invalid";

        void OnValidate()
        {
            if (m_LastSetID != m_ID)
            {
                // Get a locally unique, scene non-specific, persistent id for this component
                string fullID = UnityEditor.GlobalObjectId.GetGlobalObjectIdSlow(this).ToString();
                string[] idParts = fullID.Split('-');
                if (idParts.Length < 5) // not sure how this would happen, but just in case
                    m_ID = fullID;
                else
                    m_ID = idParts[3]+"-"+idParts[4];
                m_LastSetID = m_ID;
            }
        }
#endif
        public virtual string Name => (string.IsNullOrEmpty(m_Topic) ? "" : $"({m_Topic}) ") + GetType().ToString().Split('.').Last();

        public int Priority { get; set; }
        public abstract bool CanShowDrawing { get; }

        Dictionary<string, IVisual> m_Visuals = new Dictionary<string, IVisual>();
        public IEnumerable<IVisual> AllVisuals => m_Visuals.Values;

        public virtual IVisual GetOrCreateVisual(string topic)
        {
            IVisual visual;
            if (m_Visuals.TryGetValue(topic, out visual))
                return visual;

            visual = CreateVisual(topic);
            m_Visuals.Add(topic, visual);
            return visual;
        }

        protected abstract IVisual CreateVisual(string topic);

        public virtual void Start()
        {
            if (m_Topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<T>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_Topic, this, Priority, MessageRegistry.GetSubtopic<T>());
            }
        }

        public static void ListenForMessages(string topic, System.Action<Message> callback)
        {
            RosTopicState state = ROSConnection.GetOrCreateInstance().GetTopic(topic);
            MessageSubtopic subtopic = MessageRegistry.GetSubtopic<T>();
            if (subtopic == MessageSubtopic.Response && state != null)
            {
                if (state.ServiceResponseTopic == null)
                    Debug.Log("Failed to subscribe to " + topic + " response!");
                else
                    state.ServiceResponseTopic.AddSubscriber(callback);
            }
            else
                state.AddSubscriber(callback);
        }
    }
}
