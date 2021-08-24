using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;


namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BaseVisualFactory<T> : MonoBehaviour, IVisualFactory, IPriority
        where T : Message
    {
        [SerializeField]
        protected string m_Topic;
        public string Topic { get => m_Topic; set => m_Topic = value; }

        public int Priority { get; set; }
        public abstract bool CanShowDrawing { get; }

        Dictionary<string, IVisual> m_Visuals = new Dictionary<string, IVisual>();
        public IEnumerable<IVisual> AllVisuals => m_Visuals.Values;

        public IVisual GetOrCreateVisual(string topic)
        {
            IVisual visual;
            if (m_Visuals.TryGetValue(topic, out visual))
                return visual;

            visual = CreateVisual();
            m_Visuals.Add(topic, visual);
            return visual;
        }

        protected abstract IVisual CreateVisual();

        public virtual void Start()
        {
            if (m_Topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<T>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
            }
        }
    }
}
