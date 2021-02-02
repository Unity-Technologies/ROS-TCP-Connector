using RosMessageGeneration;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSTransform = RosMessageTypes.Geometry.Transform;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class ObjectTransformVisualizer : MonoBehaviour
{
    public string[] visualizeTopics;
    public bool handleAllTransformMessages;
    public CoordinateSpaceSelection coordinateSpace = CoordinateSpaceSelection.FLU;

    public virtual void Awake()
    {
        foreach (string topic in visualizeTopics)
            MessageVisualizations.RegisterVisualizer(CreateVisualizer, topic);

        if (handleAllTransformMessages)
            MessageVisualizations.RegisterVisualizer(CreateVisualizer, typeof(ROSTransform));
    }

    IMessageVisualizer CreateVisualizer(string topic, Message message)
    {
        ROSTransform transformMessage = (ROSTransform)message;
        transform.position = transformMessage.translation.From(coordinateSpace);
        transform.rotation = transformMessage.rotation.From(coordinateSpace);
        return new TransformVisualizer { topic = topic, transform = transformMessage };
    }

    public class TransformVisualizer : IMessageVisualizer
    {
        public string topic;
        public ROSTransform transform;

        public void GUI()
        {
            MessageVisualizations.GUI(topic, transform);
        }

        public void End()
        {
        }
    }
}
