using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using MTransform = RosMessageTypes.Geometry.Transform;
using Unity.Robotics.MessageVisualizers;

public class PrefabTransformVisualizer : MonoBehaviour, IVisualizer
{
    public string topic;
    public GameObject markerPrefab;

    public void Start()
    {
        // For a Visualizer to actually be known to the HUD,
        // it must call VisualizationRegister.RegisterVisualizer.
        // There are two versions: either register for messages on a specific topic,
        // or register for all messages of a specific type.
        // In this example, we choose between those two options based on whether
        // a topic has been specified.
        if (topic != "")
            VisualizationRegister.RegisterVisualizer(topic, this);
        else
            VisualizationRegister.RegisterVisualizer<MTransform>(this);
    }

    public object CreateDrawing(Message message, MessageMetadata meta)
    {
        // Note we have to cast from the generic Message class here.
        // BasicVisualizer would have done this for us.
        MTransform transformMessage = (MTransform)message;
        // instantiate a prefab and put it at the appropriate position and rotation.
        GameObject marker = Instantiate(markerPrefab);
        marker.transform.position = transformMessage.translation.From<FLU>();
        marker.transform.rotation = transformMessage.rotation.From<FLU>();
        return marker;
    }

    public void DeleteDrawing(object drawing)
    {
        // destroy the instance when we're done
        GameObject.Destroy((GameObject)drawing);
    }

    public System.Action CreateGUI(Message message, MessageMetadata meta, object drawing)
    {
        return () =>
        {
            // call the standard GUI function for displaying Transform messages
            ((MTransform)message).GUI(meta.topic);
        };
    }
}
