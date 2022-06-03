using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class ConnectionTest : MonoBehaviour
{
    IPublisher publisher;

    // Start is called before the first frame update
    void Start()
    {
        IConnection connection = GetComponent<IConnection>();
        connection.Connect();
        publisher = connection.RegisterPublisher<CameraInfoMsg>("/strtest");
        connection.Subscribe<CameraInfoMsg>("/strtest", OnMessage);
    }

    void OnMessage(CameraInfoMsg msg)
    {
        Debug.Log("Received " + msg.ToString());
    }

    int index;
    void Update()
    {
        if (index == 0)
        {
            publisher.Publish(new CameraInfoMsg(new HeaderMsg(new TimeMsg(123, 456), "map"), 100U, 101U, "distortion",
                new double[] { 1, 2, 3, 4 },
                new double[] { 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9 },
                new double[] { 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9 },
                new double[] { 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 1.10, 1.11, 1.12 },
                102U, 103U, new RegionOfInterestMsg(104U, 105U, 106U, 107U, true)));
        }
        index++;
    }
}
