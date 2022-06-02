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
        publisher = connection.RegisterPublisher<TimeMsg>("/strtest");
        //connection.Subscribe<StringMsg>("/strtest", OnMessage);
    }

    void OnMessage(StringMsg msg)
    {
        Debug.Log("Received " + msg.data);
    }

    int index;
    void Update()
    {
        if (index == 0)
        {
            publisher.Publish(new CameraInfoMsg(new HeaderMsg(new TimeMsg(123, 456), "map"), 100U, 101U, "distortion",
                new double[] { 1, 2, 3, 4 }, new double[] { 5.1, 6, 7, 8.01 }, new double[] { 9, 10, 11, 12 }, new double[] { 13, 14, 15, 16 },
                102U, 103U, new RegionOfInterestMsg(104U, 105U, 106U, 107U, true)));
        }
        index++;
    }
}
