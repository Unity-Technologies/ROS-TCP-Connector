using RosMessageTypes.Std;
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
        //publisher = connection.RegisterPublisher<StringMsg>("/strtest");
        connection.Subscribe<StringMsg>("/strtest", OnMessage);
    }

    void OnMessage(StringMsg msg)
    {
        Debug.Log("Received " + msg.data);
    }

    int index;
    void Update()
    {
        //publisher.Publish(new StringMsg("Hello " + index));
        index++;
    }
}
