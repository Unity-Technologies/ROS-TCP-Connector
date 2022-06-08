using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using RosMessageTypes.Diagnostic;
using Stopwatch = System.Diagnostics.Stopwatch;
using Newtonsoft.Json.Linq;

public class ConnectionTest : MonoBehaviour
{
    IPublisher publisher;

    // Start is called before the first frame update
    void Start()
    {
        IConnection connection = GetComponent<IConnection>();
        //connection.Connect();
        //publisher = connection.RegisterPublisher<CameraInfoMsg>("/strtest");
        connection.Subscribe<DiagnosticArrayMsg>("/strtest", OnMessage);
    }

    void OnMessage(DiagnosticArrayMsg msg)
    {
        Debug.Log("Received " + msg.ToString());
    }

    /*    int index;
        void Update()
        {
            if (index == 30)
            {
                index++;

                var ser = new JsonSerializer(false);
                var deser = new JsonDeserializer(false);
                //Debug.Log("Round trip: "+deser.DeserializeMessage<HeaderMsg>(ser.ToJsonString(new HeaderMsg(new TimeMsg(123, 456), "hello"))));

                var msg = new DiagnosticArrayMsg(new HeaderMsg(), new DiagnosticStatusMsg[] {
                    new DiagnosticStatusMsg(0, "hello", "message", "hardware", new KeyValueMsg[]{ new KeyValueMsg("key", "value") })
                });
                /*var serData = ser.ToJsonString(new CameraInfoMsg(new HeaderMsg(new TimeMsg(123, 456), "map"), 100U, 101U, "distortion",
                    new double[] { 1, 2, 3, 4 },
                    new double[] { 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9 },
                    new double[] { 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9 },
                    new double[] { 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 1.10, 1.11, 1.12 },
                    102U, 103U, new RegionOfInterestMsg(104U, 105U, 106U, 107U, true)));* /
                var serData = ser.ToJsonString(msg);

                Stopwatch stopwatch = new Stopwatch();
                Message outMsg;
                JObject jsonData;
                jsonData = JObject.Parse(serData);
                outMsg = deser.DeserializeMessage<DiagnosticArrayMsg>(serData);
                /*            stopwatch.Start();
                            for (int N = 0; N < 1000; N++)
                            {
                                outMsg = deser.DeserializeMessage<DiagnosticArrayMsg>(serData);
                            }
                            stopwatch.Stop();
                            Debug.Log("Read in "+stopwatch.ElapsedMilliseconds+"ms");* /
            }
            else
            {
                index++;
            }
        }*/
}
