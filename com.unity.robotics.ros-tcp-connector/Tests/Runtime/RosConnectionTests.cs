using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.TestTools;

namespace UnitTests
{
    public class RosConnectionTests
    {
        [Test]
        public void GetOrCreateInstance_CallOnce_ReturnsValidInstance()
        {
            RosEndpointConnection ros = RosEndpointConnection.GetOrCreateInstance();
            ros.ConnectOnStart = false;
            Assert.NotNull(ros);
        }

        [Test]
        public void GetOrCreateInstance_CallTwice_ReturnsSameInstance()
        {
            RosEndpointConnection ros = RosEndpointConnection.GetOrCreateInstance();
            Assert.NotNull(ros);
            ros.ConnectOnStart = false;
            RosEndpointConnection ros2 = RosEndpointConnection.GetOrCreateInstance();
            Assert.AreEqual(ros, ros2);
        }
    }
}
