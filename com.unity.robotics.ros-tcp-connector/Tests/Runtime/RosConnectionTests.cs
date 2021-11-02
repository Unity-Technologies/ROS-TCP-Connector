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

namespace Tests.Runtime
{
    public class RosConnectionTests
    {
        [Test]
        public void RosConnectionCanCreateInstance()
        {
            ROSConnection ros = ROSConnection.GetOrCreateInstance();
            Assert.NotNull(ros);
            ros.ConnectOnStart = false;
            ROSConnection ros2 = ROSConnection.GetOrCreateInstance();
            Assert.AreEqual(ros, ros2);
        }

        [Test]
        public void RosConnectionCanCreate()
        {
            ROSConnection ros = ROSConnection.GetOrCreateInstance();
            ros.ConnectOnStart = false;
        }
    }
}
