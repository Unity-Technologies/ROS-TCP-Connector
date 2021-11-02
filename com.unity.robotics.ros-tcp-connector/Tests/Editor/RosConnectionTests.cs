using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.TestTools;
using UnityEngine;
using UnityEngine.TestTools;
using Moq;

namespace BuildTests
{
    public class RosConnectionTests
    {
        [Test]
        public void RosConnectionMockTest()
        {
            var mock = new Mock<ROSConnection>();
        }
    }
}
