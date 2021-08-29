using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Tests.Runtime
{
    public class RosGeometryTests
    {
        [Test]
        public void TransfromtoLocal_ChildObjectTransform_LocalTransform()
        {
            GameObject parent = new GameObject();
            parent.transform.position = new Vector3(1, 1, 1);
            GameObject child = new GameObject();
            child.transform.parent = parent.transform;
            child.transform.localPosition = new Vector3(.2f, .3f, .5f);
            child.transform.localRotation = new Quaternion(.3f, .4f, .5f, 1);

            var testMsg = child.transform.ToLocal<FLU>();
            var fluPosition = child.transform.localPosition.To<FLU>();
            var fluRotation = child.transform.localRotation.To<FLU>();
            Assert.IsNotNull(testMsg);
            Assert.AreApproximatelyEqual(fluPosition.x, (float)testMsg.translation.x);
            Assert.AreApproximatelyEqual(fluPosition.y, (float)testMsg.translation.y);
            Assert.AreApproximatelyEqual(fluPosition.z, (float)testMsg.translation.z);
            Assert.AreApproximatelyEqual(fluRotation.x, (float)testMsg.rotation.x);
            Assert.AreApproximatelyEqual(fluRotation.y, (float)testMsg.rotation.y);
            Assert.AreApproximatelyEqual(fluRotation.z, (float)testMsg.rotation.z);
            Assert.AreApproximatelyEqual(fluRotation.w, (float)testMsg.rotation.w);
        }
    }
}
