using System.Collections.Generic;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace UnitTests
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

            Object.DestroyImmediate(parent);
        }

        [Test, TestCaseSource(nameof(EnuTestCases))]
        public void CompassENU_Success(CardinalDirection direction,
            Vector3 unityPosition, Vector3<ENU> enuPosition,
            Quaternion unityRotation, Quaternion<ENU> enuRotation)
        {
            GeometryCompass.UnityZAxisDirection = direction;
            Assert.AreEqual(enuPosition, unityPosition.To<ENU>());
            Assert.AreEqual(unityPosition, enuPosition.toUnity);
            AssertApproximatelyEqual(enuRotation, unityRotation.To<ENU>());
            AssertApproximatelyEqual(unityRotation, enuRotation.toUnity);
        }

        [Test, TestCaseSource(nameof(NedTestCases))]
        public void CompassNED_Success(CardinalDirection direction,
            Vector3 unityPosition, Vector3<NED> nedPosition,
            Quaternion unityRotation, Quaternion<NED> nedRotation)
        {
            GeometryCompass.UnityZAxisDirection = direction;
            Assert.AreEqual(nedPosition, unityPosition.To<NED>());
            Assert.AreEqual(unityPosition, nedPosition.toUnity);
            AssertApproximatelyEqual(nedRotation, unityRotation.To<NED>());
            AssertApproximatelyEqual(unityRotation, nedRotation.toUnity);
        }

        static IEnumerable<TestCaseData> EnuTestCases
        {
            get
            {
                yield return new TestCaseData(CardinalDirection.North,
                    new Vector3(1, 2, 3), new Vector3<ENU>(1, 3, 2),
                    Quaternion.identity, new Quaternion<ENU>(0, 0, -Mathf.Sqrt(2) / 2, -Mathf.Sqrt(2) / 2));
                yield return new TestCaseData(CardinalDirection.East,
                    new Vector3(1, 2, 3), new Vector3<ENU>(3, -1, 2),
                    Quaternion.identity, new Quaternion<ENU>(0, 0, 0, -1));
                yield return new TestCaseData(CardinalDirection.South,
                    new Vector3(1, 2, 3), new Vector3<ENU>(-1, -3, 2),
                    Quaternion.identity, new Quaternion<ENU>(0, 0, Mathf.Sqrt(2) / 2, -Mathf.Sqrt(2) / 2));
                yield return new TestCaseData(CardinalDirection.West,
                    new Vector3(1, 2, 3), new Vector3<ENU>(-3, 1, 2),
                    Quaternion.identity, new Quaternion<ENU>(0, 0, 1, 0));
            }
        }

        static IEnumerable<TestCaseData> NedTestCases
        {
            get
            {
                yield return new TestCaseData(CardinalDirection.North,
                    new Vector3(1, 2, 3), new Vector3<NED>(3, 1, -2),
                    Quaternion.identity, new Quaternion<NED>(0, 0, 0, -1));
                yield return new TestCaseData(CardinalDirection.East,
                    new Vector3(1, 2, 3), new Vector3<NED>(-1, 3, -2),
                    Quaternion.identity, new Quaternion<NED>(0, 0, -Mathf.Sqrt(2) / 2, -Mathf.Sqrt(2) / 2));
                yield return new TestCaseData(CardinalDirection.South,
                    new Vector3(1, 2, 3), new Vector3<NED>(-3, -1, -2),
                    Quaternion.identity, new Quaternion<NED>(0, 0, -1, 0));
                yield return new TestCaseData(CardinalDirection.West,
                    new Vector3(1, 2, 3), new Vector3<NED>(1, -3, -2),
                    Quaternion.identity, new Quaternion<NED>(0, 0, -Mathf.Sqrt(2) / 2, Mathf.Sqrt(2) / 2));
            }
        }

        void AssertApproximatelyEqual<C>(Quaternion<C> expectedRotation, Quaternion<C> rotation) where C : ICoordinateSpace, new()
        {
            AssertApproximatelyEqual(
                new Quaternion(expectedRotation.x, expectedRotation.y, expectedRotation.z, expectedRotation.w),
                new Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
        }

        void AssertApproximatelyEqual(Quaternion expectedRotation, Quaternion rotation)
        {
            Assert.AreApproximatelyEqual(expectedRotation.eulerAngles.x, rotation.eulerAngles.x);
            Assert.AreApproximatelyEqual(expectedRotation.eulerAngles.y, rotation.eulerAngles.y);
            Assert.AreApproximatelyEqual(expectedRotation.eulerAngles.z, rotation.eulerAngles.z);
        }
    }
}
