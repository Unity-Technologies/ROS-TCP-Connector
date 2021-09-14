using System.Collections.Generic;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Tests.Runtime
{
    public class RosGeometryTests
    {
        GeometryCompass m_Compass;

        [SetUp]
        public void SetUp()
        {
            m_Compass = new GameObject().AddComponent<GeometryCompass>();
        }

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
            m_Compass.UnityZAxisDirection = direction;
            Assert.AreEqual(enuPosition, m_Compass.ToENU(unityPosition));
            Assert.AreEqual(unityPosition, m_Compass.FromENU(enuPosition));
            AssertApproximatelyEqual(enuRotation, m_Compass.ToENU(unityRotation));
            AssertApproximatelyEqual(unityRotation, m_Compass.FromENU(enuRotation));
        }

        [Test, TestCaseSource(nameof(NedTestCases))]
        public void CompassNED_Success(CardinalDirection direction,
            Vector3 unityPosition, Vector3<NED> nedPosition,
            Quaternion unityRotation, Quaternion<NED> nedRotation)
        {
            m_Compass.UnityZAxisDirection = direction;
            Assert.AreEqual(nedPosition, m_Compass.ToNED(unityPosition));
            Assert.AreEqual(unityPosition, m_Compass.FromNED(nedPosition));
            AssertApproximatelyEqual(nedRotation, m_Compass.ToNED(unityRotation));
            AssertApproximatelyEqual(unityRotation, m_Compass.FromNED(nedRotation));
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


        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(m_Compass.gameObject);
        }

        void AssertApproximatelyEqual<C>(Quaternion<C> expectedRotation, Quaternion<C> rotation) where C : ICoordinateSpace, new()
        {
            Assert.AreApproximatelyEqual(expectedRotation.x, rotation.x);
            Assert.AreApproximatelyEqual(expectedRotation.y, rotation.y);
            Assert.AreApproximatelyEqual(expectedRotation.z, rotation.z);
            Assert.AreApproximatelyEqual(expectedRotation.w, rotation.w);
        }

        void AssertApproximatelyEqual(Quaternion expectedRotation, Quaternion rotation)
        {
            AssertApproximatelyEqual(
                new Quaternion<RUF>(expectedRotation.x, expectedRotation.y, expectedRotation.z, expectedRotation.w),
                new Quaternion<RUF>(rotation.x, rotation.y, rotation.z, rotation.w));
        }
    }
}
