using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector.TransformManagement;
using UnityEngine;

[TestFixture]
class TransformManagementTests
{
    static readonly Type k_InvalidOperation = typeof(InvalidOperationException);
    const string k_EmptyParentName = "EmptyParent";
    const string k_OtherParentName = "OtherParent";
    // Convenience properties for generating commonly used objects.
    static TransformStream EmptyParent => new TransformStream(null, k_EmptyParentName);
    static TransformStream OtherParent => new TransformStream(EmptyParent, k_OtherParentName);

    public static IEnumerable<TestCaseData> ParentHandlingTestCases()
    {
        // (TransformStream stream, Vector3 position, Quaternion rotation, TransformStream parent,
        // Type expectedException, TransformStream expectedParent)
        // Refuse to add a frame to stream without setting a parent first
        yield return new TestCaseData(new TransformStream(null, "Child"),
            null, k_InvalidOperation, null);
        // Accept frame when things are set up correctly
        // Create a new parent object here so we can pass the reference into both arguments
        var emptyParent = EmptyParent;
        yield return new TestCaseData(new TransformStream(emptyParent, "Child"),
            emptyParent, null, emptyParent);
        // Refuse to add a frame from a different parent than expected
        // Reset emptyParent object to ensure no state carries over from previous test
        emptyParent = EmptyParent;
        yield return new TestCaseData(new TransformStream(emptyParent, "Child"),
            OtherParent, k_InvalidOperation, emptyParent);
        // Parentless stream can accept a frame with an accompanying parent, and becomes adopted
        emptyParent = EmptyParent;
        yield return new TestCaseData(new TransformStream(null, "Child"),
            emptyParent, null, emptyParent);
        var otherParent = OtherParent;
        yield return new TestCaseData(new TransformStream(null, "Child"),
            otherParent, null, otherParent);
    }

    [Test, TestCaseSource(nameof(ParentHandlingTestCases))]
    public void TransformStream_AddFrame_ManagesParentCorrectly(
            TransformStream stream, TransformStream parent, Type expectedException, TransformStream expectedParent)
    {
        try
        {
            // null parent, adding a frame with no parent, should fail
            stream.AddFrame(0, Vector3.zero, Quaternion.identity, parent);
        }
        catch (Exception e)
        {
            if (expectedException == null)
            {
                throw;
            }
            Assert.IsInstanceOf(expectedException, e);
        }

        Assert.AreSame(expectedParent, stream.Parent);
    }

    //public static IEnumerable<TestCaseData> FrameSortingTestCases()
    //{
    //    yield return null;
    //}

    //[Test, TestCaseSource(nameof(FrameSortingTestCases))]
    //void TransformStream_AddFrame_SortsCorrectly(
    //    TransformStream stream, double timestamp, Vector3 translation, Quaternion rotation)
    //{

    //}

}
