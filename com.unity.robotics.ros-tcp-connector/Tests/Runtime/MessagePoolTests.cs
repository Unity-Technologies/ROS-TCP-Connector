using System.Collections.Generic;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Std;

namespace UnitTests
{
    public class MessagePoolTests
    {
        [Test]
        public void AddAndGet_OneMessage_ReturnsSameMessage()
        {
            MessagePool<StringMsg> pool = new MessagePool<StringMsg>();
            const string firstMessageContents = "first message";
            StringMsg str1 = new StringMsg(firstMessageContents);
            pool.AddMessage(str1);
            StringMsg str2 = pool.GetOrCreateMessage();
            StringMsg str3 = pool.GetOrCreateMessage();
            Assert.AreEqual(str1, str2);
            Assert.AreNotEqual(str2, str3);
        }

        [Test]
        public void AddAndGet_MultipleMessages_ReturnCorrectMessage()
        {
            MessagePool<StringMsg> pool = new MessagePool<StringMsg>();
            const string firstMessageContents = "first message";
            StringMsg str1 = new StringMsg(firstMessageContents);
            StringMsg str2 = new StringMsg(firstMessageContents);
            pool.AddMessage(str1);
            pool.AddMessage(str2);
            StringMsg str3 = pool.GetOrCreateMessage();
            StringMsg str4 = pool.GetOrCreateMessage();
            StringMsg str5 = pool.GetOrCreateMessage();
            Assert.AreEqual(str1, str3);
            Assert.AreEqual(str2, str4);
            Assert.AreNotEqual(str1, str5);
            Assert.AreNotEqual(str2, str5);
        }
    }
}
