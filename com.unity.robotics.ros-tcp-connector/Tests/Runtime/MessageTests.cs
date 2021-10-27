using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Std;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System;

namespace Tests.Runtime
{
    public class MessageTests
    {
        [Test]
        public void RoundTrip_Int32()
        {
            const int v = 1234;
            Int32Msg inMsg = new Int32Msg(v);
            Int32Msg outMsg = MessageRoundTrip(inMsg, Int32Msg.Deserialize);
            Assert.AreEqual(inMsg.data, v);
            Assert.AreEqual(inMsg.data, outMsg.data);
        }

        [Test]
        public void RoundTrip_MaxInt()
        {
            const int v = int.MaxValue;
            Int32Msg inMsg = new Int32Msg(v);
            Int32Msg outMsg = MessageRoundTrip(inMsg, Int32Msg.Deserialize);
            Assert.AreEqual(inMsg.data, v);
            Assert.AreEqual(inMsg.data, outMsg.data);
        }

        [Test]
        public void RoundTrip_MinInt()
        {
            const int v = int.MinValue;
            Int32Msg inMsg = new Int32Msg(v);
            Int32Msg outMsg = MessageRoundTrip(inMsg, Int32Msg.Deserialize);
            Assert.AreEqual(inMsg.data, v);
            Assert.AreEqual(inMsg.data, outMsg.data);
        }

        [Test]
        public void RoundTrip_String()
        {
            const string v = "hello";
            StringMsg inMsg = new StringMsg(v);
            StringMsg outMsg = MessageRoundTrip(inMsg, StringMsg.Deserialize);
            Assert.AreEqual(inMsg.data, v);
            Assert.AreEqual(inMsg.data, outMsg.data);
        }

        [Test]
        public void RoundTrip_UnicodeString()
        {
            const string v = "ಠ_ಠ";
            StringMsg inMsg = new StringMsg(v);
            StringMsg outMsg = MessageRoundTrip(inMsg, StringMsg.Deserialize);
            Assert.AreEqual(inMsg.data, v);
            Assert.AreEqual(inMsg.data, outMsg.data);
        }

        [Test]
        public void RoundTrip_EmptyString()
        {
            const string v = "";
            StringMsg inMsg = new StringMsg(v);
            StringMsg outMsg = MessageRoundTrip(inMsg, StringMsg.Deserialize);
            Assert.AreEqual(inMsg.data, v);
            Assert.AreEqual(inMsg.data, outMsg.data);
        }

        public T MessageRoundTrip<T>(T inMsg, Func<MessageDeserializer, T> deserialize) where T : Message
        {
            MessageSerializer ser = new MessageSerializer();
            ser.SerializeMessage(inMsg);
            MessageDeserializer deser = new MessageDeserializer();
            deser.InitWithBuffer(ser.GetBytes());
            return deserialize(deser);
        }
    }
}
