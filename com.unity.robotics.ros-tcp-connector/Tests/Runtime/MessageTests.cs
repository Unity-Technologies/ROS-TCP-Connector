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
        class TestMessage : Message
        {
        }

        class TestResponse : Message
        {
        }

        [Test]
        public void MessageRegistry_CanRegister()
        {
            const string rosMessageName = "testmessage";
            Func<MessageDeserializer, TestMessage> deserializer = des => new TestMessage();

            MessageRegistry.Register(rosMessageName, deserializer);

            Assert.AreEqual(rosMessageName, MessageRegistry.GetRosMessageName<TestMessage>());
            Assert.AreEqual(deserializer, MessageRegistry.GetDeserializeFunction<TestMessage>());
            Assert.AreEqual(deserializer, MessageRegistry.GetDeserializeFunction(rosMessageName));
        }

        [Test]
        public void MessageRegistry_CanRegisterSubtopics()
        {
            const string rosMessageName = "testmessage";
            Func<MessageDeserializer, TestMessage> deserializer = des => new TestMessage();
            Func<MessageDeserializer, TestResponse> deserializer2 = des => new TestResponse();

            MessageRegistry.Register(rosMessageName, deserializer, MessageSubtopic.Default);
            MessageRegistry.Register(rosMessageName, deserializer2, MessageSubtopic.Response);

            Assert.AreEqual(rosMessageName, MessageRegistry.GetRosMessageName<TestMessage>());
            Assert.AreEqual(rosMessageName, MessageRegistry.GetRosMessageName<TestResponse>());
            Assert.AreEqual(MessageSubtopic.Default, MessageRegistry.GetSubtopic<TestMessage>());
            Assert.AreEqual(MessageSubtopic.Response, MessageRegistry.GetSubtopic<TestResponse>());
            Assert.AreEqual(deserializer, MessageRegistry.GetDeserializeFunction<TestMessage>());
            Assert.AreEqual(deserializer, MessageRegistry.GetDeserializeFunction(rosMessageName, MessageSubtopic.Default));
            Assert.AreEqual(deserializer2, MessageRegistry.GetDeserializeFunction<TestResponse>());
            Assert.AreEqual(deserializer2, MessageRegistry.GetDeserializeFunction(rosMessageName, MessageSubtopic.Response));
            Assert.IsNull(MessageRegistry.GetDeserializeFunction(rosMessageName, MessageSubtopic.Goal));
        }

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
