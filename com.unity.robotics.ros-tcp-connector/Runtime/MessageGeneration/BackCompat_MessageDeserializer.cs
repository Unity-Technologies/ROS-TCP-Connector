using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class MessageDeserializer
{
    public int ReadLength() => throw new NotImplementedException();
    public void Read<T>(out T data) => throw new NotImplementedException();
    public void Read<T>(out T data, int length) => throw new NotImplementedException();
    public void Read<T>(out T data, Func<MessageDeserializer, Message> func, int length) => throw new NotImplementedException();
    public void Read<T>(out T data, int elementSize, int length) => throw new NotImplementedException();
}
