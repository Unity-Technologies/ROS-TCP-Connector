using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IMessagePool
    {
        void AddMessage(Message messageToRecycle);
    }

    public class MessagePool<T> : IMessagePool where T : Message
    {
        Queue<T> m_Contents;
        int m_MaxSize;

        public MessagePool(int maxSize)
        {
            m_MaxSize = maxSize;
            m_Contents = new Queue<T>(maxSize);
        }

        public void AddMessage(T messageToRecycle)
        {
            // not bothering to lock here - if multiple threads add things simultaneously we could end up a little over m_MaxSize but it's not a big deal
            if (m_Contents.Count < m_MaxSize)
            {
                m_Contents.Enqueue(messageToRecycle);
            }
        }

        void IMessagePool.AddMessage(Message message)
        {
            if (!(message is T))
            {
                throw new System.ArgumentException($"MessagePool<{MessageRegistry.GetRosMessageName<T>()}> can't store a \"{message.RosMessageName}\" message.");
            }

            AddMessage((T)message);
        }

        /**
         * @return a message of type T full of garbage data, be sure to update it accordingly.
         */
        public T GetMessage()
        {
            T result = null;

            // when Unity migrates to .NET 5 this should become TryDequeue
            lock (m_Contents)
            {
                if (m_Contents.Count > 0)
                {
                    result = m_Contents.Dequeue();
                }
            }

            if (result == null)
            {
                result = System.Activator.CreateInstance<T>();
            }

            return result;
        }
    }
}
