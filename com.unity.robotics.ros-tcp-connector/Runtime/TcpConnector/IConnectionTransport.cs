using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IConnectionTransport
    {
        Stream Connect();
    }
}
