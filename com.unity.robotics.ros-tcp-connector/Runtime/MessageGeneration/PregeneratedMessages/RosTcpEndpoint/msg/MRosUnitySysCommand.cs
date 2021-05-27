//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RosTcpEndpoint
{
    public class MRosUnitySysCommand : Message
    {
        public const string k_RosMessageName = "ros_tcp_endpoint/RosUnitySysCommand";
        public override string RosMessageName => k_RosMessageName;

        public string command;
        public string params_json;

        public MRosUnitySysCommand()
        {
            this.command = "";
            this.params_json = "";
        }

        public MRosUnitySysCommand(string command, string params_json)
        {
            this.command = command;
            this.params_json = params_json;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.command));
            listOfSerializations.Add(SerializeString(this.params_json));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var commandStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.command = DeserializeString(data, offset, commandStringBytesLength);
            offset += commandStringBytesLength;
            var params_jsonStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.params_json = DeserializeString(data, offset, params_jsonStringBytesLength);
            offset += params_jsonStringBytesLength;

            return offset;
        }

        public override string ToString()
        {
            return "MRosUnitySysCommand: " +
            "\ncommand: " + command.ToString() +
            "\nparams_json: " + params_json.ToString();
        }

        [UnityEngine.RuntimeInitializeOnLoadMethod(UnityEngine.RuntimeInitializeLoadType.AfterAssembliesLoaded)]
        static void OnLoad()
        {
            MessageRegistry.Register<MRosUnitySysCommand>(k_RosMessageName);
        }
    }
}