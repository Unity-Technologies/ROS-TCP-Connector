namespace Unity.Robotics.ROSTCPConnector.SysCommands
{
    struct ConnectionsParameters
    {
        public bool keep_connections;
        public float timeout_in_s;
    }
    
    struct Subscribe
    {
        public string topic;
        public string message_name;
    }

    struct Publish
    {
        public string topic;
        public string message_name;
    }
}