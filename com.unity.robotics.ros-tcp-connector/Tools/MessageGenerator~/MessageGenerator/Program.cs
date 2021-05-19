using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace MessageGenerator
{
    class Program
    {
        static void Main(string[] args)
        {
            if (args.Length != 2)
            {
                Console.Error.WriteLine("Usage: MessageGenerator <path/to/file.msg> <destination folder>");
                return;
            }

            List<string> warnings;
            string extension = System.IO.Path.GetExtension(args[0]);
            switch (extension)
            {
                case ".msg":
                    warnings = MessageAutoGen.GenerateSingleMessage(args[0], args[1]);
                    break;
                case ".srv":
                    warnings = ServiceAutoGen.GenerateSingleService(args[0], args[1]);
                    break;
                case ".action":
                    warnings = ActionAutoGen.GenerateSingleAction(args[0], args[1]);
                    break;
                default:
                    Console.Error.WriteLine("Don't understand extension " + extension);
                    return;
            }

            foreach (string warning in warnings)
            {
                Console.Error.WriteLine(warning);
            }
        }
    }
}
