/*
© Siemens AG, 2019  
Author: Sifan Ye (sifan.ye@siemens.com) 
Licensed under the Apache License, Version 2.0 (the "License"); 
you may not use this file except in compliance with the License.    
You may obtain a copy of the License at 
<http://www.apache.org/licenses/LICENSE-2.0>.   
Unless required by applicable law or agreed to in writing, software 
distributed under the License is distributed on an "AS IS" BASIS,   
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    
See the License for the specific language governing permissions and 
limitations under the License.  
*/

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public class MessageAutoGen
    {
        static Dictionary<string, string> CachedRosPackages = new Dictionary<string, string>();

        public static string GetRosPackageName(string messagePath)
        {
            string firstParentDir = Path.GetDirectoryName(messagePath);
            if (CachedRosPackages.ContainsKey(firstParentDir))
                return CachedRosPackages[firstParentDir];

            // look for package.xml in a parent folder
            string parentDir = firstParentDir;
            while (true)
            {
                string packagePath = Path.Combine(parentDir, "package.xml");
                if (File.Exists(packagePath))
                {
                    XmlReader reader = XmlReader.Create(File.OpenRead(packagePath));
                    while(reader.Read())
                    {
                        if (reader.NodeType == XmlNodeType.Element && reader.Name == "name")
                        {
                            string result = reader.ReadElementContentAsString().Trim();
                            CachedRosPackages[firstParentDir] = result;
                            return result;
                        }
                    }
                    // if reading the file didn't find a name, break out to the old method again
                    break;
                }
                int oldLength = parentDir.Length;
                parentDir = Path.GetDirectoryName(parentDir);
                if (parentDir == null || parentDir.Length >= oldLength)
                    break;
            }

            // Failed while reading package.xml! Fall back to the old simple method
            string[] hierarchy = messagePath.Split(new char[] { '/', '\\' });
            return hierarchy[hierarchy.Length - 3];
        }

        public static string GetMessageOutFolder(string outPath, string rosPackageName)
        {
            return Path.Combine(outPath, MsgAutoGenUtilities.ResolvePackageName(rosPackageName));
        }

        public static string GetMessageClassPath(string inFilePath, string outPath)
        {
            string rosPackageName = MessageAutoGen.GetRosPackageName(inFilePath);
            string outFolder = MessageAutoGen.GetMessageOutFolder(outPath, rosPackageName);
            string extension = Path.GetExtension(inFilePath);
            string className = MsgAutoGenUtilities.CapitalizeFirstLetter(Path.GetFileNameWithoutExtension(inFilePath));
            return Path.Combine(outFolder, "msg", className + ".cs");
        }

        public static List<string> GenerateSingleMessage(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
            // If no ROS package name is provided, extract from path
            if(rosPackageName == "")
                rosPackageName = GetRosPackageName(inPath);
            outPath = GetMessageOutFolder(outPath, rosPackageName);

            string inFileName = Path.GetFileNameWithoutExtension(inPath);

            if (!(rosPackageName.Equals("std_msgs") && (inFileName.Equals("Time") || inFileName.Equals("Duration"))))
            {
                if (verbose)
                {
                    Console.WriteLine("Parsing: " + inPath);
                    Console.WriteLine("Output Location: " + outPath);
                }

                MessageTokenizer tokenizer = new MessageTokenizer(inPath, new HashSet<string>(MsgAutoGenUtilities.builtInTypesMapping.Keys));
                List<List<MessageToken>> listOfTokens = tokenizer.Tokenize();

                if (listOfTokens.Count != 1)
                {
                    throw new MessageParserException("Unexpected number of sections. Simple message should have 1 section.");
                }

                MessageParser parser = new MessageParser(listOfTokens[0], outPath, rosPackageName, "msg", MsgAutoGenUtilities.builtInTypesMapping, MsgAutoGenUtilities.builtInTypesDefaultInitialValues, MsgAutoGenUtilities.numericTypeDeserializationFunctions, MsgAutoGenUtilities.numericTypeByteSize);
                parser.Parse();
                return parser.GetWarnings();
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine(inFileName + " will not be generated");
                }
                return new List<string>();
            }
        }

        public static List<string> GeneratePackageMessages(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
            List<string> warnings = new List<string>();

            if (inPath.EndsWith("/") || inPath.EndsWith("\\"))
            {
                inPath = inPath.Remove(inPath.Length - 1);
            }

            if (rosPackageName.Equals(""))
            {
                rosPackageName = inPath.Split(new char[] { '/', '\\' }).Last();
            }

            string[] files = Directory.GetFiles(Path.Combine(inPath, "msg"), "*.msg");

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No message files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " message files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleMessage(file, outPath, rosPackageName, verbose));
                }
            }
            return warnings;
        }

        public static List<string> GenerateDirectoryMessages(string inPath, string outPath, bool verbose = false)
        {
            List<string> warnings = new List<string>();

            if (inPath.EndsWith("/") || inPath.EndsWith("\\"))
            {
                inPath = inPath.Remove(inPath.Length - 1);
            }

            string[] files = Directory.GetFiles(inPath, "*.msg", SearchOption.AllDirectories);

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No message files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " message files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleMessage(file, outPath, verbose: verbose));
                }
            }
            return warnings;
        }
    }
}
