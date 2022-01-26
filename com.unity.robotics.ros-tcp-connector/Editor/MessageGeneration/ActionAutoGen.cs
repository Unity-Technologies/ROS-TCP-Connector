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

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public class ActionAutoGen
    {
        private static readonly string[] types = { "Goal", "Result", "Feedback" };
        private static readonly MessageSubtopic[] subtopics = { MessageSubtopic.Goal, MessageSubtopic.Result, MessageSubtopic.Feedback };

        public static string[] GetActionClassPaths(string inFilePath, string outPath)
        {
            string rosPackageName = MessageAutoGen.GetRosPackageName(inFilePath);
            string outFolder = MessageAutoGen.GetMessageOutFolder(outPath, rosPackageName);
            string extension = Path.GetExtension(inFilePath);
            string className = MsgAutoGenUtilities.CapitalizeFirstLetter(Path.GetFileNameWithoutExtension(inFilePath)) + MsgAutoGenUtilities.ActionClassSuffix;

            string[] result = new string[types.Length];
            for (int Idx = 0; Idx < types.Length; ++Idx)
            {
                result[Idx] = Path.Combine(outFolder, "action", className + types[Idx] + ".cs");
            }
            return result;
        }

        public static List<string> GenerateSingleAction(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
            // If no ROS package name is provided, extract from path
            if (rosPackageName.Equals(""))
            {
                string[] hierarchy = inPath.Split(new char[] { '/', '\\' });
                rosPackageName = hierarchy[hierarchy.Length - 3];
            }

            outPath = Path.Combine(outPath, MsgAutoGenUtilities.ResolvePackageName(rosPackageName));

            string inFileName = Path.GetFileNameWithoutExtension(inPath);

            if (verbose)
            {
                Console.WriteLine("Parsing: " + inPath);
                Console.WriteLine("Output Location: " + outPath);
            }

            MessageTokenizer tokenizer = new MessageTokenizer(inPath, new HashSet<string>(MsgAutoGenUtilities.builtInTypesMapping.Keys));
            List<List<MessageToken>> listsOfTokens = tokenizer.Tokenize();

            if (listsOfTokens.Count != 3)
            {
                throw new MessageParserException("Unexpected number of sections. Action should have 3 sections.");
            }

            List<string> warnings = new List<string>();

            ActionWrapper actionWrapper = new ActionWrapper(inPath, rosPackageName, outPath);

            for (int i = 0; i < listsOfTokens.Count; i++)
            {
                List<MessageToken> tokens = listsOfTokens[i];

                // Action is made up of goal, result, feedback
                string className = inFileName + types[i] + MsgAutoGenUtilities.ActionClassSuffix;

                // Parse and generate goal, result, feedback messages
                MessageParser parser = new MessageParser(
                    tokens,
                    outPath,
                    rosPackageName,
                    "action",
                    MsgAutoGenUtilities.builtInTypesMapping,
                    MsgAutoGenUtilities.builtInTypesDefaultInitialValues,
                    className,
                    subtopic: subtopics[i]
                );
                parser.Parse();
                warnings.AddRange(parser.GetWarnings());

                // Generate action section wrapper messages
                actionWrapper.WrapActionSections(types[i]);
            }

            // Generate action wrapper
            actionWrapper.WrapAction();

            return warnings;
        }

        public static List<string> GeneratePackageActions(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
            List<string> warnings = new List<string>();

            string[] files = Directory.GetFiles(Path.Combine(inPath, "action"), "*.action");

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No action files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " action files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleAction(file, outPath, rosPackageName, verbose));
                }
            }

            return warnings;
        }

        public static List<string> GenerateDirectoryActions(string inPath, string outPath, bool verbose = false)
        {
            List<string> warnings = new List<string>();

            if (inPath.EndsWith("/") || inPath.EndsWith("\\"))
            {
                inPath = inPath.Remove(inPath.Length - 1);
            }

            string[] files = Directory.GetFiles(inPath, "*.action", SearchOption.AllDirectories);

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No action files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " action files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleAction(file, outPath, verbose: verbose));
                }
            }
            return warnings;
        }
    }

    public class ActionWrapper
    {

        private const string ONE_TAB = "    ";
        private const string TWO_TABS = "        ";
        private const string THREE_TABS = "            ";

        private readonly string inPath;
        private readonly string inFileName;

        private readonly string rosPackageName;

        private readonly string outPath;

        private Dictionary<string, string> symbolTable;

        public ActionWrapper(string inPath, string rosPackageName, string outPath)
        {
            this.inPath = inPath;
            this.inFileName = Path.GetFileNameWithoutExtension(inPath);
            this.rosPackageName = rosPackageName;
            this.outPath = Path.Combine(outPath, "action");
        }

        private string GenerateDefaultValueConstructor(string className)
        {
            string constructor = "";

            constructor += TWO_TABS + "public " + className + "() : base()\n";
            constructor += TWO_TABS + "{\n";

            foreach (string identifier in symbolTable.Keys)
            {
                constructor += TWO_TABS + ONE_TAB + "this." + identifier + " = ";
                string type = symbolTable[identifier];
                constructor += "new " + type + "();\n";
            }

            constructor += TWO_TABS + "}\n";

            return constructor;
        }

        private string GenerateParameterizedConstructor(string className, string msgType)
        {
            string constructor = "";

            string paramsIn = "";
            string paramsOut = "";
            string assignments = "";

            if (msgType.Equals("Goal"))
            {
                paramsIn += "HeaderMsg header, GoalIDMsg goal_id, ";
                paramsOut += "header, goal_id";
            }
            else if (msgType.Equals("Result") || msgType.Equals("Feedback"))
            {
                paramsIn += "HeaderMsg header, GoalStatusMsg status, ";
                paramsOut += "header, status";
            }

            foreach (string identifier in symbolTable.Keys)
            {
                string type = symbolTable[identifier];
                paramsIn += type + " " + identifier + ", ";
                assignments += TWO_TABS + ONE_TAB + "this." + identifier + " = " + identifier + ";\n";
            }

            if (!paramsIn.Equals(""))
            {
                paramsIn = paramsIn.Substring(0, paramsIn.Length - 2);
            }

            constructor += TWO_TABS + "public " + className + "(" + paramsIn + ") : base(" + paramsOut + ")\n";
            constructor += TWO_TABS + "{\n";
            constructor += assignments;
            constructor += TWO_TABS + "}\n";

            return constructor;
        }

        private string GenerateDeserializerConstructor(string className, bool callBase = true)
        {
            string constructor = "";
            string assignments = "";

            foreach (string identifier in symbolTable.Keys)
            {
                string type = symbolTable[identifier];

                if (MsgAutoGenUtilities.nonMessageTypes.Contains(type))
                {
                    assignments += THREE_TABS + $"deserializer.Read(out this.{identifier});\n";
                }
                else
                {
                    assignments += THREE_TABS + $"this.{identifier} = {type}.Deserialize(deserializer);\n";
                }
            }

            constructor += TWO_TABS + $"public static {className} Deserialize(MessageDeserializer deserializer) => new {className}(deserializer);\n\n";
            constructor += TWO_TABS + $"{className}(MessageDeserializer deserializer){(callBase ? " : base(deserializer)" : "")}\n";
            constructor += TWO_TABS + "{\n";
            constructor += assignments;
            constructor += TWO_TABS + "}\n";

            return constructor;
        }

        private string GenerateSerializationStatements(string msgType)
        {
            string function = "";
            function += TWO_TABS + "public override void SerializeTo(MessageSerializer serializer)\n";
            function += TWO_TABS + "{\n";

            string[] inheritedParams = new string[0];

            // Inherited params
            if (msgType.Equals("Goal"))
            {
                inheritedParams = new[] { "header", "goal_id" };

            }
            else if (msgType.Equals("Result") || msgType.Equals("Feedback"))
            {
                inheritedParams = new[] { "header", "status" };
            }

            foreach (string paramName in inheritedParams)
            {
                function += THREE_TABS + "serializer.Write(this." + paramName + ");\n";
            }

            foreach (string identifier in symbolTable.Keys)
            {
                function += THREE_TABS + "serializer.Write(this." + identifier + ");\n";
            }

            function += TWO_TABS + "}\n\n";

            return function;
        }

        public void WrapActionSections(string type)
        {
            string wrapperName = inFileName + "Action" + type + MsgAutoGenUtilities.ActionClassSuffix;
            string msgName = inFileName + type + MsgAutoGenUtilities.ActionClassSuffix;

            string outPath = Path.Combine(this.outPath, wrapperName + ".cs");

            string imports =
                "using System.Collections.Generic;\n" +
                "using Unity.Robotics.ROSTCPConnector.MessageGeneration;\n" +
                "using RosMessageTypes.Std;\n" +
                "using RosMessageTypes.Actionlib;\n\n";

            symbolTable = new Dictionary<string, string>();

            using (StreamWriter writer = new StreamWriter(outPath, false))
            {
                // Write imports
                writer.Write(imports);

                // Write namespace
                writer.Write(
                    "namespace RosMessageTypes." + MsgAutoGenUtilities.ResolvePackageName(rosPackageName) + "\n" +
                    "{\n"
                    );

                // Write class declaration
                writer.Write(
                    ONE_TAB + "public class " + wrapperName + " : Action" + type + "<" + msgName + ">\n" +
                    ONE_TAB + "{\n"
                    );

                // Write ROS package name
                writer.Write(
                    TWO_TABS + "public const string k_RosMessageName = \"" + rosPackageName + "/" + inFileName + "Action" + type + "\";\n" +
                    TWO_TABS + "public override string RosMessageName => k_RosMessageName;\n\n"
                    );

                // Record goal/result/feedback declaration
                symbolTable.Add(MsgAutoGenUtilities.LowerFirstLetter(type), msgName);

                writer.Write("\n");

                // Write default value constructor
                writer.Write(GenerateDefaultValueConstructor(wrapperName) + "\n");

                // Write parameterized constructor
                writer.Write(GenerateParameterizedConstructor(wrapperName, type));

                writer.Write(GenerateDeserializerConstructor(wrapperName));

                writer.Write(GenerateSerializationStatements(type));

                // Omit the subtype because subtype is baked into the class name
                writer.Write(MsgAutoGenUtilities.InitializeOnLoad());

                // Close class
                writer.Write(ONE_TAB + "}\n");
                // Close namespace
                writer.Write("}\n");
            }
        }

        public void WrapAction()
        {
            string msgNamePrefix = inFileName + MsgAutoGenUtilities.ActionClassSuffix;
            string className = msgNamePrefix + "Action";
            string type = "wrapper";

            string outPath = Path.Combine(this.outPath, className + ".cs");

            string imports =
                "using System.Collections.Generic;\n" +
                "using Unity.Robotics.ROSTCPConnector.MessageGeneration;\n" +
                "\n\n";

            symbolTable = new Dictionary<string, string>();

            using (StreamWriter writer = new StreamWriter(outPath, false))
            {
                // Write imports
                writer.Write(imports);

                // Write namespace
                writer.Write(
                    "namespace RosMessageTypes." + MsgAutoGenUtilities.ResolvePackageName(rosPackageName) + "\n" +
                    "{\n"
                    );

                // Write class declaration
                string[] genericParams = new string[] {
                    msgNamePrefix + "ActionGoal",
                    msgNamePrefix + "ActionResult",
                    msgNamePrefix + "ActionFeedback",
                    msgNamePrefix + "Goal",
                    msgNamePrefix + "Result",
                    msgNamePrefix + "Feedback"
                };
                writer.Write(
                    ONE_TAB + "public class " + className + " : Action<" + string.Join(", ", genericParams) + ">\n" +
                    ONE_TAB + "{\n"
                    );

                // Write ROS package name
                writer.Write(
                    TWO_TABS + "public const string k_RosMessageName = \"" + rosPackageName + "/" + inFileName + "Action" + "\";\n" +
                    TWO_TABS + "public override string RosMessageName => k_RosMessageName;\n\n"
                    );

                // Record variables
                // Action Goal
                symbolTable.Add("action_goal", className + "Goal");
                // Action Result
                symbolTable.Add("action_result", className + "Result");
                //Action Feedback
                symbolTable.Add("action_feedback", className + "Feedback");

                // Write default value constructor
                writer.Write("\n");
                writer.Write(GenerateDefaultValueConstructor(className) + "\n");
                writer.Write(GenerateDeserializerConstructor(className, false) + "\n");

                writer.Write(GenerateSerializationStatements(type));

                // Close class
                writer.Write(ONE_TAB + "}\n");
                // Close namespace
                writer.Write("}\n");
            }
        }

    }
}

