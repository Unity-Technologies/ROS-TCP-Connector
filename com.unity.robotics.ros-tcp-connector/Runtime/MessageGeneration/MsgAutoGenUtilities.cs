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

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public class MsgAutoGenUtilities
    {
        public const string ONE_TAB = "    ";
        public const string TWO_TABS = "        ";
        public const string PROPERTY_EXTENSION = " { get; set; }";

        public const string MessageClassPrefix = "M";

        public static readonly Dictionary<string, string> builtInTypesMapping = new Dictionary<string, string>
        {
            {"bool", "bool"},
            {"int8", "sbyte"},
            {"uint8", "byte"},
            {"int16", "short"},
            {"uint16", "ushort"},
            {"int32", "int"},
            {"uint32", "uint"},
            {"int64", "long"},
            {"uint64", "ulong"},
            {"float32", "float"},
            {"float64", "double"},
            {"string", "string"},
            {"time", "MTime"},
            {"duration", "MDuration"},
            {"char", "byte"}, // Deprecated alias for uint8 -> byte  in C#
            {"byte", "sbyte"} // Deprecated alias for int8  -> sbyte in C#
        };

        public static readonly Dictionary<string, string> builtInTypesDefaultInitialValues = new Dictionary<string, string>
        {
            {"bool", "false"},
            {"sbyte", "0"},
            {"byte", "0"},
            {"short", "0"},
            {"ushort", "0"},
            {"int", "0"},
            {"uint", "0"},
            {"long", "0"},
            {"ulong", "0"},
            {"float", "0.0f"},
            {"double", "0.0"},
            {"string", "\"\""},
            {"Time", "new Time()"},
            {"Duration", "new Duration()"}
        };

        public static readonly Dictionary<string, string> numericTypeDeserializationFunctions = new Dictionary<string, string>
        {
            {"sbyte", "(sbyte)data[offset];"},
            {"byte", "data[offset];"},
            {"bool", "BitConverter.ToBoolean(data, offset)"},
            {"char", "BitConverter.ToChar(data, offset)"},
            {"double", "BitConverter.ToDouble(data, offset)"},
            {"short", "BitConverter.ToInt16(data, offset)"},
            {"int", "BitConverter.ToInt32(data, offset)"},
            {"long", "BitConverter.ToInt64(data, offset)"},
            {"float", "BitConverter.ToSingle(data, offset)"},
            {"ushort", "BitConverter.ToUInt16(data, offset)"},
            {"uint", "BitConverter.ToUInt32(data, offset)"},
            {"ulong", "BitConverter.ToUInt64(data, offset)"}
        };

        public static readonly Dictionary<string, int> numericTypeByteSize = new Dictionary<string, int>
        {
            {"sbyte", 1},
            {"byte", 1},
            {"bool", 1},
            {"char", 2},
            {"double", 8},
            {"short", 2},
            {"int", 4},
            {"long", 8},
            {"float", 4},
            {"ushort", 2},
            {"uint", 4},
            {"ulong", 8}
        };


        public static HashSet<string> CSharpKeywords = new HashSet<string>
        {
            "abstract", "as", "base", "bool", "break", "byte", "case", "catch", "char",
            "checked", "class", "continue", "decimal", "default", "delegate", "do", "double",
            "else", "enum", "event", "explicit", "extern", "false", "finally", "fixed", "const",
            "float", "for", "foreach", "goto", "if", "implicit", "in", "int", "interface",
            "internal", "is", "lock", "long", "namespace", "new", "null", "object",
            "operator", "out", "override", "params", "private", "protected", "public",
            "readonly", "ref", "return", "sbyte", "sealed", "short", "sizeof",
            "stackalloc", "static", "string", "struct", "switch", "this", "throw", "true",
            "try", "typeof", "uint", "ulong", "unchecked", "unsafe", "ushort", "using",
            "virtual", "void", "volatile", "while"
        };

        public static string CapitalizeFirstLetter(string s)
        {
            return Char.ToUpper(s[0]) + s.Substring(1);
        }

        public static string LowerFirstLetter(string s)
        {
            return Char.ToLower(s[0]) + s.Substring(1);
        }

        public static string PascalCase(string s)
        {
            string[] words = s.Split('_');
            for (int i = 0; i < words.Length; i++)
            {
                words[i] = CapitalizeFirstLetter(words[i]);
            }
            return String.Join("", words);

        }

        public static string ResolvePackageName(string s)
        {
            if (s.Contains("_msgs") || s.Contains("_srvs") || s.Contains("_actions"))
            {
                return PascalCase(s.Substring(0, s.LastIndexOf('_')));
            }
            if (s.Contains("_"))
            {
                return PascalCase(s);
            }
            return CapitalizeFirstLetter(s);
        }
    }
}
