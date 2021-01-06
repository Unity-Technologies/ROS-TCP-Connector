/*
© Siemens AG, 2019  
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

using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosMessageGeneration
{
    public abstract class DirectoryAutoGenEditorWindow : EditorWindow
    {
        [SerializeField]
        private static string lastInputDirectory = "";
        [SerializeField]
        private static string lastOutputDirectory = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosMessages");

        private string inPath = "";
        private string outPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosMessages");

        protected abstract string GenerationType { get; }
        protected abstract string FileExtension { get; }

        protected virtual void OnGUI()
        {
            GUILayout.Label("Directory " + GenerationType + " auto generation", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            inPath = EditorGUILayout.TextField("Input Path", inPath);
            if (GUILayout.Button("Browse", GUILayout.Width(150)))
            {
                inPath = EditorUtility.OpenFolderPanel("Select Folder...", lastInputDirectory, "");
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            outPath = EditorGUILayout.TextField("Output Location", outPath);
            if (GUILayout.Button("Browse", GUILayout.Width(150)))
            {
                outPath = EditorUtility.OpenFolderPanel("Select Folder...", lastOutputDirectory, "");
            }
            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("GENERATE!"))
            {
                if (inPath.Equals(""))
                {
                    EditorUtility.DisplayDialog(
                        title: "Error",
                        message: "Empty input path!\nPlease specify input path",
                        ok: "OK");
                }
                else
                {
                    lastInputDirectory = inPath;
                    lastOutputDirectory = outPath;
                    try
                    {
                        List<string> warnings = new List<string>();
                        string[] files = Directory.GetFiles(inPath, "*." + FileExtension, SearchOption.AllDirectories);
                        if (files.Length == 0)
                        {
                            EditorUtility.DisplayDialog(
                                title: "No " + GenerationType + " files found!",
                                message: "No " + GenerationType + " files found!",
                                ok: "OK");
                            Reset();
                        }
                        else
                        {
                            for (int i = 0; i < files.Length; i++)
                            {
                                string file = files[i];
                                string[] hierarchy = file.Split(new char[] { '/', '\\' });
                                string rosPackageName = hierarchy[hierarchy.Length - 3];
                                try
                                {
                                    EditorUtility.DisplayProgressBar(
                                        "Working...(" + (i + 1) + "/" + files.Length + ")",
                                        "Parsing " + file,
                                        (i + 1) / (float)files.Length);
                                    warnings.AddRange(Generate(file, outPath, rosPackageName));
                                }
                                catch (MessageTokenizerException e)
                                {
                                    Debug.LogError(e.ToString() + e.Message);
                                    EditorUtility.DisplayDialog(
                                        title: "Message Tokenizer Exception",
                                        message: e.Message,
                                        ok: "OK");
                                }
                                catch (MessageParserException e)
                                {
                                    Debug.LogError(e.ToString() + e.Message);
                                    EditorUtility.DisplayDialog(
                                        title: "Message Parser Exception",
                                        message: e.Message,
                                        ok: "OK");
                                }
                            }
                            // Done
                            EditorUtility.ClearProgressBar();
                            AssetDatabase.Refresh();
                            if (warnings.Count > 0)
                            {
                                EditorUtility.DisplayDialog(
                                    title: "Code Generation Complete",
                                    message: "Output at: " + outPath + "\nYou have " + warnings.Count + " warning(s)",
                                    ok: "OK");
                                foreach (string w in warnings)
                                {
                                    Debug.LogWarning(w);
                                }
                            }
                            else
                            {
                                EditorUtility.DisplayDialog(
                                    title: "Code Generation Complete",
                                    message: "Output at: " + outPath,
                                    ok: "OK");
                            }
                            Reset();
                        }
                    }
                    catch (DirectoryNotFoundException e)
                    {
                        EditorUtility.DisplayDialog(
                            title: "Folder not found",
                            message: e.Message,
                            ok: "OK");
                        Reset();
                    }
                }
            }
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        private void Reset()
        {
            inPath = lastInputDirectory;
            outPath = lastOutputDirectory;
        }

        protected abstract List<string> Generate(string inPath, string outPath, string rosPackageName = "");
    }
}
