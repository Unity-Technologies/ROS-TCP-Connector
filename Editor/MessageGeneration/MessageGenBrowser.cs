using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace RosMessageGeneration
{
    public class MessageGenBrowser : EditorWindow
    {
        [SerializeField]
        private static string lastInputDirectory = string.Empty;
        [SerializeField]
        private static string lastOutputDirectory = string.Empty;

        private string inPath = "";
        private string outPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosMessages");
        private List<string> foldedOutList = new List<string> { };

        [MenuItem("RosMessageGeneration/Browse...", false, 2)]
        public static void OpenWindow()
        {
            MessageGenBrowser window = GetWindow<MessageGenBrowser>(false, "Message Auto Generation", true);
            window.minSize = new Vector2(500, 300);
            window.maxSize = new Vector2(800, 1200);
            window.Show();
        }

        protected string FileExtension { get; }
        Vector2 scrollPos;

        protected virtual void OnGUI()
        {
            EditorGUILayout.BeginHorizontal();
            inPath = EditorGUILayout.TextField("Input Path", inPath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(150)))
            {
                inPath = EditorUtility.OpenFolderPanel("Select Folder...", lastInputDirectory, "");
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            outPath = EditorGUILayout.TextField("Output Location", outPath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(150)))
            {
                outPath = EditorUtility.OpenFolderPanel("Select Folder...", lastOutputDirectory, "");
            }
            EditorGUILayout.EndHorizontal();

            if (!Directory.Exists(inPath))
            {
                if (GUILayout.Button("Select a folder", GUILayout.Width(200)))
                {
                    inPath = EditorUtility.OpenFolderPanel("Select Folder...", lastInputDirectory, "");
                }
            }
            else
            {
                EditorGUILayout.LabelField(inPath+":");
                scrollPos = EditorGUILayout.BeginScrollView(scrollPos);
                ShowContents(inPath);
                EditorGUILayout.EndScrollView();
            }
        }

        void ShowContents(string path)
        {
            foreach (string folder in Directory.EnumerateDirectories(path))
            {
                string shortName = Path.GetFileName(folder);
                if (shortName == "msg" || shortName == "srv")
                {
                    ShowMsgFolder(folder);
                }
                else
                {
                    //GUIStyle hstyle = GUI.skin.
                    EditorGUILayout.BeginHorizontal();
                    bool isFoldedOut = HashsetFoldout(folder, shortName);
                    EditorGUILayout.EndHorizontal();
                    if (isFoldedOut)
                    {
                        EditorGUI.indentLevel++;
                        ShowContents(folder);
                        EditorGUI.indentLevel--;
                    }
                }
            }
        }

        void ShowMsgFolder(string path)
        {
            EditorGUILayout.BeginHorizontal();
            string shortName = Path.GetFileName(path);
            bool isFoldedOut = HashsetFoldout(path, shortName);
            if (!isFoldedOut)
            {
                int numMsgs = 0;
                int numSrvs = 0;
                foreach (string file in Directory.EnumerateFiles(path))
                {
                    string ext = Path.GetExtension(file);
                    switch (ext)
                    {
                        case ".msg":
                            numMsgs++;
                            break;
                        case ".srv":
                            numSrvs++;
                            break;
                    }
                }
                if (numMsgs > 0)
                {
                    if(GUILayout.Button("Build " + numMsgs + " msg" + (numMsgs > 1 ? "s" : ""), GUILayout.Width(200)))
                    {
                        MessageAutoGen.GenerateDirectoryMessages(path, outPath);
                        AssetDatabase.Refresh();
                    }
                }
                if (numSrvs > 0)
                {
                    if(GUILayout.Button("Build " + numSrvs + " srv" + (numSrvs > 1 ? "s" : ""), GUILayout.Width(200)))
                    {
                        MessageAutoGen.GenerateDirectoryMessages(path, outPath);
                        AssetDatabase.Refresh();
                    }
                }
            }
            EditorGUILayout.EndHorizontal();
            if (isFoldedOut)
            {
                EditorGUI.indentLevel++;
                foreach (string file in Directory.EnumerateFiles(path))
                {
                    string ext = Path.GetExtension(file);
                    if (ext == ".msg" || ext == ".srv")
                        ShowSingleBuildLine(file);
                }
                EditorGUI.indentLevel--;
            }
        }

        void ShowSingleBuildLine(string file)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField(Path.GetFileName(file));

            if (Path.GetExtension(file) == ".srv")
            {
                string[] fileOutPath = ServiceAutoGen.GetServiceClassPaths(file, outPath);
                bool exists = fileOutPath.All(path=>File.Exists(path));
                string buildStr = (exists ? "Rebuild" : "Build");

                if (GUILayout.Button(buildStr + " srv", GUILayout.Width(200)))
                {
                    ServiceAutoGen.GenerateSingleService(file, outPath);
                    AssetDatabase.Refresh();
                }
            }
            else
            {
                string fileOutPath = MessageAutoGen.GetMessageClassPath(file, outPath);
                bool exists = File.Exists(fileOutPath);
                string buildStr = (exists ? "Rebuild" : "Build");

                if (GUILayout.Button(buildStr+" msg", GUILayout.Width(200)))
                {
                    MessageAutoGen.GenerateSingleMessage(file, outPath);
                    AssetDatabase.Refresh();
                }
            }
            EditorGUILayout.EndHorizontal();
        }

        bool HashsetFoldout(string key, string label)
        {
            bool isFoldedOut = foldedOutList.Contains(key);
            bool shouldBeFoldedOut = EditorGUILayout.Foldout(isFoldedOut, label, true, EditorStyles.foldout);

            if (shouldBeFoldedOut && !isFoldedOut)
            {
                foldedOutList.Add(key);
            }
            else if (!shouldBeFoldedOut && isFoldedOut)
            {
                foldedOutList.Remove(key);
            }
            return shouldBeFoldedOut;
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        private void Reset()
        {
            inPath = "";
            outPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosMessages");
        }

        protected List<string> Generate(string inPath, string outPath, string rosPackageName = "")
        {
            return new List<string>();
        }
    }
}
