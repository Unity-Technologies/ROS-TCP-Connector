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

        // we actually only want the hashset, but Unity can't serialize that - so we keep foldedOutList as a backup.
        private HashSet<string> foldedOutHash;

        [SerializeField]
        private List<string> foldedOutList = new List<string> { };
        [SerializeField]
        private Vector2 scrollPos;

        const int BUTTON_WIDTH = 100;

        [MenuItem("RosMessageGeneration/Browse...", false, 2)]
        public static void OpenWindow()
        {
            MessageGenBrowser window = GetWindow<MessageGenBrowser>(false, "Message Auto Generation", true);
            window.minSize = new Vector2(300, 300);
            window.maxSize = new Vector2(800, 1200);
            window.Show();
        }

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
                foreach (string folder in Directory.EnumerateDirectories(inPath))
                {
                    ShowFolder(folder);
                }
                EditorGUILayout.EndScrollView();
            }
        }

        void ShowFolder(string path)
        {
            string shortName = Path.GetFileName(path);
            if (shortName == "msg" || shortName == "srv")
            {
                ShowMsgFolder(path);
            }
            else
            {
                //GUIStyle hstyle = GUI.skin.
                EditorGUILayout.BeginHorizontal();
                bool isFoldedOut = HashsetFoldout(path, shortName);
                EditorGUILayout.EndHorizontal();
                if (isFoldedOut)
                {
                    EditorGUI.indentLevel++;
                    foreach (string folder in Directory.EnumerateDirectories(path))
                    {
                        ShowFolder(folder);
                    }
                    EditorGUI.indentLevel--;
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
                    if(GUILayout.Button("Build " + numMsgs + " msg" + (numMsgs > 1 ? "s" : ""), GUILayout.Width(BUTTON_WIDTH)))
                    {
                        MessageAutoGen.GenerateDirectoryMessages(path, outPath);
                        AssetDatabase.Refresh();
                    }
                }
                if (numSrvs > 0)
                {
                    if(GUILayout.Button("Build " + numSrvs + " srv" + (numSrvs > 1 ? "s" : ""), GUILayout.Width(BUTTON_WIDTH)))
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
                foreach (string folder in Directory.EnumerateDirectories(path))
                {
                    ShowFolder(folder);
                }
                EditorGUI.indentLevel--;
            }
        }

        void ShowSingleBuildLine(string file)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField(Path.GetFileName(file));
/*            if(GUILayout.Button(Path.GetFileName(file), EditorStyles.label))
            {
                UnityEditorInternal.InternalEditorUtility.OpenFileAtLineExternal(file, 0);
            }*/

            if (Path.GetExtension(file) == ".srv")
            {
                string[] fileOutPath = ServiceAutoGen.GetServiceClassPaths(file, outPath);
                bool exists = fileOutPath.All(path=>File.Exists(path));
                string buildStr = (exists ? "Rebuild" : "Build");

                if (GUILayout.Button(buildStr + " srv", GUILayout.Width(BUTTON_WIDTH)))
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

                if (GUILayout.Button(buildStr+" msg", GUILayout.Width(BUTTON_WIDTH)))
                {
                    MessageAutoGen.GenerateSingleMessage(file, outPath);
                    AssetDatabase.Refresh();
                }
            }
            EditorGUILayout.EndHorizontal();
        }

        bool HashsetFoldout(string key, string label)
        {
            if(foldedOutHash == null)
            {
                foldedOutHash = new HashSet<string>();
                foreach (string s in foldedOutList)
                    foldedOutHash.Add(s);
            }

            bool isFoldedOut = foldedOutHash.Contains(key);
            bool shouldBeFoldedOut = EditorGUILayout.Foldout(isFoldedOut, label, true, EditorStyles.foldout);

            if (shouldBeFoldedOut && !isFoldedOut)
            {
                foldedOutList.Add(key);
                foldedOutHash.Add(key);
            }
            else if (!shouldBeFoldedOut && isFoldedOut)
            {
                foldedOutList.Remove(key);
                foldedOutHash.Remove(key);
            }
            return shouldBeFoldedOut;
        }
    }
}
