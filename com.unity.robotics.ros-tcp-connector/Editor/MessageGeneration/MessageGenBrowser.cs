using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.Editor.MessageGeneration
{
    public class MessageGenBrowser : EditorWindow, ISerializationCallbackReceiver
    {
        HashSet<string> m_FoldedOutHash = new HashSet<string>();

        // Unity doesn't know how to serialize a HashSet, so do it manually
        [SerializeField]
        List<string> m_FoldedOutHashSerialized = new List<string>();

        public void OnBeforeSerialize()
        {
            m_FoldedOutHashSerialized = m_FoldedOutHash.ToList();
        }

        public void OnAfterDeserialize()
        {
            m_FoldedOutHash.Clear();
            foreach (string s in m_FoldedOutHashSerialized)
                m_FoldedOutHash.Add(s);
        }

        [SerializeField]
        Vector2 m_ScrollPos;

        bool m_IsCacheDirty = true;
        CachedEntry m_CacheRoot;

        const int BUTTON_WIDTH = 100;

        enum CachedEntryStatus
        {
            Folder,
            Ignored,
            UnbuiltMsgFile,
            BuiltMsgFile,
            UnbuiltSrvFile,
            BuiltSrvFile,
            UnbuiltActionFile,
            BuiltActionFile,
        }

        struct CachedEntry
        {
            public string path;
            public string label => Path.GetFileName(path);
            public CachedEntryStatus status;
            public List<CachedEntry> contents;
            public string buildLabel;
            public bool hasBuildButton => buildLabel != null;
        }

        [MenuItem("Robotics/Generate ROS Messages...", false, 2)]
        public static void OpenWindow()
        {
            MessageGenBrowser window = GetWindow<MessageGenBrowser>(false, "ROS Message Browser", true);
            window.minSize = new Vector2(300, 300);
            window.maxSize = new Vector2(800, 1200);
            window.Show();
        }

        protected virtual void OnGUI()
        {
            EditorGUILayout.BeginHorizontal();
            MessageGenBrowserSettings settings = MessageGenBrowserSettings.Get();
            string inPath = settings.inputPath;
            inPath = EditorGUILayout.TextField("ROS message path", inPath);
            if (GUILayout.Button("Browse", GUILayout.Width(100)))
            {
                inPath = EditorUtility.OpenFolderPanel("Select ROS message folder", inPath, "");
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            string relativeOutPath = settings.relativeOutPath;
            relativeOutPath = EditorGUILayout.TextField("Built message path", relativeOutPath);
            if (GUILayout.Button("Browse", GUILayout.Width(100)))
            {
                string absOutPath = EditorUtility.OpenFolderPanel("Select Unity message folder", settings.outputPath, "");
                if (absOutPath != "")
                    relativeOutPath = MessageGenBrowserSettings.ToRelativePath(absOutPath);
            }
            EditorGUILayout.EndHorizontal();

            if (!Directory.Exists(inPath))
            {
                if (GUILayout.Button("Select ROS folder", GUILayout.Width(200)))
                {
                    inPath = EditorUtility.OpenFolderPanel("Select ROS message folder", inPath, "");
                }
            }
            else
            {
                EditorGUILayout.LabelField(inPath + ":");
                m_ScrollPos = EditorGUILayout.BeginScrollView(m_ScrollPos);
                if (m_IsCacheDirty || m_CacheRoot.path == null)
                    RefreshCache(inPath);

                ShowCachedEntry(m_CacheRoot);
                EditorGUILayout.EndScrollView();
            }

            if (inPath != settings.inputPath || relativeOutPath != settings.relativeOutPath)
            {
                if (inPath != "")
                    settings.inputPath = inPath;
                settings.relativeOutPath = relativeOutPath;
                settings.Save();
                m_IsCacheDirty = true;
            }
        }

        void ShowCachedEntry(CachedEntry entry)
        {
            if (entry.status == CachedEntryStatus.Ignored)
            {
                return;
            }
            else if (entry.status != CachedEntryStatus.Folder)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField(entry.label);

                if (entry.hasBuildButton && GUILayout.Button(entry.buildLabel, GUILayout.Width(BUTTON_WIDTH)))
                {
                    // build this msg, srv, or action file
                    switch (entry.status)
                    {
                        case CachedEntryStatus.BuiltMsgFile:
                        case CachedEntryStatus.UnbuiltMsgFile:
                            MessageAutoGen.GenerateSingleMessage(entry.path, MessageGenBrowserSettings.Get().outputPath);
                            break;
                        case CachedEntryStatus.BuiltSrvFile:
                        case CachedEntryStatus.UnbuiltSrvFile:
                            ServiceAutoGen.GenerateSingleService(entry.path, MessageGenBrowserSettings.Get().outputPath);
                            break;
                        case CachedEntryStatus.BuiltActionFile:
                        case CachedEntryStatus.UnbuiltActionFile:
                            ActionAutoGen.GenerateSingleAction(entry.path, MessageGenBrowserSettings.Get().outputPath);
                            break;
                    }
                    AssetDatabase.Refresh();
                    m_IsCacheDirty = true;
                }
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                bool isFoldedOut = m_FoldedOutHash.Contains(entry.path);

                EditorGUILayout.BeginHorizontal();
                bool shouldBeFoldedOut = EditorGUILayout.Foldout(isFoldedOut, entry.label, true, EditorStyles.foldout);
                if (entry.hasBuildButton && GUILayout.Button(entry.buildLabel, GUILayout.Width(BUTTON_WIDTH)))
                {
                    // build this directory
                    MessageAutoGen.GenerateDirectoryMessages(entry.path, MessageGenBrowserSettings.Get().outputPath);
                    ServiceAutoGen.GenerateDirectoryServices(entry.path, MessageGenBrowserSettings.Get().outputPath);
                    ActionAutoGen.GenerateDirectoryActions(entry.path, MessageGenBrowserSettings.Get().outputPath);
                    AssetDatabase.Refresh();
                    m_IsCacheDirty = true;
                }
                EditorGUILayout.EndHorizontal();

                if (isFoldedOut)
                {
                    EditorGUI.indentLevel++;
                    foreach (CachedEntry subEntry in entry.contents)
                    {
                        ShowCachedEntry(subEntry);
                    }
                    EditorGUI.indentLevel--;
                }

                if (shouldBeFoldedOut != isFoldedOut)
                {
                    m_IsCacheDirty = true;

                    if (shouldBeFoldedOut)
                        m_FoldedOutHash.Add(entry.path);
                    else
                        m_FoldedOutHash.Remove(entry.path);
                }
            }
        }

        void RefreshCache(string inPath)
        {
            m_CacheRoot = CacheFolder(inPath);
            m_IsCacheDirty = false;
        }

        CachedEntry CacheFolder(string path)
        {
            List<CachedEntry> contents = new List<CachedEntry>();
            string buildLabel = null;

            bool isExpanded = m_FoldedOutHash.Contains(path);
            int numFolders = 0;
            foreach (string folder in Directory.EnumerateDirectories(path))
            {
                if (isExpanded)
                {
                    CachedEntry entry = CacheFolder(folder);
                    if (entry.status != CachedEntryStatus.Ignored)
                    {
                        contents.Add(entry);
                    }
                }
                numFolders++;
            }

            int numMsgs = 0;
            int numSrvs = 0;
            int numActions = 0;
            foreach (string file in Directory.EnumerateFiles(path))
            {
                CachedEntryStatus status = GetFileStatus(file);
                if (status == CachedEntryStatus.Ignored)
                    continue;

                string type = "";
                if (status == CachedEntryStatus.BuiltSrvFile || status == CachedEntryStatus.UnbuiltSrvFile)
                {
                    numSrvs++;
                    type = "srv";
                }
                else if (status == CachedEntryStatus.BuiltActionFile || status == CachedEntryStatus.UnbuiltActionFile)
                {
                    numActions++;
                    type = "action";
                }
                else
                {
                    numMsgs++;
                    type = "msg";
                }

                if (isExpanded)
                {
                    if (status == CachedEntryStatus.BuiltMsgFile || status == CachedEntryStatus.BuiltSrvFile || status == CachedEntryStatus.BuiltActionFile)
                        buildLabel = "Rebuild " + type;
                    else
                        buildLabel = "Build " + type;

                    contents.Add(new CachedEntry()
                    {
                        path = file,
                        status = status,
                        buildLabel = buildLabel,
                    });
                }
            }

            List<string> buildStrings = new List<string>();

            if (numMsgs > 0)
                buildStrings.Add(numMsgs + " msg" + (numMsgs > 1 ? "s" : ""));

            if (numSrvs > 0)
                buildStrings.Add(numSrvs + " srv" + (numSrvs > 1 ? "s" : ""));

            if (numActions > 0)
                buildStrings.Add(numActions + " action" + (numActions > 1 ? "s" : ""));

            if(buildStrings.Count > 0)
                buildLabel = "Build " + string.Join(", ", buildStrings.Select(x => x.ToString()));

            return new CachedEntry()
            {
                path = path,
                contents = contents,
                status = (numMsgs + numSrvs + numActions + numFolders == 0) ? CachedEntryStatus.Ignored : CachedEntryStatus.Folder,
                buildLabel = buildLabel,
            };
        }

        CachedEntryStatus GetFileStatus(string path)
        {
            switch (Path.GetExtension(path))
            {
                case ".msg":
                    string builtMessagePath = MessageAutoGen.GetMessageClassPath(path, MessageGenBrowserSettings.Get().outputPath);
                    return File.Exists(builtMessagePath) ? CachedEntryStatus.BuiltMsgFile : CachedEntryStatus.UnbuiltMsgFile;
                case ".srv":
                    string[] builtServicePaths = ServiceAutoGen.GetServiceClassPaths(path, MessageGenBrowserSettings.Get().outputPath);
                    return builtServicePaths.All(file => File.Exists(file)) ? CachedEntryStatus.BuiltSrvFile : CachedEntryStatus.UnbuiltSrvFile;
                case ".action":
                    string[] builtActionPaths = ActionAutoGen.GetActionClassPaths(path, MessageGenBrowserSettings.Get().outputPath);
                    return builtActionPaths.All(file => File.Exists(file)) ? CachedEntryStatus.BuiltActionFile : CachedEntryStatus.UnbuiltActionFile;
                  }

            return CachedEntryStatus.Ignored;
        }
    }
}
