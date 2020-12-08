using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace RosMessageGeneration
{
    public class MessageGenBrowser : EditorWindow
    {
        // we actually only want the hashset, but Unity won't serialize that - so we keep foldedOutList as a backup.
        private HashSet<string> foldedOutHash;

        [SerializeField]
        private List<string> foldedOutList = new List<string> { };
        [SerializeField]
        private Vector2 scrollPos;

        const int BUTTON_WIDTH = 100;

        bool isCacheDirty = true;
        CachedEntry cacheRoot;

        enum CachedEntryStatus
        {
            NormalFolder,
            MsgFolder,
            IgnoredFile,
            UnbuiltMsgFile,
            BuiltMsgFile,
            UnbuiltSrvFile,
            BuiltSrvFile,
        }

        struct CachedEntry
        {
            public string path;
            public string label => Path.GetFileName(path);
            public CachedEntryStatus status;
            public List<CachedEntry> contents;
            public string numMsgs;
        }

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
            MessageGenBrowserSettings settings = MessageGenBrowserSettings.Get();
            string inPath = settings.inputPath;
            inPath = EditorGUILayout.TextField("Input Path", inPath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(150)))
            {
                inPath = EditorUtility.OpenFolderPanel("Select Folder...", inPath, "");
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            string relativeOutPath = settings.relativeOutPath;
            relativeOutPath = EditorGUILayout.TextField("Output Location", relativeOutPath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(150)))
            {
                string absOutPath = EditorUtility.OpenFolderPanel("Select Folder...", settings.outputPath, "");
                if(absOutPath != "")
                    relativeOutPath = MessageGenBrowserSettings.ToRelativePath(absOutPath);
            }
            EditorGUILayout.EndHorizontal();

            if (!Directory.Exists(inPath))
            {
                if (GUILayout.Button("Select a folder", GUILayout.Width(200)))
                {
                    inPath = EditorUtility.OpenFolderPanel("Select Folder...", inPath, "");
                }
            }
            else
            {
                EditorGUILayout.LabelField(inPath+":");
                scrollPos = EditorGUILayout.BeginScrollView(scrollPos);
                if (isCacheDirty || cacheRoot.path == null)
                    RefreshCache(inPath);

                ShowCachedEntry(cacheRoot);
                EditorGUILayout.EndScrollView();
            }

            if(inPath != settings.inputPath || relativeOutPath != settings.relativeOutPath)
            {
                if(inPath != "")
                    settings.inputPath = inPath;
                settings.relativeOutPath = relativeOutPath;
                settings.Save();
                isCacheDirty = true;
            }
        }

        void ShowCachedEntry(CachedEntry entry)
        {
            bool isFolder = false;
            string buildLabel = null;
            switch (entry.status)
            {
                case CachedEntryStatus.IgnoredFile:
                    return;
                case CachedEntryStatus.BuiltMsgFile:
                    buildLabel = "Rebuild msg";
                    break;
                case CachedEntryStatus.BuiltSrvFile:
                    buildLabel = "Rebuild srv";
                    break;
                case CachedEntryStatus.UnbuiltMsgFile:
                    buildLabel = "Build msg";
                    break;
                case CachedEntryStatus.UnbuiltSrvFile:
                    buildLabel = "Build srv";
                    break;
                case CachedEntryStatus.NormalFolder:
                case CachedEntryStatus.MsgFolder:
                    isFolder = true;
                    buildLabel = (entry.numMsgs != null) ? "Build " + entry.numMsgs : null;
                    break;
            }

            if(!isFolder)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField(entry.label);

                if(buildLabel != null)
                {
                    if (GUILayout.Button(buildLabel, GUILayout.Width(BUTTON_WIDTH)))
                    {
                        // build this msg/srv file
                        switch(entry.status)
                        {
                            case CachedEntryStatus.BuiltMsgFile:
                            case CachedEntryStatus.UnbuiltMsgFile:
                                MessageAutoGen.GenerateSingleMessage(entry.path, MessageGenBrowserSettings.Get().outputPath);
                                break;
                            case CachedEntryStatus.BuiltSrvFile:
                            case CachedEntryStatus.UnbuiltSrvFile:
                                ServiceAutoGen.GenerateSingleService(entry.path, MessageGenBrowserSettings.Get().outputPath);
                                break;
                        }
                        AssetDatabase.Refresh();
                        isCacheDirty = true;
                    }
                }
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                bool isFoldedOut = foldedOutHash.Contains(entry.path);
                EditorGUILayout.BeginHorizontal();
                bool shouldBeFoldedOut = EditorGUILayout.Foldout(isFoldedOut, entry.label, true, EditorStyles.foldout);
                if (buildLabel != null && GUILayout.Button(buildLabel, GUILayout.Width(BUTTON_WIDTH)))
                {
                    // build this directory
                    MessageAutoGen.GenerateDirectoryMessages(entry.path, MessageGenBrowserSettings.Get().outputPath);
                    ServiceAutoGen.GenerateDirectoryServices(entry.path, MessageGenBrowserSettings.Get().outputPath);
                    AssetDatabase.Refresh();
                    isCacheDirty = true;
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

                if (shouldBeFoldedOut && !isFoldedOut)
                {
                    isCacheDirty = true;
                    foldedOutList.Add(entry.path);
                    foldedOutHash.Add(entry.path);
                }
                else if (!shouldBeFoldedOut && isFoldedOut)
                {
                    isCacheDirty = true;
                    foldedOutList.Remove(entry.path);
                    foldedOutHash.Remove(entry.path);
                }
            }
        }

        void RefreshCache(string inPath)
        {
            cacheRoot = CacheFolder(inPath);
            isCacheDirty = false;
        }

        CachedEntry CacheFolder(string path)
        {
            if (foldedOutHash == null)
            {
                foldedOutHash = new HashSet<string>();
                foreach (string s in foldedOutList)
                    foldedOutHash.Add(s);
            }

            List<CachedEntry> contents = new List<CachedEntry>();

            bool isExpanded = foldedOutHash.Contains(path);
            if(isExpanded)
            {
                foreach (string folder in Directory.EnumerateDirectories(path))
                {
                    contents.Add(CacheFolder(folder));
                }
            }

            int numMsgs = 0;
            int numSrvs = 0;
            foreach (string file in Directory.EnumerateFiles(path))
            {
                CachedEntryStatus status = GetFileStatus(file);
                if (status == CachedEntryStatus.IgnoredFile)
                    continue;

                if (status == CachedEntryStatus.BuiltSrvFile || status == CachedEntryStatus.UnbuiltSrvFile)
                    numSrvs++;
                else
                    numMsgs++;

                if(isExpanded)
                    contents.Add(new CachedEntry()
                    {
                        path = file,
                        status = status
                    });
            }

            string numMsgsLabel = "";
            if (numMsgs > 0 && numSrvs > 0)
                numMsgsLabel = numMsgs + " msg"+(numMsgs>1?"s":"")+", " + numSrvs + " srv" + (numSrvs > 1 ? "s" : "");
            else if (numMsgs > 0)
                numMsgsLabel = numMsgs + " msg"+ (numMsgs > 1 ? "s" : "");
            else if (numSrvs > 0)
                numMsgsLabel = numSrvs + " srv"+ (numSrvs > 1 ? "s" : "");
            else
                numMsgsLabel = "";

            return new CachedEntry()
            {
                path = path,
                contents = contents,
                status = CachedEntryStatus.NormalFolder,
                numMsgs = numMsgsLabel,
            };
        }

        CachedEntryStatus GetFileStatus(string path)
        {
            switch (Path.GetExtension(path))
            {
                case ".msg":
                    string builtPath = MessageAutoGen.GetMessageClassPath(path, MessageGenBrowserSettings.Get().outputPath);
                    return File.Exists(builtPath) ? CachedEntryStatus.BuiltMsgFile : CachedEntryStatus.UnbuiltMsgFile;
                case ".srv":
                    string[] builtPaths = ServiceAutoGen.GetServiceClassPaths(path, MessageGenBrowserSettings.Get().outputPath);
                    return builtPaths.All(file => File.Exists(file)) ? CachedEntryStatus.BuiltSrvFile : CachedEntryStatus.UnbuiltSrvFile;
            }

            return CachedEntryStatus.IgnoredFile;
        }
    }
}
