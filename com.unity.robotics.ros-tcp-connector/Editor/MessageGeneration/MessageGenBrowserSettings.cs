using System.IO;
using UnityEditor;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.Editor.MessageGeneration
{
    public class MessageGenBrowserSettings : ScriptableObject
    {
        public string inputPath;
        public string relativeOutPath;
        public string outputPath => Path.Combine(Application.dataPath, relativeOutPath);
        const string SETTINGS_FILENAME = "msgbrowser_settings.asset";

        static MessageGenBrowserSettings settings;
        public static MessageGenBrowserSettings Get()
        {
            if (settings == null)
            {
                settings = AssetDatabase.LoadAssetAtPath<MessageGenBrowserSettings>(Path.Combine("Assets", SETTINGS_FILENAME));
                if (settings == null)
                {
                    settings = ScriptableObject.CreateInstance<MessageGenBrowserSettings>();
                    settings.inputPath = "";
                    settings.relativeOutPath = "RosMessages";
                }
            }
            return settings;
        }

        public void Save()
        {
            if (!File.Exists(Path.Combine(Application.dataPath, SETTINGS_FILENAME)))
            {
                AssetDatabase.CreateAsset(this, Path.Combine("Assets", SETTINGS_FILENAME));
                AssetDatabase.SaveAssets();
            }
            else
            {
                EditorUtility.SetDirty(this);
            }
        }

        public static string ToRelativePath(string fullPath)
        {
            string dataPath = Path.GetFullPath(Application.dataPath);
            fullPath = Path.GetFullPath(fullPath);

            if (!fullPath.StartsWith(dataPath))
                return "";

            return fullPath.Substring(dataPath.Length + 1);
        }
    }
}
