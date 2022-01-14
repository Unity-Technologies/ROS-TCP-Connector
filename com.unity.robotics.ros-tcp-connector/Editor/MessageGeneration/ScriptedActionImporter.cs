using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;
#if UNITY_2020_2_OR_NEWER
using UnityEditor.AssetImporters;
#else
using UnityEditor.Experimental.AssetImporters;
#endif

namespace Unity.Robotics.ROSTCPConnector.Editor.MessageGeneration
{
    [ScriptedImporter(1, "action")]
    public class ScriptedActionImporter : ScriptedImporter
    {
        public override void OnImportAsset(AssetImportContext ctx)
        {
            var inputPath = Path.Combine(Directory.GetCurrentDirectory(), ctx.assetPath);
            var outputPath = MessageGenBrowserSettings.Get().outputPath;
            ActionAutoGen.GenerateSingleAction(inputPath, MessageGenBrowserSettings.Get().outputPath);

            foreach (string builtPath in ServiceAutoGen.GetServiceClassPaths(inputPath, outputPath))
            {
                var builtAssetPath = Path.Combine("Assets", MessageGenBrowserSettings.ToRelativePath(builtPath));
                AssetDatabase.ImportAsset(builtAssetPath);
                var messageClass = AssetDatabase.LoadAssetAtPath(builtAssetPath, typeof(MonoScript));
                if (messageClass != null)
                    ctx.AddObjectToAsset("messageClass", messageClass);
            }
        }
    }
}
