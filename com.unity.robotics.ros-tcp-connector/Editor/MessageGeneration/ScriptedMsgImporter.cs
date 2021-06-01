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
    [ScriptedImporter(1, "msg")]
    public class ScriptedMsgImporter : ScriptedImporter
    {
        public override void OnImportAsset(AssetImportContext ctx)
        {
            string inputPath = Path.Combine(Directory.GetCurrentDirectory(), ctx.assetPath);
            string outputPath = MessageGenBrowserSettings.Get().outputPath;
            MessageAutoGen.GenerateSingleMessage(inputPath, outputPath);

            string builtPath = MessageAutoGen.GetMessageClassPath(inputPath, outputPath);
            string builtAssetPath = Path.Combine("Assets", MessageGenBrowserSettings.ToRelativePath(builtPath));
            AssetDatabase.ImportAsset(builtAssetPath);
            Object messageClass = AssetDatabase.LoadAssetAtPath(builtAssetPath, typeof(MonoScript));
            if (messageClass != null)
                ctx.AddObjectToAsset("messageClass", messageClass);
        }
    }
}
