using RosMessageGeneration;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.AssetImporters;
using UnityEngine;

[ScriptedImporter(1, "srv")]
public class ScriptedSrvImporter : ScriptedImporter
{
    public override void OnImportAsset(AssetImportContext ctx)
    {
        string inputPath = Path.Combine(Directory.GetCurrentDirectory(), ctx.assetPath);
        string outputPath = MessageGenBrowserSettings.Get().outputPath;
        ServiceAutoGen.GenerateSingleService(inputPath, MessageGenBrowserSettings.Get().outputPath);

        foreach (string builtPath in ServiceAutoGen.GetServiceClassPaths(inputPath, outputPath))
        {
            string builtAssetPath = Path.Combine("Assets", MessageGenBrowserSettings.ToRelativePath(builtPath));
            AssetDatabase.ImportAsset(builtAssetPath);
            Object messageClass = AssetDatabase.LoadAssetAtPath(builtAssetPath, typeof(MonoScript));
            if (messageClass != null)
                ctx.AddObjectToAsset("messageClass", messageClass);
        }
    }
}
