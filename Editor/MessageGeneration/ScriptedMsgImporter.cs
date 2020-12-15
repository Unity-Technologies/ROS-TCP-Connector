using RosMessageGeneration;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.AssetImporters;
using UnityEngine;

[ScriptedImporter(1, "msg")]
public class ScriptedMsgImporter : ScriptedImporter
{
    public override void OnImportAsset(AssetImportContext ctx)
    {
        string inputPath =  Path.Combine(Directory.GetCurrentDirectory(), ctx.assetPath);
        string outputPath = MessageGenBrowserSettings.Get().outputPath;
        MessageAutoGen.GenerateSingleMessage(inputPath, outputPath);
        
        string builtPath = MessageAutoGen.GetMessageClassPath(inputPath, outputPath);
        string builtAssetPath = Path.Combine("Assets", MessageGenBrowserSettings.ToRelativePath(builtPath));
        AssetDatabase.ImportAsset(builtAssetPath);
        Object messageClass = AssetDatabase.LoadAssetAtPath(builtAssetPath, typeof(MonoScript));
        if(messageClass != null)
            ctx.AddObjectToAsset("messageClass", messageClass);
    }
}
