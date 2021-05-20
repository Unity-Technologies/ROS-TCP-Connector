using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;

public class MessageBuilder: ScriptableObject
{
    public static void Build(string sourceFile, string targetFolder)
    {
        string exePath = AssetDatabase.GUIDToAssetPath("816eb56585d65134cb3ca3ed31c9fa9b");
        string exeName = System.IO.Path.GetFileName(exePath);

        System.Diagnostics.Process p = System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo()
        {
            FileName = System.IO.Path.Combine(Application.dataPath, "..", exePath),
            Arguments = sourceFile + " " + targetFolder,
            UseShellExecute = false,
            RedirectStandardError = true,
            RedirectStandardOutput = true
        });
        p.WaitForExit();

        while (!p.StandardError.EndOfStream)
        {
            Debug.LogError($"Error running {exeName}: " + p.StandardError.ReadLine());
        }

        string stdOut = p.StandardOutput.ReadToEnd();
        if(stdOut != "")
            Debug.Log($"Output from {exeName}: "+stdOut);

        p.Close();
        AssetDatabase.Refresh();
    }
}
