using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;

public class MessageBuilder: ScriptableObject
{
    [UnityEditor.MenuItem("Robotics/Test Generator")]
    static void Test()
    {
        Build(@"C:\Users\laurie.cheers\Documents\ROS2Messages\common_interfaces-master\geometry_msgs\msg\Vector3.msg", Application.dataPath);
    }

    public static void Build(string sourceFile, string targetFolder)
    {
        string exePath = AssetDatabase.GUIDToAssetPath("816eb56585d65134cb3ca3ed31c9fa9b");

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
            Debug.LogError("Error running MessageGenerator.exe: "+p.StandardError.ReadLine());

        string stdOut = p.StandardOutput.ReadToEnd();
        if(stdOut != "")
            Debug.Log("Output from MessageGenerator.exe: "+stdOut);

        p.Close();
        UnityEditor.AssetDatabase.Refresh();
    }
}
