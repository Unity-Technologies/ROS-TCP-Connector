using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using System.Runtime.InteropServices;

public class MessageBuilder: ScriptableObject
{
    public static void Build(string sourceFile, string targetFolder)
    {
#if UNITY_EDITOR_OSX
        string exePath = AssetDatabase.GUIDToAssetPath("20189e360b49da041b86e48c3eb6b6af");
#elif UNITY_EDITOR_LINUX
        string exePath = AssetDatabase.GUIDToAssetPath("f6548c1741bccda428981a0a40a6e275");
#else
        //        string exePath = AssetDatabase.GUIDToAssetPath("0c9b6f69171169246bf168b506904b8f");
        string exePath = AssetDatabase.GUIDToAssetPath("0b30214e32bb6e54c8dd1a4aea599867");
#endif
        string exeName = System.IO.Path.GetFileName(exePath);

        System.Diagnostics.Process p = System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo()
        {
            FileName = System.IO.Path.GetFullPath(exePath),
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
