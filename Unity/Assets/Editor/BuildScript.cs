using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

/// <summary>
/// Batch-сборка билда из командной строки:
///   Unity -batchmode -nographics -quit -projectPath Unity/ \
///         -executeMethod BuildScript.PerformBuild \
///         -buildOutput Build/GFSX_Simulator.app
/// </summary>
public static class BuildScript
{
    public static void PerformBuild()
    {
        string output = GetArg("-buildOutput") ?? "Build/GFSX_Simulator.app";

        // Активные сцены из Build Settings
        string[] scenes = System.Array.ConvertAll(
            EditorBuildSettings.scenes,
            s => s.path);

        if (scenes.Length == 0)
        {
            // Fallback: если в Build Settings пусто — берём SampleScene
            string fallback = "Assets/Scenes/SampleScene.unity";
            if (File.Exists(fallback)) scenes = new[] { fallback };
        }

        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output)));

        var options = new BuildPlayerOptions
        {
            scenes           = scenes,
            locationPathName = output,
            target           = EditorUserBuildSettings.activeBuildTarget,
            options          = BuildOptions.None // без Development
        };

        BuildReport report = BuildPipeline.BuildPlayer(options);
        BuildSummary s = report.summary;

        if (s.result == BuildResult.Succeeded)
        {
            Debug.Log($"[BuildScript] OK: {s.totalSize / (1024 * 1024)} MB → {output}");
            EditorApplication.Exit(0);
        }
        else
        {
            Debug.LogError($"[BuildScript] FAIL: {s.result}");
            EditorApplication.Exit(1);
        }
    }

    private static string GetArg(string name)
    {
        var args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length - 1; i++)
            if (args[i] == name) return args[i + 1];
        return null;
    }
}
