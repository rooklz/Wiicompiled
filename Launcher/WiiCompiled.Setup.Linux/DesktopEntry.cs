namespace WiiCompiled.Setup.Linux;

/// <summary>
/// freedesktop.org .desktop application-menu entries. Replaces ShellIntegration.cs's registry
/// uninstall entry (no Linux analogue for an unpackaged tool - Windows already skips that step for
/// portable installs, this just applies that same behavior universally) and .lnk shortcuts.
/// </summary>
internal static class DesktopEntry
{
    private static string ApplicationsDirectory =>
        Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "applications");

    private static string PathFor(string profile) =>
        Path.Combine(ApplicationsDirectory, $"wiicompiled-{profile}.desktop");

    public static void Create(string profile, string displayName, string toolDllPath)
    {
        // No --install-dir argument: launch-base/launch-retro already resolve everything from
        // install-state.json, so there is nothing for this shortcut to pass beyond which profile.
        Directory.CreateDirectory(ApplicationsDirectory);
        var launchArg = profile == "retro-rewind" ? "launch-retro" : "launch-base";
        var contents =
            "[Desktop Entry]\n" +
            "Type=Application\n" +
            $"Name={displayName}\n" +
            $"Exec=dotnet \"{toolDllPath}\" {launchArg}\n" +
            "Categories=Game;\n" +
            "Terminal=false\n";
        File.WriteAllText(PathFor(profile), contents);
    }

    public static void Remove(string profile)
    {
        var path = PathFor(profile);
        if (File.Exists(path)) File.Delete(path);
    }
}
