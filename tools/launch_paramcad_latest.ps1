param(
    [switch]$ResolveOnly,
    [string]$RepositoryRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'

function Get-ParamCadCandidates {
    param([string]$Root)
    $buildRoots = New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
    Get-ChildItem -LiteralPath (Join-Path $Root 'artifacts') -Directory -ErrorAction SilentlyContinue |
        Where-Object {
            -not ($_.Attributes -band [IO.FileAttributes]::ReparsePoint) -and
                ($_.Name -like 'build*' -or (Test-Path -LiteralPath (Join-Path $_.FullName 'CMakeCache.txt') -PathType Leaf))
        } |
        ForEach-Object { [void]$buildRoots.Add($_.FullName) }
    $appBuild = Join-Path $Root 'apps\ParamCAD Studio\build'
    if (Test-Path -LiteralPath $appBuild -PathType Container) {
        Get-ChildItem -LiteralPath $appBuild -Directory -ErrorAction SilentlyContinue |
            Where-Object { -not ($_.Attributes -band [IO.FileAttributes]::ReparsePoint) } |
            ForEach-Object {
                [void]$buildRoots.Add($_.FullName)
                Get-ChildItem -LiteralPath $_.FullName -Directory -ErrorAction SilentlyContinue |
                    Where-Object { -not ($_.Attributes -band [IO.FileAttributes]::ReparsePoint) } |
                    ForEach-Object { [void]$buildRoots.Add($_.FullName) }
            }
    }

    $runtimeRoots = New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
    foreach ($build in $buildRoots) {
        [void]$runtimeRoots.Add((Join-Path $build 'Release'))
        $cache = Join-Path $build 'CMakeCache.txt'
        if (Test-Path -LiteralPath $cache -PathType Leaf) {
            Select-String -LiteralPath $cache -Pattern '^CMAKE_RUNTIME_OUTPUT_DIRECTORY(?:_RELEASE)?:[^=]*=(.+)$' |
                ForEach-Object {
                    $runtime = $_.Matches[0].Groups[1].Value.Trim().Trim('"')
                    if (-not [IO.Path]::IsPathRooted($runtime)) { $runtime = Join-Path $build $runtime }
                    [void]$runtimeRoots.Add($runtime)
                    [void]$runtimeRoots.Add((Join-Path $runtime 'Release'))
                }
        }
    }

    # Search only runtime outputs with bounded depth, never dependency/source trees.
    $visited = New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
    foreach ($runtime in $runtimeRoots) {
        $pending = New-Object 'System.Collections.Generic.Queue[object]'
        $pending.Enqueue([pscustomobject]@{ Path=$runtime; Depth=0 })
        while ($pending.Count -gt 0) {
            $directory = $pending.Dequeue()
            if (-not $visited.Add($directory.Path)) { continue }
            if (-not (Test-Path -LiteralPath $directory.Path -PathType Container)) { continue }
            $directoryInfo = Get-Item -LiteralPath $directory.Path
            if ($directoryInfo.Attributes -band [IO.FileAttributes]::ReparsePoint) { continue }
            $candidate = Join-Path $directory.Path 'ParamCAD Studio.exe'
            if (Test-Path -LiteralPath $candidate -PathType Leaf) { Get-Item -LiteralPath $candidate }
            if ($directory.Depth -ge 3) { continue }
            Get-ChildItem -LiteralPath $directory.Path -Directory -ErrorAction SilentlyContinue |
                Where-Object { -not ($_.Attributes -band [IO.FileAttributes]::ReparsePoint) } |
                ForEach-Object { $pending.Enqueue([pscustomobject]@{ Path=$_.FullName; Depth=$directory.Depth+1 }) }
        }
    }
}

function Get-ParamCadLaunchPlan {
    param([IO.FileInfo]$Newest, [object[]]$Processes)
    foreach ($process in $Processes) {
        try {
            # Reuse only this build, including checking for a replacement at the same path.
            if ($process.Path -ieq $Newest.FullName -and
                $process.MainWindowHandle -ne [IntPtr]::Zero -and
                $process.StartTime.ToUniversalTime() -ge $Newest.LastWriteTimeUtc) {
                return [pscustomobject]@{ Action='Activate'; Executable=$Newest.FullName; ProcessId=$process.Id; WindowHandle=$process.MainWindowHandle }
            }
        } catch { continue }
    }
    return [pscustomobject]@{ Action='Launch'; Executable=$Newest.FullName; ProcessId=$null; WindowHandle=$null }
}

# Allow function tests to dot-source this file without launching anything.
if ($MyInvocation.InvocationName -eq '.') { return }

$launchMutex = $null
$ownsMutex = $false
try {
    if (-not $ResolveOnly) {
        $launchMutex = New-Object System.Threading.Mutex($false, 'Local\BochuParamCADLatestLauncher')
        try { $ownsMutex = $launchMutex.WaitOne(10000) }
        catch [System.Threading.AbandonedMutexException] { $ownsMutex = $true }
        if (-not $ownsMutex) { exit 0 }
    }

    $newest = Get-ParamCadCandidates -Root $RepositoryRoot |
        Sort-Object @{Expression='LastWriteTimeUtc';Descending=$true}, @{Expression='FullName';Descending=$false} |
        Select-Object -First 1
    if ($null -eq $newest) {
        if ($ResolveOnly) { throw "No ParamCAD Studio release build was found under $RepositoryRoot." }
        Add-Type -AssemblyName System.Windows.Forms
        [System.Windows.Forms.MessageBox]::Show("No ParamCAD Studio release build was found under $RepositoryRoot.",
            'ParamCAD Studio', 'OK', 'Warning') | Out-Null
        exit 1
    }

    $processes = @(Get-Process -Name 'ParamCAD Studio' -ErrorAction SilentlyContinue)
    $plan = Get-ParamCadLaunchPlan -Newest $newest -Processes $processes
    if ($ResolveOnly) {
        [pscustomobject]@{ Action=$plan.Action; Executable=$plan.Executable; LastWriteTimeUtc=$newest.LastWriteTimeUtc; ReuseProcessId=$plan.ProcessId } |
            ConvertTo-Json -Compress
        exit 0
    }

    if ($plan.Action -eq 'Activate') {
        Add-Type @"
using System;
using System.Runtime.InteropServices;
public class ParamCadWindow {
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int n);
}
"@
        [ParamCadWindow]::ShowWindow($plan.WindowHandle, 9) | Out-Null
        [ParamCadWindow]::SetForegroundWindow($plan.WindowHandle) | Out-Null
        exit 0
    }

    $env:PYTHONPATH = (Join-Path $RepositoryRoot 'core\reconstruction') + ';' +
        (Join-Path $RepositoryRoot 'artifacts\structured-light-python')
    $started = Start-Process -FilePath $plan.Executable -WorkingDirectory $newest.DirectoryName -PassThru
    # Serialize rapid double-clicks until the new process has a window.
    $deadline = [DateTime]::UtcNow.AddSeconds(5)
    while ([DateTime]::UtcNow -lt $deadline -and -not $started.HasExited) {
        $started.Refresh()
        if ($started.MainWindowHandle -ne [IntPtr]::Zero) { break }
        Start-Sleep -Milliseconds 100
    }
} finally {
    if ($ownsMutex) { $launchMutex.ReleaseMutex() }
    if ($null -ne $launchMutex) { $launchMutex.Dispose() }
}
