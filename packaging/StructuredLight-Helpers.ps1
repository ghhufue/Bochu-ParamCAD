# Producer-side offline deployment. No pip, downloads or machine-wide installs.
function Assert-StructuredLightWheels([string]$CacheDirectory) {
    $lock = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'structured-light-wheels.json') -Raw | ConvertFrom-Json
    foreach ($package in $lock.packages) {
        $path = Join-Path $CacheDirectory $package.file
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing offline wheel: $path" }
        if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $package.sha256) {
            throw "Offline wheel checksum mismatch: $($package.file)"
        }
    }
    return $lock
}

function Install-StructuredLightWheels([string]$PythonRoot,[string]$CacheDirectory) {
    $lock = Assert-StructuredLightWheels $CacheDirectory
    $destination = Join-Path $PythonRoot 'site-packages'
    New-Item -ItemType Directory -Path $destination -Force | Out-Null
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    foreach ($package in $lock.packages) {
        $path = Join-Path $CacheDirectory $package.file
        $archive = [IO.Compression.ZipFile]::OpenRead($path)
        try {
            foreach ($entry in $archive.Entries) {
                $target = [IO.Path]::GetFullPath((Join-Path $destination $entry.FullName))
                Assert-ChildPath $destination $target
            }
        } finally { $archive.Dispose() }
        [IO.Compression.ZipFile]::ExtractToDirectory($path,$destination)
    }
    $pth = Get-ChildItem -LiteralPath $PythonRoot -Filter 'python*._pth' -File | Select-Object -First 1
    if ($null -eq $pth) { throw 'Embedded Python path configuration is missing' }
    $lines = @(Get-Content -LiteralPath $pth.FullName)
    if ($lines -notcontains 'site-packages') { $lines += 'site-packages' }
    Set-Content -LiteralPath $pth.FullName -Value $lines -Encoding ASCII
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'structured-light-wheels.json') -Destination $PythonRoot
}

function Assert-PythonNativeRuntime([string]$PythonRoot) {
    Initialize-PeReader
    $binaries = @(Get-ChildItem -LiteralPath $PythonRoot -Recurse -File | Where-Object { $_.Extension -in @('.pyd','.dll','.exe') })
    $names = @{}
    foreach ($binary in $binaries) { $names[$binary.Name.ToLowerInvariant()] = $binary.FullName }
    foreach ($binary in $binaries) {
        foreach ($name in [ParamCadPackaging.PeReader]::Imports($binary.FullName)) {
            if (-not (Test-SystemImport $name) -and -not $names.ContainsKey($name.ToLowerInvariant())) {
                throw "Missing Python native runtime $name (imported by $($binary.FullName))"
            }
        }
    }
    return $binaries.Count
}
