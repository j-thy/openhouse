# Run from the extracted Windows package (Install-Driver.bat / Uninstall-Driver.bat).
# Never replaces a running driver's DLL.
param(
    [ValidateSet('Install', 'Uninstall', 'Restore', 'Status')][string]$Action = 'Install',
    [string]$SteamVRPath = '',
    [string]$DriverPath = (Join-Path $PSScriptRoot 'driver_lighthouse.dll')
)
$ErrorActionPreference = 'Stop'
if ($Action -eq 'Restore') { $Action = 'Uninstall' }
if (-not $SteamVRPath) {
    $pathsFile = Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath'
    if (Test-Path $pathsFile) {
        foreach ($candidate in (Get-Content -Raw $pathsFile | ConvertFrom-Json).runtime) {
            if (Test-Path (Join-Path $candidate 'drivers\lighthouse\bin\win64')) {
                $SteamVRPath = $candidate; break
            }
        }
    }
    if (-not $SteamVRPath) {
        $steam = (Get-ItemProperty 'HKCU:\Software\Valve\Steam').SteamPath
        $SteamVRPath = Join-Path $steam 'steamapps\common\SteamVR'
    }
}
$directory = Join-Path $SteamVRPath 'drivers\lighthouse\bin\win64'
$target = Join-Path $directory 'driver_lighthouse.dll'
$backup = "$target.stock"
if (-not (Test-Path $target)) { throw "Driver not found at $target. Supply -SteamVRPath." }
if ($Action -eq 'Status') {
    Write-Output "Driver: $target"
    Write-Output "SHA256: $((Get-FileHash $target -Algorithm SHA256).Hash)"
    Write-Output "Stock backup present: $(Test-Path $backup)"
    exit 0
}
# Steam holds SteamVR's driver DLL open even when SteamVR itself is not running: with
# it up, replacing the file fails with "Cannot create a file when that file already
# exists". Check the whole chain, and name what to close.
$hosts = @(Get-Process -Name steam,vrserver,vrmonitor,vrcompositor,monado-service -ErrorAction SilentlyContinue)
if ($hosts) {
    throw "Stop $($hosts.ProcessName -join ', ') before changing the driver (Steam loads SteamVR's driver DLL too)."
}
# The update check (openhouse-check.exe): an app SteamVR starts with every session that
# warns when a SteamVR update replaced this driver. Registered in Steam's config:
# appconfig.json lists app manifests, vrappconfig\ holds auto-launch.
$checkKey = 'openhouse.check'
$checkDir = Join-Path $env:LOCALAPPDATA 'openhouse'
$checkManifest = Join-Path $checkDir 'openhouse-check.vrmanifest'
# Without a byte order mark (PowerShell 5's UTF8 writes one).
function Write-Text([string]$path, [string]$text) {
    [System.IO.File]::WriteAllText($path, $text + "`n", (New-Object System.Text.UTF8Encoding($false)))
}
function Get-SteamConfig {
    $pathsFile = Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath'
    if (Test-Path $pathsFile) {
        $config = @((Get-Content -Raw $pathsFile | ConvertFrom-Json).config) | Select-Object -First 1
        if ($config) { return $config }
    }
    return Join-Path (Split-Path (Split-Path (Split-Path $SteamVRPath))) 'config'
}
function Set-CheckRegistration([bool]$register) {
    $config = Get-SteamConfig
    $appconfig = Join-Path $config 'appconfig.json'
    $data = [ordered]@{}
    if (Test-Path $appconfig) {
        $parsed = Get-Content -Raw $appconfig | ConvertFrom-Json
        foreach ($p in $parsed.PSObject.Properties) { $data[$p.Name] = $p.Value }
    }
    $paths = @(@($data['manifest_paths']) | Where-Object { $_ -and ($_ -ne $checkManifest) })
    if ($register) { $paths += $checkManifest }
    $data['manifest_paths'] = $paths
    New-Item -ItemType Directory -Force -Path $config | Out-Null
    Write-Text $appconfig (ConvertTo-Json $data -Depth 10)
    $app = Join-Path $config "vrappconfig\$checkKey.vrappconfig"
    if ($register) {
        New-Item -ItemType Directory -Force -Path (Split-Path $app) | Out-Null
        Write-Text $app '{ "autolaunch" : true, "last_launch_time" : "0" }'
    } elseif (Test-Path $app) {
        Remove-Item -LiteralPath $app -Force
    }
}

# The openhouse page in SteamVR's settings: a settings schema in the lighthouse driver's
# resources (SteamVR has none there today; one it ships later is kept as .stock).
$schema = Join-Path (Split-Path (Split-Path $directory)) 'resources\settings\settingsschema.vrsettings'
function Install-Schema {
    $source = Join-Path $PSScriptRoot 'settingsschema.vrsettings'
    if (-not (Test-Path $source)) { return }
    New-Item -ItemType Directory -Force -Path (Split-Path $schema) | Out-Null
    if ((Test-Path $schema) -and -not (Select-String -Quiet -Path $schema -Pattern 'driver_openhouse')) {
        Copy-Item -LiteralPath $schema -Destination "$schema.stock" -Force
    }
    Copy-Item -LiteralPath $source -Destination $schema -Force
}
function Remove-Schema {
    if ((Test-Path $schema) -and (Select-String -Quiet -Path $schema -Pattern 'driver_openhouse')) { Remove-Item -LiteralPath $schema -Force }
    if (Test-Path "$schema.stock") { Move-Item -LiteralPath "$schema.stock" -Destination $schema -Force }
}

if ($Action -eq 'Uninstall') {
    # First, so the check goes even when there's no driver backup.
    Remove-Schema
    Set-CheckRegistration $false
    if (Test-Path $checkDir) { Remove-Item -LiteralPath $checkDir -Recurse -Force }
}
if ($Action -eq 'Install') {
    if (-not (Test-Path $DriverPath)) { throw "Package DLL missing: $DriverPath" }
    # The stock DLL is kept as driver_lighthouse.dll.stock: our driver loads it at run time
    # for headset displays. Back up whatever is installed unless it is ours (a SteamVR
    # update puts a newer stock DLL in place: that one replaces an older backup).
    $installed = [System.Text.Encoding]::ASCII.GetString([System.IO.File]::ReadAllBytes($target))
    $isOurs = $installed.Contains('liblh session started')
    if (-not $isOurs) {
        Copy-Item -LiteralPath $target -Destination $backup -Force
        Write-Output "Saved SteamVR's driver as $backup"
    } elseif (-not (Test-Path $backup)) {
        throw "The installed driver is already openhouse's and there is no stock backup ($backup). Verify SteamVR's files in Steam, then install again."
    }
    $source = $DriverPath
} else {
    if (-not (Test-Path $backup)) { throw "SteamVR's driver backup isn't there ($backup). Nothing to undo, or verify SteamVR's files in Steam." }
    Move-Item -LiteralPath $backup -Destination $target -Force
    Write-Output "SteamVR's own driver is back. Your room and settings in %APPDATA%\lh are kept."
    exit 0
}
Copy-Item -LiteralPath $source -Destination "$target.new" -Force
try {
    Move-Item -LiteralPath "$target.new" -Destination $target -Force
} catch {
    Remove-Item -LiteralPath "$target.new" -Force -ErrorAction SilentlyContinue
    throw "Could not replace $target - a process still has it open. Close Steam and SteamVR completely, then run Install-Driver.bat again. ($($_.Exception.Message))"
}
if ((Get-FileHash $target).Hash -ne (Get-FileHash $source).Hash) { throw 'Installed file hash mismatch' }
Install-Schema
$checkExe = Join-Path $PSScriptRoot 'apps\openhouse-check.exe'
if (Test-Path $checkExe) {
    New-Item -ItemType Directory -Force -Path $checkDir | Out-Null
    Copy-Item -LiteralPath $checkExe -Destination (Join-Path $checkDir 'openhouse-check.exe') -Force
    $manifest = [ordered]@{
        source = 'openhouse'
        applications = @([ordered]@{
            app_key = $checkKey
            launch_type = 'binary'
            binary_path_windows = (Join-Path $checkDir 'openhouse-check.exe')
            arguments = '"' + $directory + '"'
            is_dashboard_overlay = $true
            strings = @{ en_us = @{ name = 'openhouse update check'; description = 'Tells you when a SteamVR update replaced the openhouse driver' } }
        })
    }
    Write-Text $checkManifest (ConvertTo-Json $manifest -Depth 10)
    Set-CheckRegistration $true
}
Write-Output "Installed. Start SteamVR."
