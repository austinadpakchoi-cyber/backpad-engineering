param(
    [Parameter(Mandatory=$true)][string]$SdkRoot,
    [Parameter(Mandatory=$true)][string]$ToolchainRoot,
    [Parameter(Mandatory=$true)][string]$SettingsFile,
    [Parameter(Mandatory=$true)][string]$BuildDir
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$SdkRoot = (Resolve-Path -LiteralPath $SdkRoot).Path
$ToolchainRoot = (Resolve-Path -LiteralPath $ToolchainRoot).Path
$SettingsFile = (Resolve-Path -LiteralPath $SettingsFile).Path
$BuildDir = [IO.Path]::GetFullPath($BuildDir)
if (Test-Path -LiteralPath $BuildDir) { throw 'Choose a new BuildDir; existing builds are not overwritten.' }
New-Item -ItemType Directory -Path $BuildDir | Out-Null
$env:ZEPHYR_BASE = Join-Path $SdkRoot 'zephyr'
$env:ZEPHYR_TOOLCHAIN_VARIANT = 'zephyr/gnu'
$env:ZEPHYR_SDK_INSTALL_DIR = Join-Path $ToolchainRoot 'opt\zephyr-sdk'
$env:PYTHONPATH = (Join-Path $ToolchainRoot 'opt\bin'),(Join-Path $ToolchainRoot 'opt\bin\Lib'),(Join-Path $ToolchainRoot 'opt\bin\Lib\site-packages') -join ';'
$toolPaths = '.', 'mingw64\bin', 'bin', 'opt\bin', 'opt\bin\Scripts', 'opt\nanopb\generator-bin', 'nrfutil\bin', 'opt\zephyr-sdk\gnu\arm-zephyr-eabi\bin' | ForEach-Object { Join-Path $ToolchainRoot $_ }
$env:PATH = ($toolPaths -join ';') + ';' + $env:PATH
$env:TEMP = Join-Path $BuildDir 'temp'
$env:TMP = $env:TEMP
$env:CCACHE_DIR = Join-Path $BuildDir 'cache\ccache'
$env:CCACHE_TEMPDIR = $env:TEMP
$env:PYTHONPYCACHEPREFIX = Join-Path $BuildDir 'cache\python'
New-Item -ItemType Directory -Path $env:TEMP -Force | Out-Null
$python = Join-Path $ToolchainRoot 'opt\bin\python.exe'
$generated = Join-Path $BuildDir 'generated'
& $python -B (Join-Path $PSScriptRoot 'source_id.py') --settings $SettingsFile --out $generated
if ($LASTEXITCODE -ne 0) { throw 'Build ID generation failed' }
Push-Location $SdkRoot
try {
    & $python -m west build --no-sysbuild -b backpad/nrf52840 -s (Join-Path $repo 'firmware\app') -d (Join-Path $BuildDir 'firmware') -- ('-DUSER_CACHE_DIR=' + (Join-Path $BuildDir 'cache')) ('-DBACKPAD_SENSOR_SETTINGS=' + $SettingsFile) ('-DBACKPAD_GENERATED_INCLUDE=' + $generated)
    if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed' }
} finally { Pop-Location }
