param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("build", "rebuild", "flash")]
    [string]$Action
)

$ErrorActionPreference = "Stop"

$uv4 = "D:\Keil_v5\UV4\UV4.exe"
$projectDir = Join-Path (Split-Path -Parent $PSScriptRoot) "Keil"
$project = Join-Path $projectDir "bldc.uvprojx"
$target = "FLASH"
$log = Join-Path $env:TEMP "et6039-keil-$Action.log"

if (-not (Test-Path -LiteralPath $uv4)) {
    throw "Keil executable not found: $uv4"
}

if (-not (Test-Path -LiteralPath $project)) {
    throw "Keil project not found: $project"
}

$mode = switch ($Action) {
    "build" { "-b" }
    "rebuild" { "-r" }
    "flash" { "-f" }
}

Remove-Item -LiteralPath $log -Force -ErrorAction SilentlyContinue

$process = Start-Process -FilePath $uv4 `
    -ArgumentList @($mode, $project, "-t", $target, "-j0", "-o", $log) `
    -WorkingDirectory $projectDir `
    -Wait `
    -PassThru

if (Test-Path -LiteralPath $log) {
    Get-Content -LiteralPath $log
}

if ($process.ExitCode -ge 2) {
    exit $process.ExitCode
}

exit 0
