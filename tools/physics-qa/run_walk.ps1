# run_walk.ps1 - Windows port of tools/physics-qa/run_walk.sh
# One command: headless walk + hip/feet plot + objective gates.
#
#   .\tools\physics-qa\run_walk.ps1 [TAG] [KEY=VALUE ...]
#
#   TAG                 name for the artifacts (default: "walk")
#   KEY=VALUE           env overrides passed to the harness. Known keys the
#                       harness reads: LEG_COMPLIANCE, STEP_HEIGHT, STANCE,
#                       SWING, GAIT_AUTHORITY, ATT_LEAN_KP, ATT_LEAN_KD,
#                       SUBSTEPS, TARGET_SPEED, ...
#
# Defaults: LEG_COMPLIANCE=5e-9 STEP_HEIGHT=50, plans/walk_script.json.
# Artifacts land in tmp\headless\walks\:
#   <TAG>.csv            telemetry (104+ columns)
#   <TAG>.hips_feet.png  hip + feet x/y plot (plot_p3.py)
#   stdout summary       rig/leg verdicts + swing + foot-slide gates
#
# The character asset (examples/collision_detection/assets/character/
# v10.character.json) is read at startup, so collider edits need no rebuild.
# A Rust change does: rebuild first with
#   cargo build --release -p collision_detection --example headless_arm_pd

[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Rest
)

$ErrorActionPreference = 'Stop'

# --- locate repo root (tools/physics-qa -> repo root) -----------------------
$Repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $Repo

$Bin = Join-Path $Repo 'target\release\examples\headless_arm_pd.exe'
if (-not (Test-Path $Bin)) {
    Write-Error "missing $Bin - run: cargo build --release -p collision_detection --example headless_arm_pd"
    exit 1
}

# find a working python interpreter
$Py = $null
foreach ($cand in 'python', 'python3', 'py') {
    $g = Get-Command $cand -ErrorAction SilentlyContinue
    if ($g) {
        $ver = & $g.Path --version 2>&1
        if ("$ver" -match 'Python 3') { $Py = $g.Path; break }
    }
}
if (-not $Py) { Write-Error 'python 3 not found on PATH'; exit 1 }

# --- parse args: first non KEY=VALUE token is the tag, rest are env vars ----
$Tag = 'walk'
$EnvOver = @{}
foreach ($a in $Rest) {
    if ($a -match '=' -and $a -notmatch '/') {
        $k, $v = $a -split '=', 2
        $EnvOver[$k] = $v
    }
    else {
        $Tag = $a
    }
}

# --- output paths ------------------------------------------------------------
$OutDir = Join-Path $Repo 'tmp\headless\walks'
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$Csv = Join-Path $OutDir "$Tag.csv"
$Png = Join-Path $OutDir "$Tag.hips_feet.png"

# Snapshot the asset WITH the tag so each artifact is self-identifying. The
# asset is live-edited between runs, and the tag does not pin it.
Copy-Item (Join-Path $Repo 'examples\collision_detection\assets\character\v10.character.json') `
          (Join-Path $OutDir "$Tag.character.json")

# defaults (a KEY=VALUE passed above wins)
$EnvAll = @{ LEG_COMPLIANCE = '1e-8'; STEP_HEIGHT = '70' }
foreach ($k in $EnvOver.Keys) { $EnvAll[$k] = $EnvOver[$k] }

$shown = if ($EnvOver.Count) {
    ($EnvOver.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }) -join ' '
}
else { '<defaults: LEG_COMPLIANCE=1e-8 STEP_HEIGHT=70>' }
Write-Host "== run ${Tag}: $shown (asset snapshot: $OutDir\$Tag.character.json)"

# --- run the harness with the env vars set for the child process -------------
$fixed = @{
    ASSET_ROOT = (Join-Path $Repo 'examples\collision_detection\assets')
    SCRIPT     = (Join-Path $Repo 'plans\walk_script.json')
    OUT_CSV    = $Csv
}

$saved = @{}
function Set-TempEnv([hashtable]$vars) {
    foreach ($k in $vars.Keys) {
        $saved[$k] = (Get-Item "Env:$k" -ErrorAction SilentlyContinue).Value
        Set-Item "Env:$k" $vars[$k]
    }
}
function Restore-Env {
    foreach ($k in $saved.Keys) {
        if ($null -ne $saved[$k]) { Set-Item "Env:$k" $saved[$k] }
        else { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
    }
}

Set-TempEnv $fixed
Set-TempEnv $EnvAll
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = 'Continue'  # the harness may log to stderr
try {
    & $Bin 2>&1 | Select-String -Pattern 'body: rows|rigidity:|leg:|walk params' |
        ForEach-Object { $_.Line }
}
finally {
    $ErrorActionPreference = $prevEAP
    Restore-Env
    $saved.Clear()
}

# --- plot --------------------------------------------------------------------
# Native stderr must not throw under $ErrorActionPreference='Stop'
# (PowerShell wraps it in a terminating NativeCommandError).
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
Write-Host '== plot'
& $Py 'tools/physics-qa/plot_p3.py' $Csv $Png 2>$null | Select-Object -First 1

# --- gates -------------------------------------------------------------------
Write-Host '== gates'
& $Py 'tools/physics-qa/analyze_swing.py' $Csv --per-swing 2>$null | Select-Object -Last 4
& $Py 'tools/physics-qa/analyze_foot_slide.py' $Csv --skip-start 1.6 --skip-end 10.5 2>$null | Select-Object -Last 2
$ErrorActionPreference = $prevEAP

# --- artifacts ---------------------------------------------------------------
Write-Host "== artifacts: $Csv  $Png"
if (Test-Path $Png) { Invoke-Item $Png }
