# run_visual.ps1 - run the RENDERED harness (visual_arm_pd) + the same hip/feet
# plot as run_walk.ps1. The visual bin is the rendered twin of headless_arm_pd:
# same script driver, same 90 Hz XPBD physics, same telemetry CSV — but booted
# under DefaultPlugins + VelloPlugin, so a window opens while it walks and the
# run self-exits (AppExit) when the script ends.
#
#   .\tools\physics-qa\run_visual.ps1 [TAG] [KEY=VALUE ...]
#
#   TAG                 name for the artifacts (default: "visual")
#   KEY=VALUE           env overrides forwarded to the harness (TARGET_SPEED,
#                       LEG_COMPLIANCE, STEP_HEIGHT, ATT_LEAN_KP, ...).
#
# Defaults mirror the tuned visual invocation from visual.rs' header comment:
#   LEG_COMPLIANCE=1e-8 STEP_HEIGHT=70 COLLISION=1 WIN_HIDDEN=1
# (WIN_HIDDEN=0 pops the window so you can watch; the run still self-exits.)
# (run_walk.ps1 headless defaults are LEG_COMPLIANCE=5e-9 STEP_HEIGHT=50 — pass
#  KEY=VALUEs explicitly if you want a like-for-like A/B against a headless tag.)
#
# Artifacts land in tmp\headless\walks\:
#   <TAG>.visual.csv           telemetry (same 104+ columns as the headless CSV)
#   <TAG>.visual.hips_feet.png hip + feet x/y plot (plot_p3.py)
#   <TAG>.visual.character.json asset snapshot (self-identifying artifacts)

[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Rest
)

$ErrorActionPreference = 'Stop'

# --- locate repo root (tools/physics-qa -> repo root) -----------------------
$Repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $Repo

$Bin = Join-Path $Repo 'target\release\examples\visual_arm_pd.exe'
if (-not (Test-Path $Bin)) {
    Write-Error "missing $Bin - run: cargo build --release -p collision_detection --example visual_arm_pd"
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
$Tag = 'visual'
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
$Csv = Join-Path $OutDir "$Tag.visual.csv"
$Png = Join-Path $OutDir "$Tag.visual.hips_feet.png"

# Snapshot the asset WITH the tag so each artifact is self-identifying.
Copy-Item (Join-Path $Repo 'examples\collision_detection\assets\character\v10.character.json') `
          (Join-Path $OutDir "$Tag.visual.character.json")

# defaults (a KEY=VALUE passed above wins). WIN_HIDDEN=1 runs the full visual
# pipeline with no OS window; pass WIN_HIDDEN=0 to watch the character walk.
$EnvAll = @{ LEG_COMPLIANCE = '1e-8'; STEP_HEIGHT = '70'; COLLISION = '1'; WIN_HIDDEN = '1' }
foreach ($k in $EnvOver.Keys) { $EnvAll[$k] = $EnvOver[$k] }

$shown = if ($EnvOver.Count) {
    ($EnvOver.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }) -join ' '
}
else { '<defaults: LEG_COMPLIANCE=1e-8 STEP_HEIGHT=70 COLLISION=1 WIN_HIDDEN=1>' }
Write-Host "== run $Tag (visual): $shown (self-exits at script end; WIN_HIDDEN=0 to watch)"

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
    & $Bin 2>&1 | Select-String -Pattern 'body: rows|rigidity:|leg:|walk params|visual harness' |
        ForEach-Object { $_.Line }
}
finally {
    $ErrorActionPreference = $prevEAP
    Restore-Env
    $saved.Clear()
}

if (-not (Test-Path $Csv)) {
    Write-Error "no telemetry written: $Csv (the harness exited before finalize?)"
    exit 1
}

# --- plot --------------------------------------------------------------------
# Native stderr must not throw under $ErrorActionPreference='Stop'.
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
Write-Host '== plot'
& $Py 'tools/physics-qa/plot_p3.py' $Csv $Png 2>$null | Select-Object -First 1

Write-Host '== gates'
& $Py 'tools/physics-qa/analyze_swing.py' $Csv --per-swing 2>$null | Select-Object -Last 4
& $Py 'tools/physics-qa/analyze_foot_slide.py' $Csv --skip-start 1.6 --skip-end 10.5 2>$null | Select-Object -Last 2
$ErrorActionPreference = $prevEAP

# --- artifacts ---------------------------------------------------------------
Write-Host "== artifacts: $Csv  $Png"
if (Test-Path $Png) { Invoke-Item $Png }
