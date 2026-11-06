# run_walker_qa.ps1 - Windows port of tools/physics-qa/run_walker_qa.sh
# Build + run the walker_qa QA scene headless, plot the hip/feet trace from
# its telemetry CSV (same legacy format every harness writes, so plot_p3.py
# reads it unchanged), then print the standing summary.
#
#   .\tools\physics-qa\run_walker_qa.ps1 [TAG] [KEY=VALUE ...]
#
#   TAG                 artifact name under tmp\walker_qa\ (default: "run")
#   KEY=VALUE           env overrides forwarded to walker_qa. Known levers:
#                       WIN_HIDDEN (default 1 - no OS window; the whole
#                       pipeline still runs), ASSET_ROOT,
#                       SCRIPT=path.json - a command track that drives the
#                       walker and self-exits the app when the track ends.
#                       CSV_PATH is fixed by the binary.
#   RUN_SECS=N          hard backstop kill, seconds (default 600; also honors
#                       a pre-set $env:RUN_SECS). A scripted run exits on its
#                       own; RUN_SECS only guards a hung keyboard run.
#
# Artifacts:
#   tmp\walker_qa\<TAG>.csv            telemetry (legacy header: scalars + P-channels)
#   tmp\walker_qa\<TAG>.hips_feet.png  plot_p3.py output (hip + feet, gait bands)
#   tmp\walker_qa\<TAG>.log            app stdout/stderr
#   stdout summary                     grounded fraction + hip/feet settle table
#
# walker_qa writes its telemetry to tmp\walker_qa\walk.csv; this script moves
# it to <TAG>.csv. A Rust change is rebuilt first (cargo build -p walker_qa).

[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Rest
)

$ErrorActionPreference = 'Stop'

# Force UTF-8 console I/O so non-ASCII (e.g. a localized error or a Chinese
# path) does not render as mojibake through the cmd/VSCode pipe's GBK codepage.
# Restore the original codepage on exit.
$OldCp = (chcp) -replace '[^0-9]', ''
chcp 65001 > $null
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding $false
$OutputEncoding = [Console]::OutputEncoding

# A localized `Write-Error` record emits Chinese "所在位置 … 字符: …" stack
# decorations; report fatals as plain ASCII instead and exit.
function Fail([string]$msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }

# --- locate repo root (tools/physics-qa -> repo root) -----------------------
$Repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $Repo

# --- parse args: first non KEY=VALUE token is the tag ------------------------
$Tag = 'run'
$EnvOver = @{}
$RunSecs = if ($env:RUN_SECS) { [double]$env:RUN_SECS } else { 600 }
foreach ($a in $Rest) {
    if ($a -match '^[A-Za-z_][A-Za-z0-9_]*=') {
        $k, $v = $a -split '=', 2
        if ($k -eq 'RUN_SECS') { $RunSecs = [double]$v }
        else { $EnvOver[$k] = $v }
    }
    else {
        $Tag = $a
    }
}

# --- output paths -------------------------------------------------------------
$OutDir  = Join-Path $Repo 'tmp\walker_qa'
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$Csv     = Join-Path $OutDir "$Tag.csv"
$Png     = Join-Path $OutDir "$Tag.hips_feet.png"
$Log     = Join-Path $OutDir "$Tag.log"
$RawCsv  = Join-Path $OutDir 'walk.csv'

# find a WORKING python interpreter: the WindowsApps 'python.exe' is a store
# stub that prints a version but fails (exit 9009) on real execution, so
# validate with `-c pass` before accepting a candidate.
$Py = $null
foreach ($cand in 'python', 'python3', 'py') {
    $g = Get-Command $cand -ErrorAction SilentlyContinue
    if (-not $g) { continue }
    $prevEAP = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $ver = & $g.Path --version 2>&1
    & $g.Path -c 'pass' 2>$null
    $ok = ($LASTEXITCODE -eq 0) -and ("$ver" -match 'Python 3')
    $ErrorActionPreference = $prevEAP
    if ($ok) { $Py = $g.Path; break }
}
if (-not $Py) { Fail 'python 3 not found on PATH' }

# --- build ---------------------------------------------------------------------
Write-Host '== build walker_qa'
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = 'Continue'  # cargo warns on stderr; must not throw
try {
    & cargo build -p walker_qa 2>&1 | Select-Object -Last 1
}
finally { $ErrorActionPreference = $prevEAP }
if ($LASTEXITCODE -ne 0) { Fail 'cargo build -p walker_qa failed' }
$Bin = Join-Path $Repo 'target\debug\walker_qa.exe'
if (-not (Test-Path $Bin)) { Fail "missing $Bin" }

# --- env for the child process --------------------------------------------------
# WIN_HIDDEN defaults to 1; a WIN_HIDDEN=... override wins (later assignment).
# ASSET_ROOT must be pinned on Windows: walker_qa's fallback is a hardcoded
# /home/ubuntu path (see run_walk.ps1 for the same idiom).
$EnvSet = @{
    WIN_HIDDEN = '1'
    ASSET_ROOT = (Join-Path $Repo 'examples\collision_detection\assets')
}
foreach ($k in $EnvOver.Keys) { $EnvSet[$k] = $EnvOver[$k] }

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

$shown = if ($EnvOver.Count) {
    ($EnvOver.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }) -join ' '
}
else { '<defaults>' }
Write-Host "== run: $Tag ($shown) - backstop ${RunSecs}s"

if (Test-Path $RawCsv) { Remove-Item $RawCsv -Force }

# Run the binary, capture combined stdout+stderr to the log. A SCRIPT track
# self-exits the app when it ends; RUN_SECS is only the backstop kill for a
# hung keyboard run (matching the bash `timeout`).
$prevEAP = $ErrorActionPreference
$ErrorActionPreference = 'Continue'  # native stderr must not throw under Stop
try {
    Set-TempEnv $EnvSet

    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $Bin
    $psi.WorkingDirectory = $Repo
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $proc = [System.Diagnostics.Process]::Start($psi)

    # Read both streams async: a blocking-only-stdout read can deadlock once
    # the stderr pipe buffer fills.
    $outTask = $proc.StandardOutput.ReadToEndAsync()
    $errTask = $proc.StandardError.ReadToEndAsync()
    if (-not $proc.WaitForExit([int]($RunSecs * 1000))) {
        try { $proc.Kill($true) } catch { $proc.Kill() }  # +children, if supported
        $proc.WaitForExit()
    }
    # Bounded stream drain: killed grandchildren can inherit the stdout pipe
    # and keep EOF from arriving; never hang on a finished run.
    $outTxt = ''; $errTxt = ''
    if ($outTask.Wait(3000)) { $outTxt = $outTask.Result }
    if ($errTask.Wait(3000)) { $errTxt = $errTask.Result }
    $combined = @($outTxt -split "`r?`n") + @($errTxt -split "`r?`n")
    ($combined | Where-Object { $_ -ne '' }) | Set-Content -Path $Log -Encoding UTF8
}
finally {
    Restore-Env
    $ErrorActionPreference = $prevEAP
}

if (-not (Test-Path $RawCsv)) {
    Write-Host "no telemetry written: $RawCsv (see $Log)"
    Get-Content $Log -Tail 5
    exit 1
}
# Tag 'walk' names the tag file onto the raw one: move would be same-file and
# die. Skip it when source and destination resolve to the same path.
if ((Resolve-Path $RawCsv).Path -ne [System.IO.Path]::GetFullPath($Csv)) {
    Move-Item $RawCsv $Csv -Force
}
Select-String -Path $Log -Pattern 'attached|floor|wrote' |
    Select-Object -Last 3 | ForEach-Object { $_.Line }

# --- plot -----------------------------------------------------------------------
Write-Host '== plot'
& $Py 'tools/physics-qa/plot_p3.py' $Csv $Png 2>$null | Select-Object -First 1

# --- standing summary --------------------------------------------------------------
Write-Host '== standing summary'
$Summary = @'
import csv, statistics, sys
rows = list(csv.DictReader(open(sys.argv[1])))
def num(r, k): return float(r[k])
if not rows:
    sys.exit("empty csv")
tail = rows[-90:]
print(f"t={num(rows[-1],'t'):.1f}s mode={rows[-1]['mode']} "
      f"grounded={sum(int(r['grounded']) for r in rows)/len(rows):.2f}")
print(f"hip y last-1s mean {statistics.mean(num(r,'P3y') for r in tail):.1f} "
      f"(physics y-down; floor surface 312, feet at ~305-310 when standing)")
print(f"feet y last-1s mean {statistics.mean((num(r,'PLLLy')+num(r,'PRLLy'))/2 for r in tail):.1f}")
print(f"hip x last-1s drift: {min(num(r,'P3x') for r in tail):.1f} .. {max(num(r,'P3x') for r in tail):.1f}")
print(f"v_filt last-1s mean {statistics.mean(abs(num(r,'v_filt')) for r in tail):.1f} px/s")
'@
# ASCII-encode the piped script so a non-UTF8 console codepage cannot mangle it.
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$Summary | & $Py - $Csv

Write-Host "== artifacts: $Csv  $Png"

# --- restore console codepage --------------------------------------------------
if ($OldCp) { chcp $OldCp > $null }
