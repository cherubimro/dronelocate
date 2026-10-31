<#
.SYNOPSIS
  Bring up the whole site on native Windows: supernode, sensor nodes, scene.

.DESCRIPTION
  The PowerShell equivalent of run_demo.sh, for people who cannot or will not
  use WSL2. Prefer WSL2 or Docker if either is available -- they run the bash
  script unmodified, so they cannot drift from it, while this file can.

  Note there is no CONFIG= prefix here. That is bash syntax: in PowerShell it
  is not an environment assignment and silently does nothing useful. Use
  -Config instead.

.EXAMPLE
  .\run_demo.ps1 -Config config\site-smooth.json
  .\run_demo.ps1 -Config config\site-scan-lite.json
  .\run_demo.ps1 -Hw n10 -HwSource uhd
#>
param(
  [string]$Config   = "config/site.json",
  [string]$LogDir   = "logs",
  [string]$Python   = "",
  [string]$Hw       = "",
  [ValidateSet("rtlsdr","uhd")]
  [string]$HwSource = "rtlsdr",
  [switch]$NoClock,
  [switch]$Force
)

$ErrorActionPreference = "Stop"
Set-Location -Path $PSScriptRoot

if (-not (Test-Path $Config)) { throw "config not found: $Config" }

# --- interpreter -----------------------------------------------------------
# Same idea as the bash probe: find a python that can actually import the
# dependencies, rather than assuming the first one on PATH is the right one.
function Find-Python {
  if ($Python) { return $Python }
  foreach ($c in @("python", "python3", "py")) {
    $exe = Get-Command $c -ErrorAction SilentlyContinue
    if (-not $exe) { continue }
    & $c -c "import zenoh, cbor2, numpy" 2>$null
    if ($LASTEXITCODE -eq 0) { return $c }
  }
  throw @"
No Python with zenoh + cbor2 + numpy found.
  pip install -r requirements.txt
or pass -Python C:\path\to\python.exe
"@
}
$py = Find-Python
Write-Host "using $py ($(& $py -V 2>&1))"

# Python block-buffers stdout when it is not a console, so without this the
# logs stay empty for minutes and a working fleet looks hung.
$env:PYTHONUNBUFFERED = "1"

# --- refuse to stack a second fleet ---------------------------------------
if (-not $Force) {
  $busy = Get-NetTCPConnection -LocalPort 7447 -State Listen -ErrorAction SilentlyContinue
  if ($busy) {
    throw @"
A fleet is already running (something is listening on 7447).
Two fleets share one supernode and produce duplicate truth streams, which
looks like the tracker losing the target at random. Stop the old one first,
or pass -Force if you really want a second site.
"@
  }
}

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
Get-ChildItem -Path $LogDir -Filter *.log -ErrorAction SilentlyContinue | Remove-Item -Force

$nodes = (Get-Content $Config -Raw | ConvertFrom-Json).nodes.id
$procs = @()

function Start-Piece([string]$name, [string[]]$argv) {
  $out = Join-Path $LogDir "$name.log"
  $p = Start-Process -FilePath $py -ArgumentList $argv -NoNewWindow -PassThru `
       -RedirectStandardOutput $out -RedirectStandardError "$out.err"
  $script:procs += $p
  return $p
}

try {
  $extra = @()
  if ($NoClock) { $extra += "--no-clock-correction" }

  Write-Host "starting supernode"
  Start-Piece "supernode" (@("supernode.py", "--config", $Config) + $extra) | Out-Null
  Start-Sleep -Seconds 3

  foreach ($n in $nodes) {
    if ($n -eq $Hw) {
      Write-Host "starting $n (real SDR via $HwSource)"
      Start-Piece $n @("node.py", "--config", $Config, "--node", $n, "--source", $HwSource) | Out-Null
    } else {
      Write-Host "starting $n (emulated)"
      Start-Piece $n @("node.py", "--config", $Config, "--node", $n, "--source", "sim") | Out-Null
    }
  }
  Start-Sleep -Seconds 3

  Write-Host "starting scene"
  Start-Piece "scene" @("scene.py", "--config", $Config) | Out-Null

  Write-Host ""
  Write-Host "console:  http://localhost:8080"
  Write-Host "spectrum: http://localhost:8081"
  Write-Host "logs:     $LogDir\"
  Write-Host "ctrl-c to stop"
  Write-Host ""
  Start-Sleep -Seconds 2
  Get-Content (Join-Path $LogDir "supernode.log") -Wait -Tail 20
}
finally {
  # Runs on ctrl-c as well as on normal exit, which is the whole point: a
  # half-stopped fleet leaves nodes and a scene alive, and the next start
  # then quietly stacks a second one on top.
  Write-Host ""
  Write-Host "stopping $($procs.Count) processes"
  foreach ($p in $procs) {
    if ($p -and -not $p.HasExited) {
      try { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue } catch {}
    }
  }
}
