@echo off
REM Windows launcher that is not blocked by PowerShell's execution policy.
REM
REM PowerShell refuses to run .ps1 files under the default Restricted policy,
REM and a downloaded one is blocked again by the Mark of the Web even under
REM RemoteSigned. Neither restriction applies to .cmd, so this wrapper starts
REM run_demo.ps1 with the policy bypassed for that one invocation only --
REM nothing is changed permanently, and no administrator rights are needed.
REM
REM   run_demo.cmd -Config config\site-smooth.json
REM   run_demo.cmd -Config config\site-scan-lite.json
REM   run_demo.cmd -Hw n10 -HwSource uhd
REM
REM All arguments are forwarded to run_demo.ps1 unchanged.

setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_demo.ps1" %*
endlocal
