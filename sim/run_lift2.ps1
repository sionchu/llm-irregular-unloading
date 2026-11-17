# Rerun 'lift' with the IK fallback (plain retreat when the lifted pose has no IK). Waits for run_lift's variants, stops it before Astra.
$u = $PSScriptRoot
Set-Location $u
$log = "$u\lift.log"
function L($m) { Add-Content $log "$(Get-Date -f s) $m" }
while (-not (Select-String -Path $log -Pattern 'LIFT_DONE' -Quiet)) { Start-Sleep -Seconds 10 }
Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'run_lift\.ps1' -and $_.ProcessId -ne $PID } | % { Stop-Process -Id $_.ProcessId -Force }
Start-Sleep -Seconds 3
Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'u1_sim_cl\.py' -and $_.CommandLine -match 'astra' } | % { Stop-Process -Id $_.ProcessId -Force }
L "run_lift stopped before Astra; lift rerun with IK fallback"
function Busy { (Get-Process kit -ErrorAction SilentlyContinue) -or (Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'cells_|chain_|lf_real|hyperframes' }) }
function Run($tag, $argList, $envs) {
  while (Busy) { Start-Sleep -Seconds 20 }
  foreach ($k in $envs.Keys) { Set-Item "Env:$k" $envs[$k] }
  $t0 = Get-Date
  L "start $tag"
  & C:\isaacsim\python.bat "$u\u1_sim_cl.py" @argList 2>&1 | Select-String -Pattern 'RESULT|Traceback|Error' | % { Add-Content $log ("  " + $_.Line) }
  L "done $tag $([int]((Get-Date) - $t0).TotalSeconds)s"
  foreach ($k in $envs.Keys) { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
}
$env:PYTHONIOENCODING = 'utf-8'; $env:U1_SC = 'irregular'; $env:U1_GRIP = 'A'
Remove-Item Env:U1_CATCH -ErrorAction SilentlyContinue
foreach ($s in 300..304) {
  if (-not (Test-Path "$u\results_irregular\agent_ikrule_lift_$s.json")) { Run "ikrule lift(fb) $s" @('--seed', "$s", '--agent', 'ikrule') @{U1_RETREAT = 'lift'; U1_TAG = '_lift' } }
}
L "LIFT2_DONE"
