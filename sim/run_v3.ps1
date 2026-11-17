# No catch conveyor (user 11/12: "받침 의미없음, 없애"). Prompt v3 = coverage rule removed, falls = damage x2.
# Waits for the running Opus v2 wall 300 to finish (kept as reference), then stops only the run_v2 queue and its u1 sim child. Log: u1\v3.log
$u = $PSScriptRoot
Set-Location $u
$log = "$u\v3.log"
function L($m) { Add-Content $log "$(Get-Date -f s) $m" }
while (-not (Test-Path "$u\results_irregular\agent_opus_v2_300.json")) { Start-Sleep -Seconds 15 }
Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'run_v2\.ps1' -and $_.ProcessId -ne $PID } | % { Stop-Process -Id $_.ProcessId -Force }
Start-Sleep -Seconds 5
# only u1 sim processes started by run_v2 (catch runs: ikrule_v2 / astra_v2)
Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'u1_sim_cl\.py' } | % { Stop-Process -Id $_.ProcessId -Force }
L "run_v2 stopped after opus_v2 300"
function Busy { (Get-Process kit -ErrorAction SilentlyContinue) -or (Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'u1_sim_cl|cells_|chain_|lf_real|hyperframes' }) }
function Run($tag, $argList, $envs) {
  while (Busy) { Start-Sleep -Seconds 20 }
  foreach ($k in $envs.Keys) { Set-Item "Env:$k" $envs[$k] }
  $t0 = Get-Date
  L "start $tag"
  & C:\isaacsim\python.bat "$u\u1_sim_cl.py" @argList 2>&1 | Select-String -Pattern '\[u1\]|Traceback|Error' | % { Add-Content $log ("  " + $_.Line) }
  L "done $tag $([int]((Get-Date) - $t0).TotalSeconds)s"
  foreach ($k in $envs.Keys) { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
}
$env:PYTHONIOENCODING = 'utf-8'; $env:U1_SC = 'irregular'; $env:U1_GRIP = 'A'
Remove-Item Env:U1_CATCH -ErrorAction SilentlyContinue
foreach ($s in 302..304) {
  if (-not (Test-Path "$u\results_irregular\agent_ikrule_$s.json")) { Run "ikrule $s" @('--seed', "$s", '--agent', 'ikrule') @{} }
}
foreach ($s in 300..304) {
  if (-not (Test-Path "$u\results_irregular\agent_astra_v3_$s.json")) { Run "astra_v3 $s" @('--seed', "$s", '--agent', 'astra') @{U1_PROMPT = 'v3'; U1_TAG = '_v3' } }
}
L "END"
