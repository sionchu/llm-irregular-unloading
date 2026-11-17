# Extend to 10 irregular walls: rule (plain + lift) and Astra v3 + lift on 305-309. Log: u1\astra3.log
$u = $PSScriptRoot
Set-Location $u
$log = "$u\astra3.log"
function L($m) { Add-Content $log "$(Get-Date -f s) $m" }
function Busy { (Get-Process kit -ErrorAction SilentlyContinue) -or (Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'cells_|chain_|lf_real|hyperframes' }) }
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
foreach ($s in 305..309) {
  if (-not (Test-Path "$u\results_irregular\agent_ikrule_$s.json")) { Run "ikrule $s" @('--seed', "$s", '--agent', 'ikrule') @{} }
  if (-not (Test-Path "$u\results_irregular\agent_ikrule_lift_$s.json")) { Run "ikrule lift $s" @('--seed', "$s", '--agent', 'ikrule') @{U1_RETREAT = 'lift'; U1_TAG = '_lift' } }
}
foreach ($s in 305..309) {
  if (-not (Test-Path "$u\results_irregular\agent_astra_v3lift_$s.json")) { Run "astra_v3lift $s" @('--seed', "$s", '--agent', 'astra') @{U1_PROMPT = 'v3'; U1_RETREAT = 'lift'; U1_TAG = '_v3lift' } }
}
L "END10"
