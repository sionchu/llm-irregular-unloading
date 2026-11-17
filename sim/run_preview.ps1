# Step 2: Astra v3 + lift + physics preview tool on walls 300-304. Waits for run_astra3b (END10). Log: u1\preview.log
$u = $PSScriptRoot
Set-Location $u
$log = "$u\preview.log"
function L($m) { Add-Content $log "$(Get-Date -f s) $m" }
while (-not (Select-String -Path "$u\astra3.log" -Pattern 'END10' -Quiet)) { Start-Sleep -Seconds 30 }
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
if (-not (Test-Path "$u\results_irregular\agent_astra_v3lift_306.json")) { Run "astra_v3lift 306 rerun" @("--seed", "306", "--agent", "astra") @{U1_PROMPT = "v3"; U1_RETREAT = "lift"; U1_TAG = "_v3lift" } }
foreach ($s in 300..304) {
  if (-not (Test-Path "$u\results_irregular\agent_astra_v3prev_$s.json")) { Run "astra_v3prev $s" @('--seed', "$s", '--agent', 'astra') @{U1_PROMPT = 'v3'; U1_RETREAT = 'lift'; U1_PREVIEW = '1'; U1_TAG = '_v3prev' } }
}
L "END"
