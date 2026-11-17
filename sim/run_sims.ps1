param([string[]]$Methods = @('rule', 'rule_noisy', 'opus_img', 'qwen3vl8b_img', 'qwen3vl8b_table'))
# Physics execution of every available plan (plans\<method>_<seed>.json) -> results\<method>_<seed>.json
$u = $PSScriptRoot
$log = "$u\sims.log"
Add-Content $log "start $(Get-Date -f s) $($Methods -join ',')"
foreach ($m in $Methods) {
  foreach ($p in Get-ChildItem "$u\plans" -Filter "${m}_*.json" | Sort-Object Name) {
    $seed = [int]($p.BaseName -replace '.*_', '')
    if ($p.BaseName -ne "${m}_$seed") { continue }
    if (Test-Path "$u\results\${m}_$seed.json") { continue }
    & C:\isaacsim\python.bat "$u\u1_sim.py" --seed $seed --plan $p.FullName 2>&1 | Select-String -Pattern 'RESULT|Traceback|Error: ' | % { Add-Content $log $_.Line }
  }
}
Add-Content $log "end $(Get-Date -f s)"
