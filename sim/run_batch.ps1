param([string]$Mode = 'obs', [string]$Method = 'rule', [int]$Seed0 = 100, [int]$N = 20)
# Sequential U1 batch: -Mode obs renders observations; -Mode run executes plans\<Method>_<seed>.json for each seed.
$u = $PSScriptRoot
$log = "$u\batch_${Mode}_${Method}.log"
Set-Content $log "start $(Get-Date -f s)" -Encoding UTF8
for ($s = $Seed0; $s -lt $Seed0 + $N; $s++) {
  if ($Mode -eq 'obs') {
    if (Test-Path "$u\obs\wall_${s}_table.json") { Add-Content $log "skip $s"; continue }
    & C:\isaacsim\python.bat "$u\u1_sim.py" --seed $s --obs 2>&1 | Select-String -Pattern '\[u1\]|Traceback' | % { Add-Content $log $_.Line }
  } else {
    $p = "$u\plans\${Method}_${s}.json"
    if (-not (Test-Path $p)) { Add-Content $log "no plan $s"; continue }
    & C:\isaacsim\python.bat "$u\u1_sim.py" --seed $s --plan $p 2>&1 | Select-String -Pattern 'RESULT|Traceback' | % { Add-Content $log $_.Line }
  }
}
Add-Content $log "end $(Get-Date -f s)"
