# U1 IK-agent experiments (irregular-unloading research thread). Waits until no other Isaac (kit.exe) runs, then runs
# sequentially. Log: u1\agent.log. Agents: ikrule (code policy, same IK tools), opus (Claude Opus via MCP tools).
param([string[]]$Agents = @('ikrule', 'opus'), [string]$Sets = 'irregular:300-304,mixed:200-204')
$env:PYTHONIOENCODING = 'utf-8'
$u = $PSScriptRoot
$log = "$u\agent.log"
Set-Location $u
function L($m) { Add-Content $log "$(Get-Date -f s) $m" }
L "queued $($Agents -join ',') $Sets"
function Busy { (Get-Process kit -ErrorAction SilentlyContinue) -or (Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'cells_|chain_|lf_real|hyperframes' }) }
$env:U1_GRIP = 'A'
foreach ($set in $Sets.Split(',')) {
  $sc, $rng = $set.Split(':'); $a, $b = $rng.Split('-'); $seeds = [int]$a..[int]$b
  $env:U1_SC = $sc
  foreach ($s in $seeds) {
    foreach ($m in $Agents) {
      if (Test-Path "$u\results_$sc\agent_${m}_$s.json") { continue }
      while (Busy) { Start-Sleep -Seconds 30 }
      $t0 = Get-Date
      & C:\isaacsim\python.bat "$u\u1_sim_cl.py" --seed $s --agent $m 2>&1 |
        Select-String -Pattern '\[u1\] (RESULT|agent|handoff|agent_)|Traceback|Error|error' | % { Add-Content $log $_.Line }
      L "$sc $m $s done $([int]((Get-Date) - $t0).TotalSeconds)s"
    }
  }
}
L "END"
