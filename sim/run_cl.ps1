# U1 closed-loop experiments (irregular-unloading research thread). Waits until no other Isaac (kit.exe) runs, then
# runs sequentially. Log: u1\cl.log. Methods: cl_rule, cl_stab, cl_stab_look3, cl_rule_look3. U1_GRIP=A (as baseline).
param([string[]]$Methods = @('cl_rule', 'cl_stab', 'cl_stab_look3'), [string]$Sets = 'irregular:300-304,mixed:200-204')
$env:PYTHONIOENCODING = 'utf-8'
$u = $PSScriptRoot
$log = "$u\cl.log"
Set-Location $u
function L($m) { Add-Content $log "$(Get-Date -f s) $m" }
L "queued $($Methods -join ',') $Sets"
function Busy { (Get-Process kit -ErrorAction SilentlyContinue) -or (Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'cells_|chain_' }) }
while (Busy) { Start-Sleep -Seconds 30 }
L "gpu free, start"
$env:U1_GRIP = 'A'
foreach ($set in $Sets.Split(',')) {
  $sc, $rng = $set.Split(':'); $a, $b = $rng.Split('-'); $seeds = [int]$a..[int]$b
  $env:U1_SC = $sc
  foreach ($m in $Methods) {
    $pl = ($m -replace '^cl_', '') -replace '_look\d+$', ''
    $k = if ($m -match '_look(\d+)$') { $Matches[1] } else { '0' }
    foreach ($s in $seeds) {
      if (Test-Path "$u\results_$sc\${m}_$s.json") { continue }
      $t0 = Get-Date
      & C:\isaacsim\python.bat "$u\u1_sim_cl.py" --seed $s --closed $pl --look $k 2>&1 |
        Select-String -Pattern '\[u1\] RESULT|Traceback|Error: |error' | % { Add-Content $log $_.Line }
      L "$sc $m $s done $([int]((Get-Date) - $t0).TotalSeconds)s"
    }
  }
}
L "END"
