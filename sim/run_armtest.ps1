# One-clip test of the arm fixes (display IK fallback + clean conveyor set-down). Log: u1\armtest.log
$u = $PSScriptRoot
Set-Location $u
$log = "$u\armtest.log"
function L($m) { Add-Content $log "$(Get-Date -f s) $m" }
function Busy { (Get-Process kit -ErrorAction SilentlyContinue) -or (Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'cells_|chain_|lf_real|hyperframes' }) }
$ff = 'ffmpeg'
$name = if ($args.Count -gt 0) { $args[0] } else { 'vid_astra300' }
$jobs = @{
  vid_rule300 = @{tr = 'agent_ikrule_300'; f = '2'; t = '2'; rt = '' }
  vid_astralift300 = @{tr = 'agent_astra_v3lift_300'; f = '12'; t = '13'; rt = 'lift' }
  vid_astra300 = @{tr = 'agent_astra_v3prev_300'; f = '9'; t = '16'; rt = 'lift' }
}
$j = $jobs[$name]
$dir = "$u\armtest_$name"
if (Test-Path $dir) { Remove-Item $dir -Recurse -Force }
while (Busy) { Start-Sleep -Seconds 20 }
$env:PYTHONIOENCODING = 'utf-8'; $env:U1_SC = 'irregular'; $env:U1_GRIP = 'A'
$e = @{U1_SHOW_EYE = '-1.5,-3.9,2.9'; U1_SHOW_TGT = '0.0,-0.2,1.0'; U1_SHOW_FOCAL = '13'; U1_CONV_Y = '-1.55'; U1_RES = '1280x720'; U1_SUB = '2';
  U1_REPLAY = "$u\results_irregular\$($j.tr)_trace.json"; U1_SHOW_FROM = $j.f; U1_SHOW_TO = $j.t; U1_RETREAT = $j.rt; U1_CATCH = ''; U1_TAG = "_armtest$name" }
foreach ($k in $e.Keys) { if ($e[$k] -eq '') { Remove-Item "Env:$k" -ErrorAction SilentlyContinue } else { Set-Item "Env:$k" $e[$k] } }
L "start $name"
$t0 = Get-Date
& C:\isaacsim\python.bat "$u\u1_sim_cl.py" --seed 300 --agent replay --show $dir --show_every 2 2>&1 | Select-String -Pattern '\[u1\]|Traceback|Error|line \d+' | % { Add-Content $log ("  " + $_.Line) }
$pngs = Get-ChildItem $dir -Filter '*.png' -ErrorAction SilentlyContinue | Sort Name
if ($pngs) { & $ff -y -loglevel error -framerate 30 -start_number ([int]$pngs[0].BaseName) -i "$dir\%05d.png" -c:v libx264 -crf 18 -pix_fmt yuv420p "$dir.mp4" }
L "done $name $([int]((Get-Date) - $t0).TotalSeconds)s frames=$($pngs.Count)"
