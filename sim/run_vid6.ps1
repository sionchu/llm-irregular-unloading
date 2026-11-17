# Re-render all video clips: camera mast removed, picked items placed on the take-away conveyor (U1_PLACE default on),
# new left-rear high camera so the conveyor is in view. Old clip dirs/mp4 moved to _clips_v5. Then data + Remotion KO/EN. Log: u1\vid6.log
$u = $PSScriptRoot
Set-Location $u
$log = "$u\vid6.log"
$ff = 'C:\Users\getch\AppData\Local\Microsoft\WinGet\Packages\Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-8.0-full_build\bin\ffmpeg.exe'
function L($m) { Add-Content $log "$(Get-Date -f s) $m" }
function Busy { (Get-Process kit -ErrorAction SilentlyContinue) -or (Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'cells_|chain_|lf_real|hyperframes' }) }
$R = "$u\results_irregular"
$jobs = @(
  @{d = 'vid_rule300'; tr = 'agent_ikrule_300'; f = '2'; t = '2'; rt = ''; ev = 2; c = '' },
  @{d = 'show_fail_opus300'; tr = 'agent_opus_300'; f = '8'; t = '9'; rt = ''; ev = 2; c = '' },
  @{d = 'show_catch_opus300'; tr = 'agent_opus_300'; f = '8'; t = '9'; rt = ''; ev = 2; c = '1' },
  @{d = 'vid_astralift300'; tr = 'agent_astra_v3lift_300'; f = '12'; t = '13'; rt = 'lift'; ev = 2; c = '' },
  @{d = 'vid_astra300'; tr = 'agent_astra_v3prev_300'; f = '9'; t = '16'; rt = 'lift'; ev = 2; c = '' },
  @{d = 'grid_rule300'; tr = 'agent_ikrule_300'; f = '0'; t = '9999'; rt = ''; ev = 8; c = '' },
  @{d = 'grid_rulelift300'; tr = 'agent_ikrule_lift_300'; f = '0'; t = '9999'; rt = 'lift'; ev = 8; c = '' },
  @{d = 'grid_astralift300'; tr = 'agent_astra_v3lift_300'; f = '0'; t = '9999'; rt = 'lift'; ev = 8; c = '' },
  @{d = 'grid_astraprev300'; tr = 'agent_astra_v3prev_300'; f = '0'; t = '9999'; rt = 'lift'; ev = 8; c = '' }
)
New-Item -ItemType Directory -Force "$u\_clips_v5" | Out-Null
L "VID6 start"
$env:PYTHONIOENCODING = 'utf-8'; $env:U1_SC = 'irregular'; $env:U1_GRIP = 'A'
foreach ($j in $jobs) {
  $dir = "$u\$($j.d)"
  if (Test-Path "$dir\done6.txt") { continue }
  if (Test-Path $dir) { Move-Item $dir "$u\_clips_v5\$($j.d)" -Force }
  if (Test-Path "$dir.mp4") { Move-Item "$dir.mp4" "$u\_clips_v5\$($j.d).mp4" -Force }
  Start-Sleep -Seconds 5
  while (Busy) { Start-Sleep -Seconds 20 }
  $e = @{U1_SHOW_EYE = '-1.5,-3.9,2.9'; U1_SHOW_TGT = '0.0,-0.2,1.0'; U1_SHOW_FOCAL = '13'; U1_CONV_Y = '-1.55'; U1_RES = '1280x720'; U1_SUB = '2';
    U1_REPLAY = "$R\$($j.tr)_trace.json"; U1_SHOW_FROM = $j.f; U1_SHOW_TO = $j.t; U1_RETREAT = $j.rt; U1_CATCH = $j.c; U1_TAG = "_v6$($j.d)" }
  foreach ($k in $e.Keys) { if ($e[$k] -eq '') { Remove-Item "Env:$k" -ErrorAction SilentlyContinue } else { Set-Item "Env:$k" $e[$k] } }
  $t0 = Get-Date
  L "start $($j.d)"
  & C:\isaacsim\python.bat "$u\u1_sim_cl.py" --seed 300 --agent replay --show $dir --show_every $j.ev 2>&1 | Select-String -Pattern '\[u1\]|Traceback|Error' | % { Add-Content $log ("  " + $_.Line) }
  foreach ($k in $e.Keys) { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
  $pngs = Get-ChildItem $dir -Filter '*.png' -ErrorAction SilentlyContinue | Sort Name
  if ($pngs) {
    & $ff -y -loglevel error -framerate 30 -start_number ([int]$pngs[0].BaseName) -i "$dir\%05d.png" -c:v libx264 -crf 18 -pix_fmt yuv420p "$dir.mp4"
    Set-Content "$dir\done.txt" 'ok'; Set-Content "$dir\done6.txt" 'ok'
  }
  L "done $($j.d) $([int]((Get-Date) - $t0).TotalSeconds)s frames=$($pngs.Count)"
}
L "CLIPS_END"
& python "$u\_vid_data.py" 2>&1 | % { Add-Content $log ("  data: " + $_) }
L "DATA_END"
$o = "$u\remotion_vid\out"
foreach ($n in 'llm_unload_white_EN', 'llm_unload_white_KO') { if (Test-Path "$o\$n.mp4") { Copy-Item "$o\$n.mp4" "$o\${n}_v5.mp4" -Force } }
cmd /c "$u\remotion_vid\render4.cmd"
L "VID6_END"

