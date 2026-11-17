# after run_vid7 ends: re-render the 4 clips made before armfix8 (display base slide), then data + Remotion again
$u = $PSScriptRoot
while (-not (Select-String -Path "$u\vid7.log" -Pattern 'V9_COPIED' -Quiet)) { Start-Sleep -Seconds 30 }
Add-Content "$u\vid7.log" "$(Get-Date -f s) VID7B clearing pre-armfix8 clips"
foreach ($d in 'show_fail_opus300', 'show_catch_opus300', 'vid_astralift300', 'grid_rule300') {
  Remove-Item -Recurse -Force "$u\$d" -ErrorAction SilentlyContinue
  Remove-Item -Force "$u\$d.mp4" -ErrorAction SilentlyContinue
}
& "$u\run_vid7.ps1"
Add-Content "$u\vid7.log" "$(Get-Date -f s) VID7B_END"
