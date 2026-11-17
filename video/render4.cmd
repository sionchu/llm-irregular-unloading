@echo off
cd /d %~dp0
set FF=C:\Users\getch\AppData\Local\Microsoft\WinGet\Packages\Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-8.0-full_build\bin\ffmpeg.exe
echo START > render4.log
call npx tsc -p . >> render4.log 2>&1
echo TSC_RC=%ERRORLEVEL% >> render4.log
call npx remotion still en-grid out\x_grid_res.png --frame=1160 >> render4.log 2>&1
call npx remotion render Main-en out\llm_unload_white_EN.mp4 --crf 18 --concurrency 4 > render_en.log 2>&1
echo EN_RC=%ERRORLEVEL% >> render4.log
call npx remotion render Main-ko out\llm_unload_white_KO.mp4 --crf 18 --concurrency 4 > render_ko.log 2>&1
echo KO_RC=%ERRORLEVEL% >> render4.log
"%FF%" -y -loglevel error -i out\llm_unload_white_EN.mp4 -vf "fps=1/8,scale=640:-1,tile=4x4" -frames:v 1 -q:v 4 out\contact_white_EN.jpg
echo RENDER_DONE >> render4.log
