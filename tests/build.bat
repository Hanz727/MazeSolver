cd /d %~dp0
meson setup --reconfigure build/
cd build 
meson compile && tests.exe
cd..
pause
