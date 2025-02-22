meson setup --reconfigure build/
cd build 
meson compile && tests.exe
cd..
