cd build
del /S /Q *.*
rd /S /Q .
cmake .. -G "Unix Makefiles"
make -j4
Physics2D.exe