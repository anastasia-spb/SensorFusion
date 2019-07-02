call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat"

mkdir build

cd build
cmake -G "Visual Studio 15 2017 Win64" .. -T "v140"
cd ..
