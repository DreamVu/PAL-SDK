Set-Location deps
if (-Not (Test-Path .\opencv.zip)) {
    Write-Output "Downloading opencv..."
    Invoke-WebRequest -OutFile opencv.zip https://github.com/opencv/opencv/archive/3.4.4.zip
}
if (-Not (Test-Path .\opencv_contrib.zip)) {
    Write-Output "Downloading opencv-contrib..."
    Invoke-WebRequest -OutFile opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.4.zip
}

if (-Not (Test-Path .\opencv)) {
    Expand-Archive -LiteralPath ./opencv.zip -DestinationPath .\
    Move-Item opencv-3.4.4 opencv
    new-item .\sources -itemtype directory
    mv opencv/* sources
    mv sources opencv/
}
if (-Not (Test-Path .\opencv_contrib)) {
    Expand-Archive -LiteralPath ./opencv_contrib.zip -DestinationPath .\
    Move-Item opencv_contrib-3.4.4 opencv_contrib
}
if (-Not (Test-Path $Env:SystemDrive\opencv)) {
    Move-Item opencv $Env:SystemDrive\
}
if (-Not (Test-Path $Env:SystemDrive\opencv_contrib)) {
    Move-Item opencv_contrib $Env:SystemDrive\
}

Set-Location ..
Write-Output "setup of opencv and qt in progress..."
cmake -G "MinGW Makefiles" -D Qt5_DIR=$Env:SystemDrive\Qt\5.12.2\mingw73_64\lib\cmake\Qt5 -D QT_MAKE_EXECUTABLE=$Env:SystemDrive\Qt\5.12.2\mingw73_64\bin\qmake.exe -D Qt5Concurrent_DIR=$Env:SystemDrive\Qt\5.12.2\mingw73_64\lib\cmake\Qt5Concurrent -D Qt5Core_DIR=$Env:SystemDrive\Qt\5.12.2\mingw73_64\lib\cmake\Qt5Core -D Qt5Gui_DIR=$Env:SystemDrive\Qt\5.12.2\mingw73_64\lib\cmake\Qt5Gui -D Qt5Test_DIR=$Env:SystemDrive\Qt\5.12.2\mingw73_64\lib\cmake\Qt5Test -D Qt5Widgets_DIR=$Env:SystemDrive\Qt\5.12.2\mingw73_64\lib\cmake\Qt5Widgets -D Qt5OpenGL_DIR=$Env:SystemDrive\Qt\5.12.2\mingw73_64\lib\cmake\Qt5OpenGL-D ENABLE_CXX11=ON -D WITH_MSMF=OFF -D WITH_OPENGL=ON WITH_OPENGL=ON -D CMAKE_BUILD_TYPE=RELEASE -D ENABLE_PRECOMPILED_HEADERS=OFF -D OPENCV_VS_VERSIONINFO_SKIP=1 -D WITH_CUDA=OFF -D BUILD_opencv_cudacodec=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=$Env:SystemDrive\opencv_contrib\modules -D OPENCV_ENABLE_NONFREE=OFF -D BUILD_EXAMPLES=OFF -S $Env:SystemDrive\opencv\sources -B $Env:SystemDrive\opencv\opencv-build
## fixing DShow error
((Get-Content -path $Env:SystemDrive\opencv\sources\modules\videoio\src\cap_dshow.cpp -Raw) -replace "#include `"DShow.h`"", "#define NO_DSHOW_STRSAFE`n#define STRSAFE_NO_DEPRECIATE`n#include `"DShow.h`"") | Set-Content -Path $Env:SystemDrive\opencv\sources\modules\videoio\src\cap_dshow.cpp
Set-Location $Env:SystemDrive\opencv\opencv-build
mingw32-make -j 6
mingw32-make install

$oldpath = (Get-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name PATH).path
$opencv_bin_path = $Env:SystemDrive + '\\opencv\\opencv-build\\install\\x64\\mingw\\bin'
if(-Not ($oldpath -Match $opencv_bin_path)){
    $newpath = $oldpath + $Env:SystemDrive + '\opencv\opencv-build\install\x64\mingw\bin;'
    Set-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name PATH -Value $newPath
}
Write-Output "Now you are good to go"
