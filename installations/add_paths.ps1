if (-Not (Test-Path $Env:SystemDrive\Qt)){
    Write-Output "Please install QT first from the provided executable."
    Write-Output "Install QT 5.12.2 with MinGW 7.3.0 64-bit."
    exit
} 
$oldpath = (Get-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name PATH).path
$newpath = $oldpath
$newpath = $newpath + $pwd + '\..\lib;'    
$qt_bin_path = $Env:SystemDrive + '\\Qt\\Tools\\mingw730_64\\bin'
if(-Not ($oldpath -Match $pal_lib_path)){
    $newpath = $newpath + $Env:SystemDrive + '\Qt\Tools\mingw730_64\bin;'    
}
Set-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name PATH -Value $newPath
