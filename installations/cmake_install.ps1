cd deps
if (-Not (Test-Path .\cmake-3.20.2-windows-x86_64.zip)) {
    Write-Output "Installing cmake...."
    Invoke-WebRequest -OutFile cmake-3.20.2-windows-x86_64.zip https://cmake.org/files/LatestRelease/cmake-3.20.2-windows-x86_64.zip
    Expand-Archive -LiteralPath ./cmake-3.20.2-windows-x86_64.zip -DestinationPath .\
    $oldpath = (Get-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name PATH).path
    $newpath = $oldpath + $pwd + '\cmake-3.20.2-windows-x86_64\bin;'
    Set-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name PATH -Value $newPath 
}
cd ..