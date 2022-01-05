if (-Not (Test-Path .\python-3.8.10-amd64.exe)) {
    Write-Output "Download python..."
    Invoke-WebRequest -OutFile python-3.8.10-amd64.exe https://www.python.org/ftp/python/3.8.10/python-3.8.10-amd64.exe
}
if (-Not (Test-Path $Env:SystemDrive\Python38)) {
    Write-Output "Installing python..."
    .\python-3.8.10-amd64.exe /passive InstallAllUsers=1 TargetDir=$Env:SystemDrive\Python38 PrependPath=1 | Out-Null
}
$oldpath = (Get-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name PATH).path
$pypath = $Env:SystemDrive + "\\Python38"
if(-Not ($oldpath -Match $pypath)){
    $newpath = $oldpath + $Env:SystemDrive + "\Python38;" + $Env:SystemDrive + "\Python38\Scripts;"
    Set-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\Session Manager\Environment' -Name PATH -Value $newPath
}

# py -m venv dreamvu_ws
# .\dreamvu_ws\Scripts\Activate.ps1 | Out-Null
python -m ensurepip --default-pip
python -m pip install --upgrade pip
# pip install --upgrade setuptools pip
pip install -r python_requirements.txt --no-cache-dir
# if (-Not (Test-Path $profile)) {
#     New-Item -Path $profile -Type File -Force
# }
# Add-Content $profile ';'
# Add-Content $profile (Get-Item .\dreamvu_ws\Scripts\Activate.ps1).FullName
Write-Output 'Python path added to environment path variable.'