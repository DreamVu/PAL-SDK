if (-Not (Test-Path $Env:SystemDrive\pal_files)) {
    New-Item -Path $Env:SystemDrive\ -Name "pal_files" -ItemType "directory"
}

Copy-Item dualenet.onnx $Env:SystemDrive/pal_files  -Force
Copy-Item -r data/ $Env:SystemDrive/pal_files -Force
Copy-Item depth*.bin $Env:SystemDrive/pal_files  -Force
