## Install python
.\setup_python_env.ps1
##  install cmake
.\cmake_install.ps1
## add necessary env variables
./add_paths.ps1

## copy binaries to C:\pal_files
cd camera_data
./setup_python_lib.ps1
cd ..

