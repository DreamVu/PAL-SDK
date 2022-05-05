#!/bin/bash

./run

File1=lut/Res1.png 
File2=./include/Res2_ERLeft_inverted.png 

validated=false 

if test -f "$File1"; then  
    cp -r lut/ ../bin_files/data/
    cp -r lut_stereo/ ../bin_files/data/    
    cp -r er/ ../bin_files/data/
    cp -r inv_er/ ../bin_files/data/ 
    
    cp -r lut/ /usr/local/bin/data/
    cp -r lut_stereo/ /usr/local/bin/data/   
    cp -r er/ /usr/local/bin/data/
    cp -r inv_er/ /usr/local/bin/data/
    
    cp unit.txt /usr/local/bin/data/    
        
    rm -rf lut/ er/ inv_er/ lut_stereo/
    validated=true
fi

if test -f "$File2"; then  
    rm ./include/Res2_ERLeft_inverted.png ./include/Res2_ERLeft_upright.png   
fi 

if $validated; then     
    rm -rf data
    rm data.zip	
    echo "[PAL:INFO] Camera data files are installed successfully." 
fi
 


