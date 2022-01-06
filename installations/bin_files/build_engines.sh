if ! command -v trtexec &> /dev/null
then
    FILE=/usr/src/tensorrt/bin/trtexec
    if test -f "$FILE"; then
        export PATH=$PATH:/usr/src/tensorrt/bin
    else
        echo "Requirements not complete. Installing..."
        FOLDER=/usr/src/tensorrt/samples/trtexec/
        if [ -d "$FOLDER" ]; then
            sudo chown -R $USER: /usr/src/tensorrt/
            cd /usr/src/tensorrt/samples/trtexec/
            make
            cd -
            if test -f "$FILE"; then
                export PATH=$PATH:/usr/src/tensorrt/bin
            else
                echo "Can't locate trtexec. run: 'sudo find / -name trtexec' and add it to the PATH"
                exit
            fi
        else
            echo "Can't locate trtexec folder. run: 'sudo find / -name tensorrt' and find the folder with trtexec. Go in that folder and run 'make'"
            exit
        fi
    fi
fi

a=1
success=""
failed=""
while [ $a -lt 6 ]
do
    echo "generating engine for $a"

    if [ "$a" -eq 1 ]
    then
        b=engine_depth384.trt
    fi
    if [ "$a" -eq 2 ]
    then
        b=engine_depth128.trt
    fi
    if [ "$a" -eq 3 ]
    then
        b=engine_floor.trt
    fi
    if [ "$a" -eq 4 ]
    then
        b=engine_detection.trt
    fi
    if [ "$a" -eq 5 ]
    then
        b=engine_track.trt
    fi
    if [ "$a" -lt 5 ]
    then
            trtexec --onnx=./generate/$a.bin --int8 --fp16 --calib=./generate/$a.cache --batch=1 --verbose --saveEngine=/usr/local/bin/data/$b --workspace=$1
    else
            trtexec --onnx=./generate/$a.bin --fp16 --batch=1 --verbose --saveEngine=/usr/local/bin/data/$b --workspace=$1
    fi

    if [ $? -eq 0 ]; then 
        rm ./data/$b 
        echo "$b engine created"
        success="$success$b "
    else
        echo "$b engine creation failed"
        $failed="$failed$b "
    fi
    a=`expr $a + 1`
done
echo "the following engines were built successfully: $success"
