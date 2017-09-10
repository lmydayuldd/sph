#!/usr/bin/env bash

# Currently deploys only MSVC64 Release builds (at two destinations).

current_dir=`pwd`
windeployqt_for_msvc64_dir="E:/dev/Qt/5.9.1/msvc2017_64/bin"
deploy_dst_1="E:/Dropbox/well/WFAIS/prog/Qt/build-SPH-Desktop_Qt_5_9_1_MSVC2017_64bit-Release/release"
deploy_dst_2="E:/Dropbox/well/WFAIS/prog/Qt/DLLs/msvc64"

export VCINSTALLDIR="C:/Program Files (x86)/Microsoft Visual Studio/Shared/14.0/VC/" # for windeployqt.exe

cd $windeployqt_for_msvc64_dir
./windeployqt "$deploy_dst_1/SPH.exe"
./windeployqt "$deploy_dst_2/SPH.exe"
cd $current_dir

declare -a deploy_dsts=("$deploy_dst_1", "$deploy_dst_2")
for deploy_dst in "${deploy_dsts[@]}"
do
	cp "E:/dev/opencv/build/install/x64/vc15/bin/opencv_core330.dll" $deploy_dst
	cp "E:/dev/opencv/build/install/x64/vc15/bin/opencv_imgcodecs330.dll" $deploy_dst
	cp "E:/dev/opencv/build/install/x64/vc15/bin/opencv_imgproc330.dll" $deploy_dst
	cp "E:/dev/opencv/build/install/x64/vc15/bin/opencv_videoio330.dll" $deploy_dst
	cp "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v8.0/bin/cudart64_80.dll" $deploy_dst
	cp "C:/Windows/System32/nvcuda.dll" $deploy_dst
	cp "C:/Windows/System32/nvcuda.dll" $deploy_dst
	cp "C:/Windows/System32/ucrtbased.dll" $deploy_dst
	cp "C:/Windows/System32/nvfatbinaryLoader.dll" $deploy_dst
done

cp $deploy_dst_1/SPH.exe $deploy_dst_2/SPH.exe
