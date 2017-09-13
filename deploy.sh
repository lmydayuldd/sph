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
	# OpenCV
	cp "E:/dev/opencv/build/install/x64/vc15/bin/opencv_core330.dll" $deploy_dst
	cp "E:/dev/opencv/build/install/x64/vc15/bin/opencv_imgcodecs330.dll" $deploy_dst
	cp "E:/dev/opencv/build/install/x64/vc15/bin/opencv_imgproc330.dll" $deploy_dst
	cp "E:/dev/opencv/build/install/x64/vc15/bin/opencv_videoio330.dll" $deploy_dst

	# CUDA
	cp "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v8.0/bin/cudart64_80.dll" $deploy_dst
	cp "C:/Windows/System32/nvcuda.dll" $deploy_dst
	cp "C:/Windows/System32/nvfatbinaryLoader.dll" $deploy_dst
	cp "C:/Windows/System32/ucrtbased.dll" $deploy_dst

	# MSVC 2015 (in case vcredist_x64.exe fails, because of lacking dependencies, like updates on Window 8.1)
	# but... it seems it still doesn't work with those below if vcredist_x64.exe has not been installed...
	# cp "C:/Windows/System32/concrt140.dll" $deploy_dst
	# cp "C:/Windows/System32/msvcp140.dll" $deploy_dst
	# cp "C:/Windows/System32/vcomp140.dll" $deploy_dst
	# cp "C:/Windows/System32/vcruntime140.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-convert-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-environment-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-filesystem-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-heap-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-locale-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-math-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-multibyte-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-runtime-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-stdio-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-string-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-time-l1-1-0.dll" $deploy_dst
	# cp "C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/Common7/IDE/Remote Debugger/x64/api-ms-win-crt-utility-l1-1-0.dll" $deploy_dst

	cp "E:/Dropbox/well/WFAIS/prog/Qt/SPH/README.md" $deploy_dst
done

cp $deploy_dst_1/SPH.exe $deploy_dst_2/SPH.exe
