#-------------------------------------------------
#
# Project created by QtCreator 2016-04-23T08:22:17
#
#-------------------------------------------------

QT += core gui opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET                        = SPH
TEMPLATE                      = app
CONFIG                       += mobility c++11
MOBILITY                      =

# Defines platform-specific preprocessor macro.
#   Edit at "Projects -> QMake -> additional arguments"
#   (by setting to CONFIG+="debug/release DESKTOP/ANDROID_BUILD COMPILER_GPP/MSVC").
CONFIG(DESKTOP_BUILD) : DEFINES += DESKTOP_BUILD
CONFIG(ANDROID_BUILD) : DEFINES += ANDROID_BUILD
CONFIG(COMPILER_MSVC) : DEFINES += COMPILER_MSVC
CONFIG(COMPILER_GPP)  : DEFINES += COMPILER_GPP

*-g++:QMAKE_CXXFLAGS_WARN_ON += -Wextra
#win32-msvc*:QMAKE_CXXFLAGS   += /sdl # TODO, it ignores some errors

CONFIG(debug, debug|release) {
    *-g++ {
        QMAKE_CXXFLAGS += -g3
    }
    win32-msvc* {
        QMAKE_CXXFLAGS += /Zi #/Z7
    }
}
else {
    *-g++ {
        QMAKE_CXXFLAGS += -O3 -msse4.2
    }
    win32-msvc* {
        QMAKE_CXXFLAGS += /O2 /arch:AVX2
    }
}

*-g++ {
    # Qt Creator seems to have a problem providing qwindows.dll / qwindowsd.dll
    CONFIG(debug, debug|release) {
        QWINDOWS_DLL_SRC = $$shell_path(D:\dev\Qt\5.9.1\mingw53_32\plugins\platforms\qwindowsd.dll)
        QWINDOWS_DLL_DST = $$shell_path($${OUT_PWD}/debug/platforms/)
    }
    else {
        QWINDOWS_DLL_SRC = $$shell_path(D:\dev\Qt\5.9.1\mingw53_32\plugins\platforms\qwindows.dll)
        QWINDOWS_DLL_DST = $$shell_path($${OUT_PWD}/release/platforms/)
    }
    QMAKE_POST_LINK += $${QMAKE_MKDIR} $${QWINDOWS_DLL_DST} & \
                       $${QMAKE_COPY} $${QWINDOWS_DLL_SRC} $${QWINDOWS_DLL_DST}
    export(QMAKE_POST_LINK)
}
win32-msvc* {
    # Qt Creator seems to have a problem providing qwindows.dll / qwindowsd.dll
    CONFIG(debug, debug|release) {
        QWINDOWS_DLL_SRC = $$shell_path(D:\dev\Qt\5.9.1\msvc2017_64\plugins\platforms\qwindowsd.dll)
        QWINDOWS_DLL_DST = $$shell_path($${OUT_PWD}/debug/platforms/)
        LIBS += -L"D:\dev\opencv\build\bin\Release"
        LIBS += -LD:\dev\opencv\build\install\x64\vc15\lib \ #####################
                    -lopencv_calib3d330 \
                    -lopencv_core330 \
                    -lopencv_features2d330 \
                    #-lopencv_ffmpeg330 \
                    -lopencv_highgui330 \
                    -lopencv_imgcodecs330 \ # for cv::imwrite
                    -lopencv_imgproc330 \
                    -lopencv_videoio330 # for cv::VideoWriter
                    #-lopencv_world330#_64
    }
    else {
        QWINDOWS_DLL_SRC_0 = $$shell_path(D:\dev\Qt\5.9.1\msvc2017_64\plugins\platforms\qwindows.dll)
        QWINDOWS_DLL_SRC_1 = $$shell_path(D:\dev\opencv\build\install\x64\vc15\bin\opencv_core330.dll)
        QWINDOWS_DLL_SRC_2 = $$shell_path(D:\dev\opencv\build\install\x64\vc15\bin\opencv_imgcodecs330.dll)
        QWINDOWS_DLL_SRC_3 = $$shell_path(D:\dev\opencv\build\install\x64\vc15\bin\opencv_imgproc330.dll)
        QWINDOWS_DLL_SRC_4 = $$shell_path(D:\dev\opencv\build\install\x64\vc15\bin\opencv_videoio330.dll)
        QWINDOWS_DLL_DST_0 = $$shell_path($${OUT_PWD}/release/platforms/)
        QWINDOWS_DLL_DST   = $$shell_path($${OUT_PWD}/release/)
    }
    QMAKE_POST_LINK += $${QMAKE_MKDIR} $${QWINDOWS_DLL_DST_0} & \
                       $${QMAKE_COPY} $${QWINDOWS_DLL_SRC_0} $${QWINDOWS_DLL_DST_0} & \
                       $${QMAKE_COPY} $${QWINDOWS_DLL_SRC_1} $${QWINDOWS_DLL_DST} & \
                       $${QMAKE_COPY} $${QWINDOWS_DLL_SRC_2} $${QWINDOWS_DLL_DST} & \
                       $${QMAKE_COPY} $${QWINDOWS_DLL_SRC_3} $${QWINDOWS_DLL_DST} & \
                       $${QMAKE_COPY} $${QWINDOWS_DLL_SRC_4} $${QWINDOWS_DLL_DST}
    export(QMAKE_POST_LINK)
}

############################################################
# OpenCV ###################################################
############################################################

INCLUDEPATH += D:\dev\opencv\build\include

CONFIG(debug, debug|release) {
    *-g++ {
        LIBS += -LD:/dev/opencv/build/bin/Debug \
                    -lopencv_calib3d330d \
                    -lopencv_core330d \
                    -lopencv_features2d330d \
                    -lopencv_ffmpeg330d \
                    -lopencv_highgui330d \
                    -lopencv_imgproc330d
                    #opencv_world330d
    }
    win32-msvc* {
        LIBS += \#-LD:/dev/opencv/build/bin/Debug \
                -L"D:\dev\opencv\build\install\x64\vc15\bin" \
                -L"D:\dev\opencv\build\install\x64\vc15\lib" \
                    -lopencv_calib3d330d \
                    -lopencv_core330d \
                    -lopencv_features2d330d \
                    #-lopencv_ffmpeg330d \
                    -lopencv_highgui330d \
                    -lopencv_imgproc330d \
                    -lopencv_videoio330d # for VideoWriter
                    #opencv_world330d
    }
}
else {
    *-g++ {
        LIBS += -LD:/dev/opencv/build/bin/Release \
                    -lopencv_calib3d330 \
                    -lopencv_core3sar30 \
                    -lopencv_features2d330 \
                    -lopencv_ffmpeg330 \
                    -lopencv_highgui330 \
                    -lopencv_imgproc330
                    #-lopencv_world330
    }
    win32-msvc* {
        LIBS += \#-LD:/dev/opencv/build/bin/Release \
                -L"D:\dev\opencv\build\install\x64\vc15\bin" \
                -L"D:\dev\opencv\build\install\x64\vc15\lib" \
                    -lopencv_calib3d330 \
                    -lopencv_core330 \
                    -lopencv_features2d330 \
                    #-lopencv_ffmpeg330 \
                    -lopencv_highgui330 \
                    -lopencv_imgcodecs330 \ # for cv::imwrite
                    -lopencv_imgproc330 \
                    -lopencv_videoio330 # for cv::VideoWriter
                    #-lopencv_world330#_64
    }
}

############################################################
# OMP, MPI & CUDA ##########################################
############################################################

INCLUDEPATH += D:/dev/ms-mpi/Include # MPI (Microsoft)

CONFIG(COMPILER_GPP)  : LIBS += -LD:/dev/ms-mpi/Lib/x86 -lmsmpi # MPI (Microsoft)
CONFIG(COMPILER_MSVC) : LIBS += -LD:/dev/ms-mpi/Lib/x64 -lmsmpi # MPI (Microsoft)

*-g++:LIBS += -pthread -fopenmp # OpenMP

*-g++:QMAKE_CXXFLAGS       += -std=c++11 -pthread -fopenmp
win32-msvc*:QMAKE_CXXFLAGS += /openmp

win32-msvc* {
    CUDA_DIR = "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0" # CUDA Toolkit
    CUDA_SDK = "C:\ProgramData\NVIDIA Corporation\CUDA Samples\v8.0" # CUDA SDK

    INCLUDEPATH += $$CUDA_DIR/include \ # CUDA
                   $$CUDA_SDK/common/inc
    CUDA_SRCS += physics/cuda.cu

    QMAKE_LIBDIR += $$CUDA_DIR/lib/x64 \
                    $$CUDA_SDK/common/lib/x64
    LIBS += -lcudart -lcuda -lcublas # CUDA
    CUDA_LIBS += -lcudart -lcuda -lcublas # CUDA
}

win32-msvc* {
    # for CDB debugger
    INCLUDEPATH += "C:\Program Files (x86)\Windows Kits\10\Source\10.0.14393.0\ucrt" \
                   "C:\Program Files (x86)\Windows Kits\10\Source\10.0.14393.0\ucrt\inc"
    LIBS += -L"C:\Program Files (x86)\Windows Kits\10\Lib\10.0.14393.0\ucrt\x64" \
            -lucrtd# -lucrt -llibucrtd -llibucrt

    VISUAL_STUDIO_COMPILER = $$shell_quote("C:\Program Files (x86)\Microsoft Visual Studio\Shared\14.0\VC\bin")
    MSVCRT_LINK_FLAG_DEBUG = "/MDd" # /MD dynamic shared DLL, /MT static, /LD dynamic DLL
    MSVCRT_LINK_FLAG_RELEASE = "/MD" # /MDd dynamic shared DLL, /MTd static, /LD dynamic DLL
    CUDA_ARCH = arch=compute_61,code=sm_61
    NVCC_FLAGS = --use_fast_math
    CUDA_INCL += $$CUDA_DIR/include \ # CUDA
                 $$CUDA_SDK/common/inc \ # CUDA
                 $$_PRO_FILE_PWD_
    CUDA_INCL = $$join(CUDA_INCL,'" -I"','-I"','"')
}

############################################################
# Configuration of the Intermediate CUDA Compiler ##########
############################################################

win32-msvc* {
    cudaIntr.clean = cudaIntrObj/*.o
    win32:cudaIntr.clean = cudaIntrObj/*.obj
}

############################################################
# Configuration of the CUDA Compiler #######################
############################################################

win32-msvc* {
    CONFIG(debug, debug|release) {
        cuda_d.input = CUDA_SRCS
        cuda_d.output = ${QMAKE_FILE_BASE}_link.o
        equals(MSVCRT_LINK_FLAG_DEBUG, "/MDd") {
            linux:cuda_d.output = ${QMAKE_FILE_BASE}_link.obj
            win32:cuda_d.output = ${QMAKE_FILE_BASE}_link.o
            NVCC_FLAGS += --shared -cudart shared
        }
        equals(MSVCRT_LINK_FLAG_DEBUG, "/MTd") {
            win32:cuda_d.output = ${QMAKE_FILE_BASE}_link.a
        }
        equals(MSVCRT_LINK_FLAG_DEBUG, "/LDd") {
            win32:cuda_d.output = ${QMAKE_FILE_BASE}_link.dll
        }
        QMAKE_EXTRA_COMPILERS += cuda_d
        cuda_d.dependency_type = TYPE_C
        cuda_d.commands = $$CUDA_DIR/bin/nvcc.exe --verbose -g \
                              $$CUDA_INCL $$CUDA_LIBS \
                              $$NVCC_FLAGS \
                              -D_DEBUG \
                              -DCOMPILER_MSVC \
                              --machine 64 -gencode $$CUDA_ARCH \
                              -ccbin $$VISUAL_STUDIO_COMPILER \ # MSVC CL.exe directory
                              -Xcompiler $$MSVCRT_LINK_FLAG_DEBUG \ # dynamic/static release
                              -Xcompiler "/nologo,/EHsc,/wd4819,/W3,/Zi,/Od" \
                              -Xcompiler "/RTC1" \ # error checking, incompatible with /O2
                              -Xcompiler "/openmp" \
                              -c -o ${QMAKE_FILE_OUT} \ # output files
                              # -dc instead, for multiple .cu files
                              ${QMAKE_FILE_NAME} # input files
    }
    else {
        cuda.input = CUDA_SRCS
        cuda.output = ${QMAKE_FILE_BASE}_link.o
        equals(MSVCRT_LINK_FLAG_RELEASE, "/MD") {
            linux:cuda.output = ${QMAKE_FILE_BASE}_link.obj
            win32:cuda.output = ${QMAKE_FILE_BASE}_link.o
            NVCC_FLAGS += --shared -cudart shared
        }
        equals(MSVCRT_LINK_FLAG_RELEASE, "/MT") {
            win32:cuda.output = ${QMAKE_FILE_BASE}_link.a
        }
        equals(MSVCRT_LINK_FLAG_RELEASE, "/LD") {
            win32:cuda.output = ${QMAKE_FILE_BASE}_link.dll
        }
        QMAKE_EXTRA_COMPILERS += cuda
        cuda.dependency_type = TYPE_C
        cuda.commands = $$CUDA_DIR/bin/nvcc.exe --verbose \
                            $$CUDA_INCL $$CUDA_LIBS \
                            $$NVCC_FLAGS \
                            -DCOMPILER_MSVC \
                            --machine 64 -gencode $$CUDA_ARCH \
                            -ccbin $$VISUAL_STUDIO_COMPILER \ # MSVC CL.exe directory
                            -Xcompiler $$MSVCRT_LINK_FLAG_RELEASE \ # dynamic/static release
                            -Xcompiler "/nologo,/EHsc,/wd4819,/W3,/Zi,/O2" \
                            -Xcompiler "/openmp,/arch:AVX2" \
                            -c -o ${QMAKE_FILE_OUT} \ # output files
                            # -dc instead, for multiple .cu files
                            ${QMAKE_FILE_NAME} # input files
    }
}

############################################################
# Project files ############################################
############################################################

RESOURCES += \
    E:/Dropbox/well/WFAIS/prog/Qt/SPH

FORMS += \
    window/main_window.ui

DISTFILES += \
    map/sim_communicating_vessels.txt \
    map/sim_dam_break.txt \
    map/sim_dam_fall.txt \
    map/sim_droplet.txt \
    shader/fragment_shader.fsh \
    shader/vertex_shader.vsh

HEADERS += \
    control/interaction.h \
    gl/form.h \
    gl/handles.h \
    gl/matrices.h \
    gl/vertex_array.h \
    machine/cloth.h \
    machine/machine.h \
    machine/particle.h \
    machine/spring.h \
    machine/rope.h \
    machine/walls.h \
    physics/computer.h \
    physics/forces.h \
    physics/geometry.h \
    physics/grid.h \
    physics/octree.h \
    physics/vector.h \
    shader/shader.h \
    shape/arrow.h \
    shape/dot.h \
    shape/landschaft.h \
    shape/line.h \
    shape/rectangle.h \
    shape/shape.h \
    shape/sphere.h \
    shape/triangle.h \
    util/debug_helper.h \
    util/enums.h \
    util/constants.h \
    util/map.h \
    util/operations.h \
    util/parser.h \
    util/settings.h \
    util/strings.h \
    util/timer.h \
    window/gl_window.h \
    window/main_window.h \
    window/simulation_window.h

SOURCES += \
    control/interaction.cpp \
    gl/form.cpp \
    gl/handles.cpp \
    gl/matrices.cpp \
    gl/vertex_array.cpp \
    machine/cloth.cpp \
    machine/machine.cpp \
    machine/particle.cpp \
    machine/rope.cpp \
    machine/spring.cpp \
    machine/walls.cpp \
    main.cpp \
    physics/computer.cpp \
    physics/forces.cpp \
    physics/grid.cpp \
    physics/octree.cpp \
    physics/vector.cpp \
    shader/shader.cpp \
    shape/arrow.cpp \
    shape/dot.cpp \
    shape/landschaft.cpp \
    shape/line.cpp \
    shape/rectangle.cpp \
    shape/shape.cpp \
    shape/sphere.cpp \
    shape/triangle.cpp \
    util/debug_helper.cpp \
    util/enums.cpp \
    util/operations.cpp \
    util/settings.cpp \
    util/strings.cpp \
    util/timer.cpp \
    util/map.cpp \
    window/gl_window.cpp \
    window/main_window.cpp \
    window/simulation_window.cpp
