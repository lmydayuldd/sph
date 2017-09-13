#include "window/simulation_window.h"

#include "control/interaction.h"
#include "gl/form.h"
#include "gl/handles.h"
#include "gl/matrices.h"
#include "gl/vertex_array.h"
#include "machine/cloth.h"
#include "machine/rope.h"
#include "machine/walls.h"
#include "physics/computer.h"
#include "physics/grid.h"
#include "shader/shader.h"
#include "util/debug_helper.h"
#include "util/map.h"
#include "util/strings.h"
#include "util/timer.h"
#include "util/settings.h"

class Particle;

#include <QDir>
#include <QScreen>
#include <QString>
#include <QKeyEvent>
#include <QMouseEvent>

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp> // does it include all opencv2/core/*.hpp ?
//#include <opencv2/core/core.hpp>
//#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio.hpp>

#include <omp.h>
#include "mpi.h"

#ifdef COMPILER_MSVC
    #include <cuda.h>
    #include <cuda_runtime.h> // checkCudaErrors()
    #include <helper_cuda.h> // checkCudaErrors() // helper_cuda.h must go after cuda_runtime.h

    template< typename T >
    unsigned checkWithoutExit(T result, char const *const func,
                              const char *const file, int const line) {
        if (result) {
            fprintf(stderr, "CUDA error at %s:%d code=%d(%s) \"%s\" \n",
                    file, line, static_cast<unsigned int>(result), _cudaGetErrorEnum(result), func);
            DEVICE_RESET
            //exit(EXIT_FAILURE);
            return static_cast<unsigned>(result);
        }
        return 0;
    }
    #define checkCudaErrorsWithoutExit(val) \
        checkWithoutExit((val), #val, __FILE__, __LINE__)
#endif

SimulationWindow *SimulationWindow::simWin = nullptr;
Timer *SimulationWindow::frameTimer = new Timer();
std::vector<QImage> SimulationWindow::screens;
bool SimulationWindow::cudaInitialized = false;
int SimulationWindow::frame = 0;
int SimulationWindow::refreshRate = 0;
long long int SimulationWindow::frameStartTime = 0;
long long int SimulationWindow::currentTime = 0;
double SimulationWindow::frame_dt = 0;

SimulationWindow::SimulationWindow()
{
    simWin = this;

//     MPI_Init(NULL, NULL);
//     MPI_Finalize();

////////////////////////////////////////////////////////////////////////////////

#ifdef COMPILER_MSVC
    int deviceCount = 0;
    int cudaDevice = 0;
    char cudaDeviceName[100];
    if (checkCudaErrorsWithoutExit(cuInit(0)) == 0) {
        checkCudaErrors(cuDeviceGetCount(&deviceCount));
        checkCudaErrors(cuDeviceGet(&cudaDevice, 0));
        checkCudaErrors(cuDeviceGetName(cudaDeviceName, 100, cudaDevice));
        qDebug() << "Number of devices: " << deviceCount;
        qDebug() << "Device name:" << cudaDeviceName;
        cudaInitialized = true;
    }
    else {
        std::cout << "You probably don't possess proper CUDA-enabled nVidia GPU. Moving on..."
                  << std::endl << std::flush;
    }
#endif
}

SimulationWindow::~SimulationWindow()
{
    std::cout << "Exiting the program." << std::flush;
}

void SimulationWindow::prepareSimulation()
{
    omp_set_num_threads(Settings::PARALLEL_OMP_THREADS);

    Strings::init(); // TODO // called Strings but reponsible for dirs also...

    Grid::init();

    Map::generate();

    for (unsigned i = 0; i < Settings::PARTICLE_COUNT_2D; ++i)
    {
        Particle::collision.push_back(std::vector<bool>(Settings::PARTICLE_COUNT_2D));
        Particle::collisionDistance.push_back(std::vector<double>(Settings::PARTICLE_COUNT_2D));
        for (unsigned j = 0; j < Settings::PARTICLE_COUNT_2D; ++j)
        {
            Particle::collision[i][j] = false;
            Particle::collisionDistance[i][j] = std::numeric_limits<double>::infinity();
        }
    }

    Machine::machines.push_back(new Walls(Settings::ARENA_DIAMETER / 2));
//    Machine::machines.push_back(
//        new Rope(
//            Vector(-6, 3, 0), Vector(6, 3, 0),
//            30, 5,
//            300, Settings::PARTICLE_RADIUS * 2, 120
//        )
//    );
//    Machine::machines.push_back(
//        new Cloth(
//            Vector(-2, -2, 0), Vector(2, 2, 0),
//            6, 300, 0.001, 12, -1
//        )
//    );
}

void SimulationWindow::initialize()
{
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    refreshRate = screen()->refreshRate(); // frame // fps // frequency // period
    QCursor::setPos(geometry().x() + width()/2, geometry().y() + height()/2);

//    Matrices::viewMatrix.translate(QVector3D(10, 10, 10));
    Matrices::camTZ = 12;
    Matrices::camRY = 0;
    Matrices::projectionMatrix.setToIdentity();
    Matrices::projectionMatrix.perspective(90.f, width()/height(), 0.1f, 100.f);

    Shader::currentShader = new Shader();
    Computer::currentComputer = new Computer();

    prepareSimulation();
}

void SimulationWindow::render()
{
    currentTime     = frameTimer->diff();
    frame_dt        = currentTime - frameStartTime;
    frameStartTime  = currentTime;

    glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);

    //Form::printForms();

    if (Interaction::rewind)
    {
        frame_dt *= -1;
    }

    if (! Interaction::pause || Interaction::key[RENDER]) {
        std::cout << "-- Frame " << frame-1 << " end ---------------------------" << std::endl << std::endl;
        std::cout << "-- Frame " << frame << " start --------------------------" << std::endl;
    }

    #pragma omp parallel if(Settings::PARALLEL_OMP)
    {
        #pragma omp master
        if (! Interaction::pause || Interaction::key[RENDER])
            std::cout << omp_get_num_threads() << " threads." << std::endl;
    }

    Interaction::interact(); //////////////////////////////////////////////
    if (! Interaction::pause || Interaction::key[RENDER])
    {
        for (unsigned i = 0; i < Settings::ITERATIONS_PER_FRAME; ++i)
        {
            Computer::currentComputer->loop();
        }
    }

    Timer drawingTimer = Timer();
    long long int drawingStartTime = 0;
    drawingStartTime = drawingTimer.diff();
    {
        Shader::currentShader->program->bind();
        {
            Matrices::viewMatrix.setToIdentity();
            Matrices::viewProjectionMatrix.setToIdentity();
            Matrices::viewProjectionInverted.setToIdentity();
            Matrices::setViewMatrix();
            Matrices::viewProjectionMatrix
                = Matrices::projectionMatrix * Matrices::viewMatrix;
            Matrices::viewProjectionInverted
                = Matrices::viewProjectionMatrix.inverted();

    //#pragma omp parallel for if(Settings::PARALLEL_OMP)
            for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
            {
                Particle::flows[0][i]->paint();
            }
    //#pragma omp parallel for if(Settings::PARALLEL_OMP)
            for (unsigned i = 0; i < Machine::machines.size(); ++i)
            {
                Machine::machines[i]->paint();
            }
        }
        Shader::currentShader->program->release();
    }
    if (! Interaction::pause || Interaction::key[RENDER])
    {
        std::cout << (drawingTimer.diff() - drawingStartTime)/1000000
                  << "ms <- Frame drawing time." << std::endl;
    }

    if (! Interaction::pause)
    {
        if (! Settings::NO_SCREENS_NO_VIDEO)
        {
            Timer screeningTimer = Timer();
            long long int screeningStartTime = 0;
            screeningStartTime = screeningTimer.diff();
            {
                QPixmap pixMap = screen()->grabWindow(0, x(), y(), width(), height());
                QImage img = pixMap.toImage();
                //screens.push_back(img);
                QString dst = Strings::DIR_FRAMES
                        + QString("frame_%1").arg(frame) + Strings::IMG_FILE_TYPE;
                img.save(dst);
            }
            if (! Interaction::pause || Interaction::key[RENDER])
            {
                std::cout << (screeningTimer.diff() - screeningStartTime)/1000000
                          << "ms <- Screen taking time." << std::endl;
            }
        }
    }

    if (! Interaction::pause || Interaction::key[RENDER])
    {
        currentTime = frameTimer->diff();
        frame_dt    = currentTime - frameStartTime;
        std::cout << round(frame_dt/1000000.0) << "ms <- Frame time." << std::endl;
        std::cout << "~" << round(1000000000/frame_dt) << " FPS" << std::endl;

        ++frame;
    }
}

void SimulationWindow::keyPressEvent(QKeyEvent *e)
{
    Interaction::keyPress(e);
}

void SimulationWindow::keyReleaseEvent(QKeyEvent *event)
{
    Interaction::keyRelease(event);
}

void SimulationWindow::mouseMoveEvent(QMouseEvent *event)
{
    Interaction::mouseMove(event);
}

void SimulationWindow::mousePressEvent(QMouseEvent *event)
{
    Interaction::mousePress(event);
}
void SimulationWindow::mouseReleaseEvent(QMouseEvent *event)
{
    Interaction::mouseRelease(event);
}

// Eliminates console output on OpenCV cv::error() calls after failed
//   OpenCV assertions.
int handleCVError(int status, const char *func_name,
                  const char *err_msg, const char *file_name,
                  int line, void *userdata)
{
    std::cout << "Failed writing video frame." << std::endl << std::flush;
    std::cout << err_msg << std::endl << std::flush;

    return 0;
}

void SimulationWindow::saveVideo()
{
    QString path = Strings::DIR_FRAMES;
    QDir dir(path);
    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);
    unsigned files_count = dir.count();
    for (unsigned i = 0; i < files_count; ++i)
    {
        QString frame_path = Strings::DIR_FRAMES + QString("frame_%1").arg(i)
                             + Strings::IMG_FILE_TYPE;
        screens.push_back(QImage(frame_path));
    }

    std::vector<cv::Mat> mats;
    for (unsigned i = 0; i < screens.size(); ++i) {
        // QImage to OpenCV cv::Mat
        cv::Mat tmp_mat(
            screens[i].height(), screens[i].width(), CV_8UC3,
            (uchar*) screens[i].bits(), screens[i].bytesPerLine()
        );
        cv::Mat fin_mat;
        cv::cvtColor(tmp_mat, fin_mat, CV_BGR2RGB);
        QString frame_path = Strings::DIR_FRAMES + QString("frame_%1").arg(i)
                             + Strings::IMG_FILE_TYPE;
        fin_mat = cv::imread(frame_path.toStdString(), CV_LOAD_IMAGE_COLOR);
        mats.push_back(fin_mat);
    }

//    cv::namedWindow("flowsVideo", CV_WINDOW_AUTOSIZE);
//    cv::imshow("flowsVideo", mats[0]);

//        std::vector<int> compression_params;
//        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//        compression_params.push_back(9);
//        cv::imwrite(
//            (Strings::DIR_FRAMES + QString("flowsVideo_test_img") + Strings::IMG_FILE_TYPE).toStdString(),
//            mats[0], compression_params
//        );

//    try {
        cv::VideoWriter video = cv::VideoWriter(
            (Strings::DIR_FRAMES + QString("flowsVideo.avi")).toStdString(),
            CV_FOURCC('M', 'J', 'P', 'G'), Settings::VIDEO_FILE_FPS,
            cv::Size(screens[0].width(), screens[0].height()),
            true
        );
        cv::redirectError(handleCVError);
        {
            unsigned failedFrames = 0;
            for (unsigned i = 0; i < mats.size(); ++i) {
                try
                {
                    video.write(mats[i]);
                }
                catch (...) { ++failedFrames; continue; }
            }
            std::cout << failedFrames << " <- failed frames count." << std::endl << std::flush;
        }
        cv::redirectError(nullptr);
        video.release();
        //cv::destroyAllWindows();
//    }
//    catch (runtime_error &ex) {
//        fprintf(stderr, "Exception: %s\n", ex.what());
//    }
}
