#include <cuda.h>
#include <builtin_types.h> // __global__ __host__ __device__

#define UNIFIED_MATH_CUDA_H
#include <vector_functions.h>
#include <math_functions.h>
#include <cuda_runtime.h> // cudaMalloc(), cudaMemcpy(), ...
#include <helper_cuda.h>

//#include <thrust/device_vector.h>
//#include <thrust/host_vector.h>
//#include <thrust/system/cuda/experimental/pinned_allocator.h>

#include <vector>
//#ifdef COMPILER_MSVC
#include <algorithm>
//#endif

#include "machine/particle.h"
#include "physics/forces.h"
#include "physics/vector.h"
#include "util/settings.h"

#include <omp.h>

using namespace std;

#ifdef COMPILER_GPP
    #define LOOP_TYPE unsigned
#elif COMPILER_MSVC
    #define LOOP_TYPE int
#endif

#define CUDA_TIME_MEASUREMENT_INIT \
    cudaEvent_t startEvent, stopEvent; \
    float time; \
    int bytes;

#define CUDA_TIME_MEASUREMENT_START \
    checkCudaErrors(cudaEventCreate(&startEvent)); \
    checkCudaErrors(cudaEventCreate(&stopEvent)); \
    checkCudaErrors(cudaEventRecord(startEvent, 0));

#define CUDA_TIME_MEASUREMENT_END \
    checkCudaErrors(cudaEventRecord(stopEvent, 0)); \
    checkCudaErrors(cudaEventSynchronize(stopEvent)); \
    checkCudaErrors(cudaEventElapsedTime(&time, startEvent, stopEvent));

#define CUDA_TIME_MEASUREMENT_FIN \
    checkCudaErrors(cudaEventDestroy(startEvent)); \
    checkCudaErrors(cudaEventDestroy(stopEvent));

////////////////////////////////////////////////////////////////////////////////
//// Interface for outside world. //////////////////////////////////////////////
//////// Kernel (= global = host + device) functions. //////////////////////////
////////////////////////////////////////////////////////////////////////////////

__forceinline __device__ double3 operator+(const double3 &v1, const double3 &v2) {
    return make_double3(v1.x+v2.x, v1.y+v2.y, v1.z+v2.z);
}

__forceinline __device__ double3 operator-(const double3 &v1, const double3 &v2) {
    return make_double3(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z);
}

__forceinline __device__ double3 operator*(const double3 &v, const double &n) {
    return make_double3(v.x*n, v.y*n, v.z*n);
}

__forceinline __device__ double3 operator/(const double3 &v, const double &n) {
    return make_double3(v.x/n, v.y/n, v.z/n);
}

__forceinline __device__ double dot(const double3 &v1, const double3 &v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

__forceinline __device__ double norm(const double3 &v) {
    //return sqrt(dot(v, v));
    return norm3d(v.x, v.y, v.z);
}

__forceinline __device__ double distance(const double3 &v1, const double3 &v2) {
    double3 v3 = v1 - v2;
    return norm(v3);
}

__forceinline __device__ double3 normal(const double3 &v) {
    return v / norm(v);
}

__forceinline __device__ void collidePair(double &rx1, double &ry1, double &rz1,
                                          double &vx1, double &vy1, double &vz1,
                                          double &rx2, double &ry2, double &rz2,
                                          double &vx2, double &vy2, double &vz2,
                                          double &m1, double &m2,
                                          double &radius,
                                          bool &is_stationary_p1,
                                          bool &is_stationary_p2)
{
    //double3 sphereNormal;
    double3 p1_r = make_double3(rx1, ry1, rz1);
    double3 p2_r = make_double3(rx2, ry2, rz2);
    double3 p1_v = make_double3(vx1, vy1, vz1);
    double3 p2_v = make_double3(vx2, vy2, vz2);
    double3 sphereNormal = normal(p1_r - p2_r);
    double distanceBorder = distance(p1_r, p2_r) - radius - radius;
    if (distanceBorder < 0.) {
        if (! is_stationary_p2)
        {
            p2_r = p2_r + sphereNormal * distanceBorder;
        }

        double3 p2_v_old = make_double3(p2_v.x, p2_v.y, p2_v.z);
        if (! is_stationary_p2)
        {
            p2_v = p2_v -
                (p2_r - p1_r) * (
                    dot(p2_v - p1_v, p2_r - p1_r)
                    / pow(distance(p2_r, p1_r), 2)
                    * 2 * m1 / (m1 + m2)
                );
    //                    *p2.v *= (1. - Settings::WATER_DAMPENING);
        }
        if (! is_stationary_p1)
        {
            p1_v = p1_v -
                (p1_r - p2_r) * (
                    dot(p1_v - p2_v_old, p1_r - p2_r)
                    / pow(distance(p1_r, p2_r), 2)
                    * 2 * m2 / (m1 + m2)
                );
            //*p1.v *= (1. - Settings::WATER_DAMPENING);
        }
        if (! is_stationary_p1) {
//    //                    *p1.r += *p1.v * (Settings::dt - exactCollisionTime);//(1. - exactCollisionTime); //// ??
//    //                    *p1.r_former = *p1.r;
        }
        if (! is_stationary_p2) {
//    //                    *p2.r += *p2.v * (Settings::dt - exactCollisionTime);//(1. - exactCollisionTime); //// ??
//    //                    *p2.r_former = *p2.r;
        }
    }

   // __syncthreads();
    rx1 = p1_r.x;
    ry1 = p1_r.y;
    rz1 = p1_r.z;
    vx1 = p1_v.x;
    vy1 = p1_v.y;
    vz1 = p1_v.z;
    rx2 = p2_r.x;
    ry2 = p2_r.y;
    rz2 = p2_r.z;
    vx2 = p2_v.x;
    vy2 = p2_v.y;
    vz2 = p2_v.z;
//    __syncthreads();
}

__global__ void collideAllKernel(unsigned particle_count,
                                 double *rx, double *ry, double *rz,
                                 double *vx, double *vy, double *vz,
                                 double *m,
                                 double radius,
                                 bool *stationary, int i)
{
//    for (unsigned i = 0; i < particle_count; ++i) {
//        for (unsigned j = i+1; j < particle_count; ++j) {
//            collidePair(rx[i], ry[i], rz[i], vx[i], vy[i], vz[i],
//                        rx[j], ry[j], rz[j], vx[j], vy[j], vz[j],
//                        m[i], m[j], radius, stationary[i], stationary[j]);
//        }
//    }

//    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
//    //unsigned j = blockIdx.x * blockDim.x + threadIdx.x;
////    for (unsigned i = 0; i < particle_count; ++i)
//        for (unsigned j = i+1; j < particle_count; ++j)
//            collidePair(rx[i], ry[i], rz[i], vx[i], vy[i], vz[i],
//                        rx[j], ry[j], rz[j], vx[j], vy[j], vz[j],
//                        m[i], m[j], radius, stationary[i], stationary[j]);

//    printf("blockIdx.x %d, threadIdx.x %d, blockIdx.y %d, threadIdx.y %d, blockDim.x %d, blockDim.y %d, gridDim.x %d, gridDim.y %d.\n",
//           blockIdx.x, threadIdx.x, blockIdx.y, threadIdx.y, blockDim.x, blockDim.y, gridDim.x, gridDim.y);

//    unsigned i = blockIdx.x * blockDim.x * blockDim.y + threadIdx.x * blockDim.x + threadIdx.y;
//    unsigned j = i + blockIdx.x * blockDim.x + threadIdx.y;
//    for (unsigned int j = i+1; j < particle_count; ++j) {
    unsigned j = blockIdx.x * 100 + threadIdx.x * 10 + threadIdx.y;
    if (j > i)
        collidePair(rx[i], ry[i], rz[i], vx[i], vy[i], vz[i],
                    rx[j], ry[j], rz[j], vx[j], vy[j], vz[j],
                    m[i], m[j], radius, stationary[i], stationary[j]);
//    }

//    unsigned i = blockIdx.x;
//    unsigned j = blockIdx.y;
//    if (j > i)
//    {
//        collidePair(rx[i], ry[i], rz[i], vx[i], vy[i], vz[i],
//                    rx[j], ry[j], rz[j], vx[j], vy[j], vz[j],
//                    m[i], m[j], radius, stationary[i], stationary[j]);
//    }
}

void GPUCollideMallocs()
{
    static bool wasExecuted = false;
    if (! wasExecuted) {
        wasExecuted = true;

        // pinned host memory
        checkCudaErrors(cudaMallocHost((void**)&Particle::rx_host, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMallocHost((void**)&Particle::ry_host, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMallocHost((void**)&Particle::rz_host, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMallocHost((void**)&Particle::vx_host, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMallocHost((void**)&Particle::vy_host, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMallocHost((void**)&Particle::vz_host, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMallocHost((void**)&Particle::m_host, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMallocHost((void**)&Particle::is_stationary_host, Settings::PARTICLE_COUNT_2D * sizeof(bool)));

        checkCudaErrors(cudaMalloc((void**)&Particle::rx_device, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMalloc((void**)&Particle::ry_device, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMalloc((void**)&Particle::rz_device, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMalloc((void**)&Particle::vx_device, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMalloc((void**)&Particle::vy_device, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMalloc((void**)&Particle::vz_device, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMalloc((void**)&Particle::m_device, Settings::PARTICLE_COUNT_2D * sizeof(double)));
        checkCudaErrors(cudaMalloc((void**)&Particle::is_stationary_device, Settings::PARTICLE_COUNT_2D * sizeof(bool)));
    }
}

void GPUCollideFrees()
{
    // pinned host memory
    checkCudaErrors(cudaFreeHost(Particle::rx_host));
    checkCudaErrors(cudaFreeHost(Particle::ry_host));
    checkCudaErrors(cudaFreeHost(Particle::rz_host));
    checkCudaErrors(cudaFreeHost(Particle::vx_host));
    checkCudaErrors(cudaFreeHost(Particle::vy_host));
    checkCudaErrors(cudaFreeHost(Particle::vz_host));
    checkCudaErrors(cudaFreeHost(Particle::m_host));
    checkCudaErrors(cudaFreeHost(Particle::is_stationary_host));

    checkCudaErrors(cudaFree(Particle::rx_device));
    checkCudaErrors(cudaFree(Particle::ry_device));
    checkCudaErrors(cudaFree(Particle::rz_device));
    checkCudaErrors(cudaFree(Particle::vx_device));
    checkCudaErrors(cudaFree(Particle::vy_device));
    checkCudaErrors(cudaFree(Particle::vz_device));
    checkCudaErrors(cudaFree(Particle::m_device));
    checkCudaErrors(cudaFree(Particle::is_stationary_device));
}

void GPUCollideAll()
{    
    CUDA_TIME_MEASUREMENT_INIT

    CUDA_TIME_MEASUREMENT_START
    {
//        std::cout << Particle::rx_host[0] << std::endl << std::flush;
//        std::cout << Particle::flows[0][0]->r->x << std::endl << std::flush;
        #pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE i = 0; i < (LOOP_TYPE) Settings::PARTICLE_COUNT_2D; ++i) {
            Particle::rx_host[i]            = Particle::flows[0][i]->r->x;
            Particle::ry_host[i]            = Particle::flows[0][i]->r->y;
            Particle::rz_host[i]            = Particle::flows[0][i]->r->z;
            Particle::vx_host[i]            = Particle::flows[0][i]->v->x;
            Particle::vy_host[i]            = Particle::flows[0][i]->v->y;
            Particle::vz_host[i]            = Particle::flows[0][i]->v->z;
            Particle::m_host[i]             = Particle::flows[0][i]->m;
            Particle::is_stationary_host[i] = Particle::flows[0][i]->isStationary;
        }
    }
    CUDA_TIME_MEASUREMENT_END
    bytes = Settings::PARTICLE_COUNT_2D * (7 * sizeof(double) + 1 * sizeof(bool));
    printf("Host Vector to Host Vector bandwidth: %fGB / %fs = %fGB/s\n",
           bytes * 1e-9, time * 1e-3, (bytes * 1e-9) / (time * 1e-3));

    CUDA_TIME_MEASUREMENT_START
    {
        // pinned host memory
        checkCudaErrors(cudaMemcpy(Particle::rx_device, Particle::rx_host, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyHostToDevice));
        checkCudaErrors(cudaMemcpy(Particle::ry_device, Particle::ry_host, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyHostToDevice));
        checkCudaErrors(cudaMemcpy(Particle::rz_device, Particle::rz_host, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyHostToDevice));
        checkCudaErrors(cudaMemcpy(Particle::vx_device, Particle::vx_host, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyHostToDevice));
        checkCudaErrors(cudaMemcpy(Particle::vy_device, Particle::vy_host, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyHostToDevice));
        checkCudaErrors(cudaMemcpy(Particle::vz_device, Particle::vz_host, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyHostToDevice));
        checkCudaErrors(cudaMemcpy(Particle::m_device, Particle::m_host, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyHostToDevice));
        checkCudaErrors(cudaMemcpy(Particle::is_stationary_device, Particle::is_stationary_host, Settings::PARTICLE_COUNT_2D * sizeof(bool), cudaMemcpyHostToDevice));
    }
    CUDA_TIME_MEASUREMENT_END
    bytes = Settings::PARTICLE_COUNT_2D * (7 * sizeof(double) + 1 * sizeof(bool));
    printf("Host to CUDA Device bandwidth: %fGB / %fs = %fGB/s\n",
           bytes * 1e-9, time * 1e-3, (bytes * 1e-9) / (time * 1e-3));

    CUDA_TIME_MEASUREMENT_START
    {
        //    cuDeviceGet...(
        //    1152 cores
        //    1024 threads per block
        //    Compute Capability         6.1
        //    Processor Count            10
        //    Cores per Processor        128
        //    Threads per Multiprocessor 2048
        //    Warp Size                  32 Threads
        //    Block has at least one warp, with at least 32 threads.

        for (unsigned i = 0; i < Settings::PARTICLE_COUNT_2D; ++i) {
//            int blocksPerGrid = Settings::PARTICLE_COUNT / 50;
//            int threadsPerBlock = Settings::PARTICLE_COUNT / blocksPerGrid;
            dim3 blocksPerGrid(20); // ? x ? x ?
            dim3 threadsPerBlock(10, 10); // ? x ? x ?
//            dim3 blocksPerGrid(2000, 2000); // ? x ? x ?
//            dim3 threadsPerBlock(1); // ? x ? x ?
            collideAllKernel<<<blocksPerGrid, threadsPerBlock>>>(
                Settings::PARTICLE_COUNT_2D,
                Particle::rx_device, Particle::ry_device, Particle::rz_device,
                Particle::vx_device, Particle::vy_device, Particle::vz_device,
                Particle::m_device,
                Settings::PARTICLE_RADIUS,
                Particle::is_stationary_device, i
            );
        }
        checkCudaErrors(cudaDeviceSynchronize());
    }
    CUDA_TIME_MEASUREMENT_END
    printf("CUDA Collision Kernel execution time: %fs\n", time * 1e-3);

    CUDA_TIME_MEASUREMENT_START
    {
        // pinned host memory
        checkCudaErrors(cudaMemcpy(&Particle::rx_host[0], Particle::rx_device, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyDeviceToHost));
        checkCudaErrors(cudaMemcpy(&Particle::ry_host[0], Particle::ry_device, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyDeviceToHost));
        checkCudaErrors(cudaMemcpy(&Particle::rz_host[0], Particle::rz_device, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyDeviceToHost));
        checkCudaErrors(cudaMemcpy(&Particle::vx_host[0], Particle::vx_device, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyDeviceToHost));
        checkCudaErrors(cudaMemcpy(&Particle::vy_host[0], Particle::vy_device, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyDeviceToHost));
        checkCudaErrors(cudaMemcpy(&Particle::vz_host[0], Particle::vz_device, Settings::PARTICLE_COUNT_2D * sizeof(double), cudaMemcpyDeviceToHost));
    }
    CUDA_TIME_MEASUREMENT_END
    bytes = Settings::PARTICLE_COUNT_2D * (6 * sizeof(double));
    printf("CUDA Device to Host bandwidth: %fGB / %fs = %fGB/s\n",
           bytes * 1e-9, time * 1e-3, (bytes * 1e-9) / (time * 1e-3));

    CUDA_TIME_MEASUREMENT_START
    {
        #pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE i = 0; i < (LOOP_TYPE) Settings::PARTICLE_COUNT_2D; ++i) {
            Particle::flows[0][i]->r->x = Particle::rx_host[i];
            Particle::flows[0][i]->r->y = Particle::ry_host[i];
            Particle::flows[0][i]->r->z = Particle::rz_host[i];
            Particle::flows[0][i]->v->x = Particle::vx_host[i];
            Particle::flows[0][i]->v->y = Particle::vy_host[i];
            Particle::flows[0][i]->v->z = Particle::vz_host[i];
        }
    }
    CUDA_TIME_MEASUREMENT_END
    bytes = Settings::PARTICLE_COUNT_2D * (6 * sizeof(double));
    printf("Host Vector to Host Vector bandwidth: %fGB / %fs = %fGB/s\n",
           bytes * 1e-9, time * 1e-3, (bytes * 1e-9) / (time * 1e-3));

    CUDA_TIME_MEASUREMENT_FIN
}
