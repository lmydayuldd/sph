#ifndef CUDA_CU_H
#define CUDA_CU_H

#include <cuda.h>
#include <builtin_types.h> // __global__ __host__ __device__

#define UNIFIED_MATH_CUDA_H
#include <vector_functions.h>
#include <math_functions.h>
#include <cuda_runtime.h> // cudaMalloc(), cudaMemcpy(), ...

//#include <thrust/device_vector.h>
//#include <thrust/host_vector.h>

#include <vector>
//#ifdef COMPILER_MSVC
#include <algorithm>
//#endif

#include "machine/particle.h"
#include "physics/forces.h"
#include "physics/vector.h"
#include "util/settings.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////
//// Interface for outside world. //////////////////////////////////////////////
//////// Kernel (= global = host + device) functions. //////////////////////////
////////////////////////////////////////////////////////////////////////////////

__forceinline __device__ double3 operator-(const double3 &v1, const double3 &v2) {
    return make_double3(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z);
}

__forceinline __device__ double3 operator+(const double3 &v1, const double3 &v2) {
    return make_double3(v1.x+v2.x, v1.y+v2.y, v1.z+v2.z);
}

__forceinline __device__ double3 operator/(const double3 &v, const double &x) {
    return make_double3(v.x/x, v.y/x, v.z/x);
}

__forceinline __device__ double3 operator*(const double3 &v, const double &x) {
    return make_double3(v.x*x, v.y*x, v.z*x);
}

__forceinline __device__ double dot(const double3 &v1, const double3 &v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

__forceinline __device__ double norm(const double3 &v) {
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
                                          unsigned &stationary1, unsigned &stationary2)
{
    //double3 sphereNormal;
    double3 p1_r = make_double3(rx1, ry1, rz1);
    double3 p2_r = make_double3(rx2, ry2, rz2);
    double3 p1_v = make_double3(vx1, vy1, vz1);
    double3 p2_v = make_double3(vx2, vy2, vz2);
    double3 sphereNormal = normal(p1_r - p2_r);
    double distanceBorder = distance(p1_r, p2_r) - radius - radius;
    if (distanceBorder < 0.) {
//        if (! stationary2)
        {
            p2_r = p2_r + sphereNormal * distanceBorder;
        }

        double3 p2_v_old = make_double3(p2_v.x, p2_v.y, p2_v.z);
//        if (! stationary2)
        {
            p2_v = p2_v -
                (p2_r - p1_r) * (
                    dot(p2_v - p1_v, p2_r - p1_r)
                    / pow(distance(p2_r, p1_r), 2)
                    * 2 * m1 / (m1 + m2)
                );
    //                    *p2.v *= (1. - Settings::WATER_DAMPENING);
        }
//        if (! stationary1)
        {
            p1_v = p1_v -
                (p1_r - p2_r) * (
                    dot(p1_v - p2_v_old, p1_r - p2_r)
                    / pow(distance(p1_r, p2_r), 2)
                    * 2 * m2 / (m1 + m2)
                );
            //*p1.v *= (1. - Settings::WATER_DAMPENING);
        }
//        if (! stationary1) {
//    //                    *p1.r += *p1.v * (Settings::dt - exactCollisionTime);//(1. - exactCollisionTime); //// ??
//    //                    *p1.r_former = *p1.r;
//        }
//        if (! stationary2) {
//    //                    *p2.r += *p2.v * (Settings::dt - exactCollisionTime);//(1. - exactCollisionTime); //// ??
//    //                    *p2.r_former = *p2.r;
//        }
    }

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
}

__global__ void collideAllKernel(unsigned particle_count,
                                 double *rx, double *ry, double *rz,
                                 double *vx, double *vy, double *vz,
                                 double *m,
                                 unsigned *stationary,
                                 double radius)
{
//    printf("Block %d, thread %d, block dim %d, grid dim %d.\n",
//           blockIdx.x, threadIdx.x, blockDim.x, gridDim.x);

//    for (unsigned i = 0; i < particle_count; ++i) {
//        for (unsigned j = i+1; j < particle_count; ++j) {
//            collidePair(rx[i], ry[i], rz[i], vx[i], vy[i], vz[i],
//                        rx[j], ry[j], rz[j], vx[j], vy[j], vz[j],
//                        m[i], m[j], radius, stationary[i], stationary[j]);
//        }
//    }

    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    //unsigned j = blockIdx.y * blockDim.y + threadIdx.y;
//    for (unsigned i = 0; i < particle_count; ++i)
        for (unsigned j = i+1; j < particle_count; ++j)
            collidePair(rx[i], ry[i], rz[i], vx[i], vy[i], vz[i],
                        rx[j], ry[j], rz[j], vx[j], vy[j], vz[j],
                        m[i], m[j], radius, stationary[i], stationary[j]);
}

void GPUCollideAll()
{
//    1152 cores
//    1024 threads per block
//    cuDeviceGet...(
//    CUDA Device Name           GeForce GTX 1060
//    Compute Capability         6.1
//    Processor Count            10
//    Cores per Processor        128
//    Threads per Multiprocessor 2048
//    Warp Size                  32 Threads

    int blocksPerGrid = Settings::PARTICLE_COUNT / 8;
    int threadsPerBlock = Settings::PARTICLE_COUNT / blocksPerGrid;
//    dim3 blocksPerGrid(512);          // 512 x 1 x 1
//    dim3 threadsPerBlock(1024, 1024); // 1024 x 1024 x 1

    std::vector<double> rx(Settings::PARTICLE_COUNT);
    std::vector<double> ry(Settings::PARTICLE_COUNT);
    std::vector<double> rz(Settings::PARTICLE_COUNT);
    std::vector<double> vx(Settings::PARTICLE_COUNT);
    std::vector<double> vy(Settings::PARTICLE_COUNT);
    std::vector<double> vz(Settings::PARTICLE_COUNT);
    std::vector<double> m(Settings::PARTICLE_COUNT);
    std::vector<unsigned> stationary(Settings::PARTICLE_COUNT);
    for (unsigned int i = 0; i < Settings::PARTICLE_COUNT; ++i) {
        rx[i]         = Particle::flows[0][i]->r->x;
        ry[i]         = Particle::flows[0][i]->r->y;
        rz[i]         = Particle::flows[0][i]->r->z;
        vx[i]         = Particle::flows[0][i]->v->x;
        vy[i]         = Particle::flows[0][i]->v->y;
        vz[i]         = Particle::flows[0][i]->v->z;
        m[i]          = Particle::flows[0][i]->m;
        stationary[i] = (unsigned) Particle::flows[0][i]->stationary;
    }

    double *rx_device = 0;
    cudaMalloc((void**)&rx_device, rx.size() * sizeof(double));
    cudaMemcpy(rx_device, rx.data(), rx.size() * sizeof(double), cudaMemcpyHostToDevice);

    double *ry_device = 0;
    cudaMalloc((void**)&ry_device, ry.size() * sizeof(double));
    cudaMemcpy(ry_device, ry.data(), ry.size() * sizeof(double), cudaMemcpyHostToDevice);

    double *rz_device = 0;
    cudaMalloc((void**)&rz_device, rz.size() * sizeof(double));
    cudaMemcpy(rz_device, rz.data(), rz.size() * sizeof(double), cudaMemcpyHostToDevice);

    double *vx_device = 0;
    cudaMalloc((void**)&vx_device, vx.size() * sizeof(double));
    cudaMemcpy(vx_device, vx.data(), vx.size() * sizeof(double), cudaMemcpyHostToDevice);

    double *vy_device = 0;
    cudaMalloc((void**)&vy_device, vy.size() * sizeof(double));
    cudaMemcpy(vy_device, vy.data(), vy.size() * sizeof(double), cudaMemcpyHostToDevice);

    double *vz_device = 0;
    cudaMalloc((void**)&vz_device, vz.size() * sizeof(double));
    cudaMemcpy(vz_device, vz.data(), vz.size() * sizeof(double), cudaMemcpyHostToDevice);

    double *m_device = 0;
    cudaMalloc((void**)&m_device, m.size() * sizeof(double));
    cudaMemcpy(m_device, m.data(), m.size() * sizeof(double), cudaMemcpyHostToDevice);

    unsigned *stationary_device = 0;
    cudaMalloc((void**)&stationary_device, stationary.size() * sizeof(unsigned));
    cudaMemcpy(stationary_device, stationary.data(), stationary.size() * sizeof(unsigned), cudaMemcpyHostToDevice);

    collideAllKernel<<<blocksPerGrid, threadsPerBlock>>>(
                                          Settings::PARTICLE_COUNT,
                                          rx_device, ry_device, rz_device,
                                          vx_device, vy_device, vz_device,
                                          m_device,
                                          stationary_device,
                                          Settings::PARTICLE_RADIUS);
    cudaDeviceSynchronize();

    cudaMemcpy(&rx[0], rx_device, rx.size() * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&ry[0], ry_device, ry.size() * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&rz[0], rz_device, rz.size() * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&vx[0], vx_device, vx.size() * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&vy[0], vy_device, vy.size() * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&vz[0], vz_device, vz.size() * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&m[0], m_device, m.size() * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(&stationary[0], stationary_device, stationary.size() * sizeof(unsigned), cudaMemcpyDeviceToHost);

    for (unsigned i = 0; i < Settings::PARTICLE_COUNT; ++i) {
        Particle::flows[0][i]->r->x = rx[i];
        Particle::flows[0][i]->r->y = ry[i];
        Particle::flows[0][i]->r->z = rz[i];
        Particle::flows[0][i]->v->x = vx[i];
        Particle::flows[0][i]->v->y = vy[i];
        Particle::flows[0][i]->v->z = vz[i];
    }

    cudaFree(rx_device);
    cudaFree(ry_device);
    cudaFree(rz_device);
    cudaFree(vx_device);
    cudaFree(vy_device);
    cudaFree(vz_device);
    cudaFree(m_device);
    cudaFree(stationary_device);
}

#endif // CUDA_CU_H
