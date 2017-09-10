#ifndef MACROS_H
#define MACROS_H

#ifdef COMPILER_GPP
    #define LOOP_TYPE unsigned
#elif COMPILER_MSVC
    #define LOOP_TYPE int
#endif

#define LOG_TIME(MSG) \
    if (! Settings::NO_PRINTOUT) \
        if (! Interaction::pause || Interaction::key[RENDER]) \
            std::cout << (timer.diff() - formerTime)/1000000 << (MSG) << std::endl;

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

#endif // MACROS_H
