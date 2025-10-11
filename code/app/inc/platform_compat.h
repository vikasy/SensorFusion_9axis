/*****
* Author: Vikas Yadav
* Date: 2020-2025
* Platform compatibility functions for Windows, macOS, and Linux
*/

#ifndef PLATFORM_COMPAT_H
#define PLATFORM_COMPAT_H

#include <time.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// Platform detection
#ifdef __APPLE__
    #include <TargetConditionals.h>
    #if TARGET_OS_MAC
        #define PLATFORM_MACOS 1
    #endif
#elif defined(_WIN32) || defined(_WIN64)
    #define PLATFORM_WINDOWS 1
#elif defined(__linux__)
    #define PLATFORM_LINUX 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Cross-platform sleep function
#ifdef PLATFORM_WINDOWS
    #include <windows.h>
    #define _sleep(ms) Sleep(ms)
#elif defined(PLATFORM_MACOS) || defined(PLATFORM_LINUX)
    #include <unistd.h>
    // Define _sleep function for Unix-like platforms (macOS/Linux)
    static inline void _sleep(unsigned int milliseconds) {
        struct timespec ts;
        ts.tv_sec = milliseconds / 1000;
        ts.tv_nsec = (milliseconds % 1000) * 1000000L;
        
        // Handle interrupted system calls properly
        while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
            // Continue sleeping if interrupted by a signal
        }
    }
#else
    // Fallback for other platforms
    #include <unistd.h>
    #define _sleep(ms) usleep((ms) * 1000)
#endif

// Cross-platform high-resolution timer
#ifdef PLATFORM_WINDOWS
    #include <windows.h>
    static inline int64_t get_time_ns(void) {
        LARGE_INTEGER frequency, counter;
        QueryPerformanceFrequency(&frequency);
        QueryPerformanceCounter(&counter);
        return (counter.QuadPart * 1000000000LL) / frequency.QuadPart;
    }
#elif defined(PLATFORM_MACOS)
    #include <mach/mach_time.h>
    static inline int64_t get_time_ns(void) {
        static mach_timebase_info_data_t info = {0, 0};
        if (info.denom == 0) {
            mach_timebase_info(&info);
        }
        uint64_t time = mach_absolute_time();
        return (int64_t)((time * info.numer) / info.denom);
    }
#elif defined(PLATFORM_LINUX)
    #include <time.h>
    static inline int64_t get_time_ns(void) {
        struct timespec ts;
        if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
            return 0; // Error handling
        }
        return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
    }
#else
    // Fallback for other platforms
    #include <time.h>
    static inline int64_t get_time_ns(void) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
    }
#endif

// Platform-specific memory alignment
#ifdef PLATFORM_MACOS
    // macOS prefers 16-byte alignment for SIMD operations
    #define PLATFORM_CACHE_LINE_SIZE 64
    #define PLATFORM_ALIGNMENT 16
    #define ALIGNED_MALLOC(size, alignment) aligned_alloc(alignment, size)
    #define ALIGNED_FREE(ptr) free(ptr)
#elif defined(PLATFORM_WINDOWS)
    #define PLATFORM_CACHE_LINE_SIZE 64
    #define PLATFORM_ALIGNMENT 8
    #define ALIGNED_MALLOC(size, alignment) _aligned_malloc(size, alignment)
    #define ALIGNED_FREE(ptr) _aligned_free(ptr)
#else
    #define PLATFORM_CACHE_LINE_SIZE 64
    #define PLATFORM_ALIGNMENT 8
    #define ALIGNED_MALLOC(size, alignment) aligned_alloc(alignment, size)
    #define ALIGNED_FREE(ptr) free(ptr)
#endif

// Platform-specific thread support
#ifdef PLATFORM_MACOS
    #include <pthread.h>
    #define THREAD_LOCAL __thread
#elif defined(PLATFORM_WINDOWS)
    #define THREAD_LOCAL __declspec(thread)
#else
    #define THREAD_LOCAL __thread
#endif

// Platform information function
static inline const char* get_platform_name(void) {
#ifdef PLATFORM_MACOS
    return "macOS";
#elif defined(PLATFORM_WINDOWS)
    return "Windows";
#elif defined(PLATFORM_LINUX)
    return "Linux";
#else
    return "Unknown";
#endif
}

// Platform capabilities information
static inline void print_platform_info(void) {
#ifdef PLATFORM_MACOS
    printf("Platform: macOS\n");
    printf("Timer: Mach absolute time\n");
    printf("Cache line size: %d bytes\n", PLATFORM_CACHE_LINE_SIZE);
    printf("Memory alignment: %d bytes\n", PLATFORM_ALIGNMENT);
#elif defined(PLATFORM_WINDOWS)
    printf("Platform: Windows\n");
    printf("Timer: QueryPerformanceCounter\n");
    printf("Cache line size: %d bytes\n", PLATFORM_CACHE_LINE_SIZE);
#elif defined(PLATFORM_LINUX)
    printf("Platform: Linux\n");
    printf("Timer: CLOCK_MONOTONIC\n");
    printf("Cache line size: %d bytes\n", PLATFORM_CACHE_LINE_SIZE);
#else
    printf("Platform: Unknown/Generic\n");
    printf("Timer: CLOCK_REALTIME fallback\n");
#endif
}

#ifdef __cplusplus
}
#endif

#endif // PLATFORM_COMPAT_H