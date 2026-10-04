/*
 * armcheck.c - cross-compile sanity check for the DE1-SoC (Cyclone V HPS)
 *
 * Build:
 *   arm-linux-gnueabihf-gcc -O2 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard \
 *       armcheck.c -o armcheck
 *
 * Static build (use if the board's glibc is older than the toolchain's):
 *   arm-linux-gnueabihf-gcc -O2 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard \
 *       -static armcheck.c -o armcheck
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/utsname.h>

#ifdef __GLIBC__
#include <gnu/libc-version.h>
#endif

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

static void rule(const char *title)
{
    printf("\n== %s ==\n", title);
}

int main(void)
{
    struct utsname u;
    uint32_t probe = 0x01020304;

    rule("compile-time identity");

#if defined(__arm__)
    printf("  arch           : ARM (32-bit)\n");
#elif defined(__aarch64__)
    printf("  arch           : AArch64  <-- not what the DE1-SoC HPS wants\n");
#elif defined(__x86_64__)
    printf("  arch           : x86-64   <-- YOU BUILT WITH THE HOST GCC\n");
#else
    printf("  arch           : unknown\n");
#endif

#ifdef __ARM_ARCH
    printf("  ARM arch rev   : v%d\n", __ARM_ARCH);
#endif

#if defined(__ARM_PCS_VFP)
    printf("  float ABI      : hard  (correct for gnueabihf)\n");
#else
    printf("  float ABI      : soft  <-- MISMATCH with an armhf rootfs\n");
#endif

#ifdef __ARM_NEON
    printf("  NEON           : compiled in\n");
#else
    printf("  NEON           : not compiled in\n");
#endif

    printf("  compiler       : gcc %d.%d.%d\n",
           __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);

#ifdef __GLIBC__
    printf("  glibc headers  : %d.%d\n", __GLIBC__, __GLIBC_MINOR__);
#endif

    rule("runtime environment");

#ifdef __GLIBC__
    printf("  glibc runtime  : %s\n", gnu_get_libc_version());
    printf("                   (if this differs wildly from the header\n"
           "                    version above, expect symbol errors)\n");
#endif

    if (uname(&u) == 0) {
        printf("  kernel         : %s %s\n", u.sysname, u.release);
        printf("  machine        : %s\n", u.machine);
        printf("  hostname       : %s\n", u.nodename);
    }

    printf("  CPUs online    : %ld\n", sysconf(_SC_NPROCESSORS_ONLN));
    printf("  page size      : %ld bytes\n", sysconf(_SC_PAGESIZE));

    rule("data model");

    printf("  sizeof(int)    : %zu\n", sizeof(int));
    printf("  sizeof(long)   : %zu\n", sizeof(long));
    printf("  sizeof(void *) : %zu\n", sizeof(void *));
    printf("  endianness     : %s\n",
           (*(uint8_t *)&probe == 0x04) ? "little (expected)" : "big");

    rule("arithmetic");

    {
        /* Exercises the FPU. On a soft-float/hard-float mismatch this is
         * where things usually go sideways rather than at load time. */
        double acc = 0.0;
        int i;
        for (i = 1; i <= 1000000; i++)
            acc += 1.0 / ((double)i * (double)i);
        printf("  sum 1/n^2      : %.9f  (pi^2/6 = 1.644934067)\n", acc);
    }

#ifdef __ARM_NEON
    {
        /* Four lanes at once; if NEON is absent this traps as SIGILL. */
        uint32x4_t a = vdupq_n_u32(7);
        uint32x4_t b = vdupq_n_u32(6);
        uint32x4_t c = vmulq_u32(a, b);
        uint32_t out[4];
        vst1q_u32(out, c);
        printf("  NEON 7*6 x4    : %u %u %u %u\n",
               out[0], out[1], out[2], out[3]);
    }
#endif

    rule("result");
    printf("  If you are reading this on the DE1-SoC, the toolchain,\n");
    printf("  the float ABI and the C library all line up. Next step:\n");
    printf("  mmap /dev/mem for the lightweight HPS-to-FPGA bridge.\n\n");

    return 0;
}
