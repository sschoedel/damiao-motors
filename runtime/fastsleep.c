// LD_PRELOAD shim: clamp the DAMIAO SDK's 100 ms RX-poll sleeps to 2 ms.
//
// The closed libdm_device polls its USB RX path with nanosleep(100ms)
// (verified via strace: thousands of tv_nsec=100000000 sleeps), which
// batches all CAN feedback into ~100 ms clumps — far too slow for the
// 100 Hz balance controller (measured: feedback arrival p95 ~101 ms
// without the shim, ~10 ms with it, hardware timestamps unchanged).
// We intercept sleeps of EXACTLY 100 ms so deliberate sleeps of any
// other duration are untouched. Build + use via run_gui.sh.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <time.h>

#define MAGIC_NS 100000000L
#define FAST_NS    2000000L

static void clamp(struct timespec *t) {
    if (t && t->tv_sec == 0 && t->tv_nsec == MAGIC_NS)
        t->tv_nsec = FAST_NS;
}

int nanosleep(const struct timespec *req, struct timespec *rem) {
    static int (*real)(const struct timespec *, struct timespec *) = 0;
    if (!real) real = dlsym(RTLD_NEXT, "nanosleep");
    struct timespec r = *req;
    clamp(&r);
    return real(&r, rem);
}

int clock_nanosleep(clockid_t cid, int flags, const struct timespec *req,
                    struct timespec *rem) {
    static int (*real)(clockid_t, int, const struct timespec *,
                       struct timespec *) = 0;
    if (!real) real = dlsym(RTLD_NEXT, "clock_nanosleep");
    struct timespec r = *req;
    if (!(flags & 1)) clamp(&r);  /* only relative sleeps */
    return real(cid, flags, &r, rem);
}

int usleep(unsigned int usec) {
    static int (*real)(unsigned int) = 0;
    if (!real) real = dlsym(RTLD_NEXT, "usleep");
    return real(usec == 100000 ? 2000 : usec);
}
