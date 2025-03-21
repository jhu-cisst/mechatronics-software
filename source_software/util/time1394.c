/******************************************************************************
 *
 * This is a minimal Linux command line program that displays timing information
 * for reading/writing blocks of data over IEEE-1394.
 *
 * Compile: gcc -Wall -lraw1394 <name of this file> -o <name of executable>
 * Usage: <name of executable> <interval> <repetition>
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <libraw1394/raw1394.h>
#include <sys/time.h>

#define N 1024

raw1394handle_t handle;

void signal_handler(int sig) {
    signal(SIGINT, SIG_DFL);
    raw1394_destroy_handle(handle);
    exit(0);
}

double gettime_us(struct timeval* tv) {
    gettimeofday(tv, 0);
    return (double)tv->tv_usec + (double)1e6 * (double)tv->tv_sec;
}

int main(int argc, char** argv)
{
    struct raw1394_portinfo ports[4];
    int nports;
    int i, rc;
    double tr, tw;
    quadlet_t data[N];
    struct timeval tv;

    if (argc != 3) {
        printf("Usage: %s interval repetitions\n", argv[0]);
        return 0;
    }

    int sz_interval = strtoul(argv[1], 0, 10);
    int sz_q, reps;
    int num_reps = strtoul(argv[2], 0, 10);;

    signal(SIGINT, signal_handler);
    if (!(handle=raw1394_new_handle())) exit(4);
    nports = raw1394_get_port_info(handle, ports, 4);
    if (nports < 0) {
        fprintf(stderr, "**** Error: could not get port info\n");
        exit(-1);
    }

    for (i=0; i<nports; i++)
    {
        if (raw1394_set_port(handle, i) < 0) continue;
        rc = 0;

        printf("Selected port %d\n", i);

        printf("Data Size in Quadlets, Read Time (us), Write Time (us)\n");

        for (sz_q=2; sz_q<513; sz_q+=sz_interval) 
        {
            for (reps=0; reps<num_reps; reps++)
            {
                tr = gettime_us(&tv);
                rc |= raw1394_read(handle, 0xffc0, 0, 4*sz_q, data);
                tr = gettime_us(&tv) - tr;

                tw = gettime_us(&tv);
                rc |= raw1394_write(handle, 0xffc0, 0, 4*sz_q, data);
                tw = gettime_us(&tv) - tw;

                if (!rc)
                    printf("%4d,%5.0f,%5.0f\n", sz_q, tr, tw);
                else {
                    printf("error code: 0x%08X\n", raw1394_get_errcode(handle));
                    goto done;
                }
            }
        }
    }

done:
    raw1394_destroy_handle(handle);
    return 0;
}
