/******************************************************************************
 *
 * This is a minimal Linux command line program that reads/writes blocks of
 * data from/to the 1394-based controller. As a special case, if the program
 * name (argv[0]) contains quad1394, it reads/write one quadlet (32-bit value).
 *
 * Compile: gcc -Wall -lraw1394 <name of this file> -o <name of executable>
 * Usage: <name of executable> [-pP] [-nN] address [size] [value1, ....]
 *     P       - IEEE-1394 port
 *     N       - address of node to talk to (1394 node ID, not board switch)
 *     size    - number of quadlets to read or write (1=quadlet transfer)
 *     address - address, in hex, to read from or write to (quadlets only)
 *     valuen  - one or more quadlet values to write, in hex
 *               read operation performed if none specified
 *               values autofilled if the number of values < size argument
 * Returns: for read operation, list of quadlet values read, one per line
 *
 * Notes:
 * - Block reads/writes apply only to hardwired real-time data registers
 * - Quadlet reads/writes can access any address, though some are read-only
 * - Node IDs are assigned automatically: n slaves get IDs 0 to n-1, PC ID=n
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <byteswap.h>
#include <string.h>  // strerror
#include <errno.h>   // errno message

// libraw1394
#include <libraw1394/raw1394.h>
#include <libraw1394/csr.h>    //1394 CSR constants

raw1394handle_t handle;

/* bus reset handler updates the bus generation */
int reset_handler(raw1394handle_t hdl, unsigned int gen) {
    int id = raw1394_get_local_id(hdl);
    printf("Bus reset to gen %d, local id %d\n", gen, id);
    raw1394_update_generation(hdl, gen);
    return 0;
}

/* signal handler cleans up and exits the program */
void signal_handler(int sig) {
    signal(SIGINT, SIG_DFL);
    raw1394_destroy_handle(handle);
    exit(0);
}

/*******************************************************************************
 * main program
 */
int main(int argc, char** argv)
{
    int i, j, rc, args_found;
    int nports;
    int port;
    nodeid_t node;
    nodeaddr_t addr;
    int size;
    quadlet_t data1;
    quadlet_t *data = &data1;

    signal(SIGINT, signal_handler);

    int isQuad1394 = (strstr(argv[0], "quad1394") != 0);
    int isBroadcast = 0;
    int isDebug = 0;

    port = 0;
    node = 0;
    size = 1;
    j = 0;
    args_found = 0;
    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                port = atoi(argv[i]+2);
                printf("Selecting port %d\n", port);
            }
            else if (argv[i][1] == 'n') {
                node = atoi(argv[i]+2);
                printf("Selecting node %d\n", node);
            }
            else if (argv[i][1] == 'd') {
                isDebug = 1;
            }
        }
        else {
            if (args_found == 0)
                addr = strtoull(argv[i], 0, 16);
            else if ((args_found == 1) && (isQuad1394))
                data1 = bswap_32(strtoul(argv[i], 0, 16));
            else if ((args_found == 1) && (!isQuad1394)) {
                size = strtoul(argv[i], 0, 10);
                /* Allocate data array, initializing contents to 0 */
                data = (quadlet_t *) calloc(sizeof(quadlet_t), size);
                if (!data) {
                    fprintf(stderr, "Failed to allocate memory for %d quadlets", size);
                    exit(-1);
                }
            }
            else if (!isQuad1394 && (j < size))
                data[j++] = bswap_32(strtoul(argv[i], 0, 16));
            else
                fprintf(stderr, "Warning: extra parameter: %s\n", argv[i]);

            args_found++;
        }
    }

    if (args_found < 1) {
        if (isQuad1394)
            printf("Usage: %s [-pP] [-nN] <address in hex> [value to write in hex]\n", argv[0]);
        else
            printf("Usage: %s [-pP] [-nN] <address in hex> <size in quadlets> [write data quadlets in hex]\n", argv[0]);
        printf("       where P = port number, N = node number\n");
        exit(0);
    }

    /* get handle to device and check for errors */
    handle = raw1394_new_handle();
    if (handle == NULL) {
        fprintf(stderr, "**** Error: could not open 1394 handle\n");
        exit(-1);
    }

    /* set the bus reset handler */
    raw1394_set_bus_reset_handler(handle, reset_handler);

    /* get number of ports and check for errors */
    nports = raw1394_get_port_info(handle, NULL, 0);
    if (nports < 0) {
        fprintf(stderr, "**** Error: could not get port info\n");
        exit(-1);
    }
    if ((port < 0) || (port >= nports)) {
        fprintf(stderr, "**** Error: port %d does not exist (num ports = %d)\n", port, nports);
        exit(-1);
    }

    /* set handle to current port and skip errors */
    rc = raw1394_set_port(handle, port);
    if (rc) {
        fprintf(stderr, "**** Error setting port %d\n", port);
        exit(-1);
    }

    // get local id and printf libraw1394 version & local_node_id for debugging
    nodeid_t id = raw1394_get_local_id(handle);
    if (isDebug) {
        printf("libraw1394_version = %s  local_node_id = %x\n", raw1394_get_libversion(), id);
    }

    /* get number of nodes connected to current port/handle */
    int nnodes = raw1394_get_nodecount(handle);
    //    bool
    if (node == 63) {
        isBroadcast = 1;
        printf("**** Warning: Broadcasting message \n");
        printf("     Address should be larger than CSR_CONFIG_ROM_END(0x%llx)\n", CSR_CONFIG_ROM_END + CSR_REGISTER_BASE);
    } else if ((node < 0) || (node >= nnodes)) {
        fprintf(stderr, "**** Error: node %d does not exist (num nodes = %d)\n", node, nnodes);
        exit(-1);
    }
    nodeid_t target_node = (id & 0xFFC0)+node;

    /* determine whether to read or write based on args_found */
    if ((isQuad1394 && (args_found == 1)) ||
            (!isQuad1394 && (args_found <= 2)))
    {
        if (isBroadcast) {
            fprintf(stderr, "**** Error: broadcast read ONT supported on node %d (num nodes = %d)\n", node, nnodes);
        }

        /* read the data block and print out the values */
        rc = raw1394_read(handle, target_node, addr, size*4, data);
        if (!rc) {
            for (j=0; j<size; j++)
                printf("0x%08X\n", bswap_32(data[j]));
        }
    } else {
        /* for write */
        if (isBroadcast) {
            // broadcast, no ack is expected
            rc = raw1394_start_write(handle, target_node, addr, size*4, data, 11);
        } else {
            // asynchronous write
            rc = raw1394_write(handle, target_node, addr, size*4, data);
        }
    }
    if (rc) {
        raw1394_errcode_t errcode;
        errcode = raw1394_get_errcode(handle);
        fprintf(stderr, "**** Error: (0x%08X) errno = %d %s \n",
                errcode, errno, strerror(errno));
    }

    // Free memory if it was dynamically allocated
    if (data != &data1)
        free(data);

    raw1394_destroy_handle(handle);
    return 0;
}
