/******************************************************************************
 *
 * This is a minimal Linux command line program that displays information
 * about the connected IEEE 1394 (Firewire) devices.
 *
 * Compile: gcc -Wall -lraw1394 <name of this file> -o <name of executable>
 * Usage: <name of executable>
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <libraw1394/raw1394.h>

/*******************************************************************************
 * main program
 */
int main(int argc, char** argv)
{
    raw1394handle_t handle;
    struct raw1394_portinfo *ports;
    int nports;
    int i, rc;

    /* get handle to device and check for errors */
    handle = raw1394_new_handle();
    if (handle == NULL) {
        fprintf(stderr, "**** Error: could not open 1394 handle\n");
        exit(-1);
    }

    /* get number of ports and check for errors */
    nports = raw1394_get_port_info(handle, NULL, 0);
    if (nports < 0) {
        fprintf(stderr, "**** Error: could not get port info\n");
        exit(-1);
    }

    ports = (struct raw1394_portinfo *) malloc(nports*sizeof(struct raw1394_portinfo));
    nports = raw1394_get_port_info(handle, ports, nports);
    for (i = 0; i < nports; i++) {
      printf("Port %d: %s (%d nodes)", i, ports[i].name, ports[i].nodes);
      if (i == 0) {
          rc = raw1394_set_port(handle, i);
          if (!rc) {
              int id = raw1394_get_local_id(handle);
              printf(", local id 0x%4X (%d)", id, id&0x3f);
          }
          printf("\n");
      }
    }
    free(ports);

    raw1394_destroy_handle(handle);
    return 0;
}
