/* File : amp1394.i */
%module amp1394
%{
#define SWIG_FILE_WITH_INIT
    
/*Put headers and other declarations here*/
#include "AmpIO.h"
#include "Eth1394Port.h"
#include "FirewirePort.h"
#include <byteswap.h>
extern void print_frame(unsigned char* buffer, int length);

/*Provide bswap_32*/
uint32_t bswap32(uint32_t in) {
    return bswap_32(in);
}
%}

%include stdint.i
%include numpy.i

%init %{
    import_array();
%}

void print_frame(unsigned char* buffer, int length);
uint32_t bswap32(uint32_t in);


typedef unsigned long int	nodeaddr_t;
typedef unsigned int		quadlet_t;
typedef unsigned int		uint32_t;


/*%apply int *INPUT { Int32 *in };*/
%typemap(in,numinputs=0)
    (quadlet_t& ARGOUT_QUADLET_T)
    (quadlet_t temp)
{
    $1 = &temp;
}
%typemap(argout) (quadlet_t& ARGOUT_QUADLET_T)
{
    $result = SWIG_Python_AppendOutput($result, SWIG_From_unsigned_SS_int(*arg$argnum));
}

%typemap(in,numinputs=0)
    (int32_t& ARGOUT_INT32_T)
    (int32_t temp)
{
    $1 = &temp;
}
%typemap(argout) (int32_t& ARGOUT_INT32_T)
{
    $result = SWIG_Python_AppendOutput($result, SWIG_From_int(*arg$argnum));
}

%typemap(in,numinputs=0)
    (uint16_t& ARGOUT_UINT16_T)
    (uint16_t temp)
{
    $1 = &temp;
}
%typemap(argout) (uint16_t& ARGOUT_UINT16_T)
{
    $result = SWIG_Python_AppendOutput($result, SWIG_From_unsigned_SS_short(*arg$argnum));
}


// AmpIO class
// ReadDoutControl
%apply AmpIO_UInt16& ARGOUT_UINT16_T {AmpIO_UInt16 &countsHigh};
%apply AmpIO_UInt16& ARGOUT_UINT16_T {AmpIO_UInt16 &countsLow};
// ReadEncoderPreload
%apply AmpIO_Int32& ARGOUT_INT32_T {AmpIO_Int32 &sdata}; 
%include "AmpIO.h"


%apply quadlet_t& ARGOUT_QUADLET_T {quadlet_t &data};
%apply (unsigned int* ARGOUT_ARRAY1, int DIM1) {(quadlet_t *rdata, unsigned int nbytes)};
%apply (unsigned int* IN_ARRAY1, int DIM1) {(quadlet_t *wdata, unsigned int nbytes)};
%include "Eth1394Port.h"
%include "FirewirePort.h"

