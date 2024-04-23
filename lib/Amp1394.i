/* File : Amp1394.i */
%module Amp1394Python
%{
#define SWIG_FILE_WITH_INIT
    
/*Put headers and other declarations here*/
#include "AmpIORevision.h"
#include "Amp1394Time.h"
#include "AmpIO.h"
#include "EthUdpPort.h"

#if Amp1394_HAS_RAW1394
  #include "FirewirePort.h"
#endif

#if Amp1394_HAS_PCAP
  #include "EthRawPort.h"
#endif

#if Amp1394_HAS_EMIO
  #include "ZynqEmioPort.h"
#endif

#include "PortFactory.h"

#ifdef _MSC_VER
#include <stdlib.h>
inline uint16_t bswap_16(uint16_t data) { return _byteswap_ushort(data); }
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#else
#include <byteswap.h>
#endif

uint16_t bswap16(uint16_t in) {
    return bswap_16(in);
}

uint32_t bswap32(uint32_t in) {
    return bswap_32(in);
}

%}

%include stdint.i
%include std_string.i
%include numpy.i

%init %{
    import_array();
%}

uint16_t bswap16(uint16_t in);
uint32_t bswap32(uint32_t in);

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

// -------------------------
// IN_ARRAY
%typecheck(SWIG_TYPECHECK_DOUBLE_ARRAY,
           fragment="NumPy_Macros")
  (quadlet_t* IN_ARRAY1, unsigned int NBYTES)
{
  $1 = is_array($input) || PySequence_Check($input);
}
%typemap(in,
         fragment="NumPy_Fragments")
  (quadlet_t* IN_ARRAY1, unsigned int NBYTES)
  (PyArrayObject* array=NULL, int is_new_object=0)
{
  npy_intp size[1] = { -1 };
  array = obj_to_array_contiguous_allow_conversion($input,
                                                   NPY_UINT,
                                                   &is_new_object);
  if (!array || !require_dimensions(array, 1) ||
      !require_size(array, size, 1)) SWIG_fail;
  $1 = (quadlet_t*) array_data(array);
  $2 = (unsigned int) array_size(array,0) * 4;
}
%typemap(freearg)
  (quadlet_t* IN_ARRAY1, unsigned int NBYTES)
{
  if (is_new_object$argnum && array$argnum)
    { Py_DECREF(array$argnum); }
}

// ----------------------------
// ARGOUT_ARRAY
%typemap(in,numinputs=1,
         fragment="NumPy_Fragments")
  (quadlet_t* ARGOUT_ARRAY1, unsigned int NBYTES)
  (PyObject* array = NULL)
{
  npy_intp dims[1];
  if (!PyInt_Check($input))
  {
    const char* typestring = pytype_string($input);
    PyErr_Format(PyExc_TypeError,
                 "Int dimension expected.  '%s' given.",
                 typestring);
    SWIG_fail;
  }
  $2 = (unsigned int) PyInt_AsLong($input);
  dims[0] = (npy_intp) $2;
  array = PyArray_SimpleNew(1, dims, NPY_UINT);
  if (!array) SWIG_fail;
  $1 = (quadlet_t*) array_data(array);
  $2 = $2 * 4;
}
%typemap(argout)
  (quadlet_t* ARGOUT_ARRAY1, unsigned int NBYTES)
{
  $result = SWIG_Python_AppendOutput($result,(PyObject*)array$argnum);
}



// AmpIO class
%apply uint16_t& ARGOUT_UINT16_T {uint16_t &rdata};
// ReadDoutControl
%apply uint16_t& ARGOUT_UINT16_T {uint16_t &countsHigh};
%apply uint16_t& ARGOUT_UINT16_T {uint16_t &countsLow};
// ReadEncoderPreload
%apply int32_t& ARGOUT_INT32_T {int32_t &sdata};

%ignore FpgaIO::ReadKSZ8851Reg(uint8_t addr, uint8_t &rdata);
%ignore FpgaIO::WriteKSZ8851Reg(uint8_t, uint8_t const &);

%import "AmpIORevision.h"

%constant int VERSION_MAJOR = Amp1394_VERSION_MAJOR;
%constant int VERSION_MINOR = Amp1394_VERSION_MINOR;
%constant int VERSION_PATCH = Amp1394_VERSION_PATCH;
%constant std::string VERSION = Amp1394_VERSION;

%include "Amp1394Types.h"
%include "Amp1394Time.h"
%include "EncoderVelocity.h"
%include "BoardIO.h"
%include "FpgaIO.h"
%include "AmpIO.h"

%apply (int* IN_ARRAY1, int DIM1) {(int* data, int size)};
%apply quadlet_t& ARGOUT_QUADLET_T {quadlet_t &data};
%apply (quadlet_t* ARGOUT_ARRAY1, unsigned int NBYTES) {(quadlet_t *rdata, unsigned int nbytes)};
%apply (quadlet_t* IN_ARRAY1, unsigned int NBYTES) {(quadlet_t *wdata, unsigned int nbytes)};
%include "BasePort.h"
%include "EthBasePort.h"
%include "EthUdpPort.h"
#if Amp1394_HAS_RAW1394
  %include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
  %include "EthRawPort.h"
#endif
#if Amp1394_HAS_EMIO
  %include "ZynqEmioPort.h"
#endif
%include "PortFactory.h"
