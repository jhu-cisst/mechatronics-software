/* File : amp1394.i */
%module amp1394
%{
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

void print_frame(unsigned char* buffer, int length);
uint32_t bswap32(uint32_t in);


%include stdint.i
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


class Eth1394Port
{
public:
    //enum { MAX_NODES = 15 };  // max number of boards, limited by rotary switch
    typedef bool (*Eth1394CallbackType)(Eth1394Port &port, unsigned char boardId, std::ostream &debugStream);

public:
    Eth1394Port(int portNum, std::ostream &debugStream = std::cerr, Eth1394CallbackType cb = 0);
    ~Eth1394Port();

    bool IsOK(void);

    void Reset(void);

    bool AddBoard(BoardIO *board);
    bool RemoveBoard(unsigned char boardId);    

    int GetNodeId(unsigned char boardId) const;
    bool ReadAllBoards(void);
    virtual bool ReadAllBoardsBroadcast(void);
    bool WriteAllBoards(void);
    virtual bool WriteAllBoardsBroadcast(void);
    
    // Read a quadlet
    %apply quadlet_t& ARGOUT_QUADLET_T {quadlet_t &data};
    bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data);

    // Write a quadlet
    bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data);

    // int eth1394_write_nodeidmode(int mode);
};


class FirewirePort
{
 public:
    FirewirePort(int portNum, std::ostream &debugStream = std::cerr);
    ~FirewirePort();

    int NumberOfUsers(void);
    int GetNumOfNodes(void){return NumOfNodes_;}
    bool IsOK(void) { return (handle != NULL); }

    bool AddBoard(BoardIO *board);
    bool RemoveBoard(unsigned char boardId);

    int GetNodeId(unsigned char boardId) const;
    unsigned long GetFirmwareVersion(unsigned char boardId) const;

    %apply quadlet_t& ARGOUT_QUADLET_T {quadlet_t &data};
    bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data);
    bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data);

    bool WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data);

    // Stop Cycle Start Packets
    void StopCycleStartPacket(void);
};

