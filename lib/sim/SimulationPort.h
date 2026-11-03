#pragma once

#include <vector>
#include <queue>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <ostream>

#include "BoardIO.h"
#include "BasePort.h"

class SimulationPort : public BasePort
{
public:
    SimulationPort(int portNum, std::ostream &ostr = std::cerr);
    ~SimulationPort();

    PortType GetPortType(void) const { return PORT_SIMULATION; }
    int NumberOfUsers(void);
    bool IsOK(void) { return true; }
    unsigned int GetBusGeneration(void) const;
    void UpdateBusGeneration(unsigned int gen) {}

    unsigned int GetPrefixOffset(MsgType msg) const { return 0; }
    unsigned int GetWritePostfixSize(void) const { return 0; }
    unsigned int GetReadPostfixSize(void) const { return 0; }
    unsigned int GetWriteQuadAlign(void) const { return 0; }
    unsigned int GetReadQuadAlign(void) const { return 0; }
    unsigned int GetMaxReadDataSize(void) const { return 2048; }
    unsigned int GetMaxWriteDataSize(void) const { return 2048; }

    bool WriteBroadcastOutput(quadlet_t *buffer, unsigned int size);
    bool WriteBroadcastReadRequest(unsigned int seq);
    void WaitBroadcastRead(void);

    void PromDelay(void) const {}

    // Adds board(s)
    bool AddBoard(BoardIO *board);
    // Removes board
    bool RemoveBoard(unsigned char boardId);

protected:
    bool Init(void);
    void Cleanup(void);
    nodeid_t InitNodes(void);

    bool ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags = 0);
    bool WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags = 0);
    bool WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes, unsigned char flags = 0);
    bool ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata, unsigned int nbytes, unsigned char flags = 0);

private:
    struct DynamicsParams {
        // 2.0e-5 kg m^2 (approx Maxon RE40 rotor + gearhead)
        double motor_inertia = 0.001;
        // 1.0e-4 Nm/(rad/s)
        double viscous_damping = 0.1;
        // this must coincide with the NmToAmps scale in the XML config
        // where you will find the scale computed as 1/torque_constant
        double torque_constant = 0.1;
        // this must coincide with the BitsToPosSI scale in the XML config
        // where you will find the scale computed as 360/counts_per_turn
        double counts_per_turn = 100000.0;
        // this is the simulation time step in seconds for joint dynamics
        // it should be equal to the period of the control loop or at least
        // comparable with the bandwidth of the simulated joint
        double dynamics_dt_sec = 0.001;
        // let's use multi-step integration for better stability
        unsigned int integration_steps = 5;
        // Thermal model parameters [C]
        double ambient_temp_c = 25.0;
        // Reduce heating effect (less aggressive temperature rise) [C/s per A^2]
        double thermal_heating_coeff = 0.05;
        // Increase cooling (stronger dissipation) [1/s]
        double thermal_cooling_coeff = 0.02;
    };

    struct AxisState
    {
        // ENC_MIDRANGE
        int32_t EncoderPos = 0x800000;
        
        double EncoderVel = 0.0;
        double SimPosition = 0.0;
        double SimVelocity = 0.0;
        
        int32_t EncoderQtr1 = 0;
        int32_t EncoderQtr5 = 0;
        int32_t EncoderRun = 0;
        
        // ENC_MIDRANGE
        int32_t EncoderPreload = 0x800000;
        int32_t EncoderOffset = 0;
        
        // zero current
        uint32_t MotorCurrent = 32768;
        
        // Default to OFF
        uint32_t MotorStatus = 0x00000000;
        
        // Simple temperature state [C]
        double TemperatureC = 25.0;
        
        // Per-axis dynamics parameters
        DynamicsParams params;
    };

    struct BoardState
    {
        uint32_t Timestamp = 0;
        uint32_t Status = 0;
        uint32_t DigitalIO = 0;
        uint32_t Temperature = 0;

        AxisState Axes[4];

        // Simulation behavior: require a power-off after startup before granting amp enable
        // When true, motors won't transition to STATUS until a power-off event is seen once.
        bool RequirePowerCycleLatch = true;
        bool HasSeenPowerOff = false;
    };

    class WriteRequest
    {
    public:
        nodeid_t node;
        nodeaddr_t addr;
        quadlet_t data;
        unsigned char flags;

        WriteRequest(nodeid_t n, nodeaddr_t a, quadlet_t d, unsigned char f)
            : node(n), addr(a), data(d), flags(f) {}
    };

    const uint32_t PERIOD_MASK = 0x03FFFFFFu;
    const uint32_t DIR_BIT = 0x40000000u;
    const uint32_t OVF_BIT = 0x80000000u;

    const char kSimQLASN[12] = "QLA 1234-56";
    const std::string SimFPGASerialString = "FPGA 1234-56";

    // i want a queue to store request for every board id or node id separately
    std::map<nodeid_t, std::queue<WriteRequest>> WriteRequestQueues;
    std::map<nodeid_t, BoardState> mBoardStates;
    
    // Simulated FPGA PROM state
    uint32_t SimPromCurrentAddr;

    // Dynamics simulation thread state
    std::thread dynamicsThread;
    std::mutex stateMutex;
    std::atomic<bool> dynamicsRun{false};

    DynamicsParams joint_params{};

    // Helper to fetch a simulated PROM byte at absolute 24-bit address
    uint8_t GetSimPromByte(uint32_t abs_addr) const;
    quadlet_t ProcessPROM(nodeid_t node);

    // Advance simple closed-loop joint dynamics for all axes
    void UpdateAxisDynamics(nodeid_t node, BoardState &state, double dt);
    void DynamicsThreadFunc();
};