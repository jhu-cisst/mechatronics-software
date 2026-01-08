// system
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <string.h>
#include <byteswap.h>
#include <sys/select.h>
#include <errno.h> // errno

// firewire
#include "SimulationPort.h"
#include "Amp1394Time.h"

// this is needed for backtrace
#include <execinfo.h>
#include <dlfcn.h>

void printBacktrace()
{
    void *callstack[128];
    int frames = backtrace(callstack, 128);
    char **symbols = backtrace_symbols(callstack, frames);

    std::cout << "Backtrace:" << std::endl;
    for (int i = 0; i < frames; ++i)
    {
        std::cout << "  [" << i << "] " << symbols[i] << std::endl;
    }
    free(symbols);
}

SimulationPort::SimulationPort(int portNum, std::ostream &ostr) : BasePort(portNum, ostr)
{
    // std::cout << "[Simulation Port] Constructor called" << std::endl;

    // Prepare PROM sector content ending with
    // 0xFF terminator as in real PROM
    SimPromCurrentAddr = 0;
    Init();
}

SimulationPort::~SimulationPort()
{
    // std::cout << "[Simulation Port] Destructor called" << std::endl;
    Cleanup();
}

bool SimulationPort::Init()
{
    // std::cout << "[Simulation Port] Init called" << std::endl;

    // Invoke scan nodes to initialize simulated nodes
    bool ret = ScanNodes();

    // Initialize neutral DAC (motor current) for all simulated boards/axes
    // to avoid unintended torque at startup before any valid command is sent.
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        for (nodeid_t n = 0; n < BoardIO::MAX_BOARDS; ++n)
        {
            BoardState &st = mBoardStates[n];
            for (int i = 0; i < 4; ++i)
            {
                st.Axes[i].MotorCurrent = 32768u; // midrange -> zero current
            }
        }
    }

    // Start dynamics background thread
    dynamicsRun = true;
    dynamicsThread = std::thread(&SimulationPort::DynamicsThreadFunc, this);

    return ret;
}

void SimulationPort::Cleanup()
{
    // std::cout << "[Simulation Port] Cleanup called" << std::endl;

    // Nothing to clean up for simulation
    if (dynamicsRun.load())
    {
        dynamicsRun = false;
        if (dynamicsThread.joinable())
            dynamicsThread.join();
    }
}

nodeid_t SimulationPort::InitNodes()
{
    // std::cout << "[Simulation Port] InitNodes called" << std::endl;

    // let's simulate the maximum number of boards since we don't
    // know exactly how many boards will be added later
    return BoardIO::MAX_BOARDS;
}

int SimulationPort::NumberOfUsers()
{
    // std::cout << "[Simulation Port] NumberOfUsers called" << std::endl;
    return 1;
}

unsigned int SimulationPort::GetBusGeneration() const
{
    // std::cout << "[Simulation Port] GetBusGeneration called" << std::endl;
    return 0;
}

bool SimulationPort::AddBoard(BoardIO *board)
{
    // std::cout << "[Simulation Port] AddBoard called" << std::endl;
    bool ret = BasePort::AddBoard(board);
    return ret;
}

bool SimulationPort::RemoveBoard(unsigned char boardId)
{
    // std::cout << "[Simulation Port] RemoveBoard called" << std::endl;
    return BasePort::RemoveBoard(boardId);
}

bool SimulationPort::WriteBroadcastOutput(quadlet_t *buffer, unsigned int size)
{
    throw std::runtime_error("Broadcast output not implemented with simulation port");
    return true;
}

bool SimulationPort::WriteBroadcastReadRequest(unsigned int seq)
{
    throw std::runtime_error("Broadcast read request not implemented with simulation port");
    return true;
}

void SimulationPort::WaitBroadcastRead()
{
    throw std::runtime_error("Broadcast read wait not implemented with simulation port");
}

bool SimulationPort::ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata, unsigned int nbytes, unsigned char)
{
    // std::cout << "[Simulation Port] ReadBlockNode called for node " << node << ", addr "
    //           << std::hex << addr << std::dec << ", nbytes " << nbytes << std::endl;

    // Simulate real-time block read (addr 0x0000)
    if (addr == 0x0000)
    {
        // Size should be 128 bytes (32 quadlets) for QLA1 with Firmware Rev 8/9
        unsigned int numQuads = nbytes / 4;
        memset(rdata, 0, nbytes);

        // Protect shared state while reading
        std::lock_guard<std::mutex> lock(stateMutex);
        BoardState &state = mBoardStates[node];
        state.Timestamp++;

        // Status: MV_GOOD | PWR_ENABLE | RELAY_ON | 4 AXES | BOARD_ID
        // 0x00080000 | 0x00040000 | 0x00030000 = 0x000F0000 (RELAY_ON = RELAY_FB | RELAY_BIT)
        // 4 Axes = 0x40000000
        // Board ID = node << 24
        if (state.Status == 0)
        {
            // Fallback if status not initialized (should be done in AddBoard)
            state.Status = 0x40080000 | (node << 24);
        }

        if (numQuads > 0)
            rdata[0] = bswap_32(state.Timestamp);
        if (numQuads > 1)
            rdata[1] = bswap_32(state.Status);
        if (numQuads > 2)
            rdata[2] = bswap_32(state.DigitalIO);
        if (numQuads > 3)
            rdata[3] = bswap_32(state.Temperature);

        // 4-7: Motor Current
        for (int i = 0; i < 4; ++i)
        {
            if (numQuads > 4 + i)
            {
                // Pack analog potentiometer (upper 16 bits) + motor current ADC (lower 16 bits)
                // Map encoder signed counts to a 16-bit potentiometer ADC centered at midrange
                const uint32_t ENC_MIDRANGE = 0x00800000u;

                // Use physical position from SimPosition to avoid issues with EncoderOffset wrapping
                double counts_from_zero = (state.Axes[i].SimPosition / (2.0 * M_PI)) * state.Axes[i].params.counts_per_turn;
                int32_t physicalPos = static_cast<int32_t>(counts_from_zero + 0x800000);

                int32_t signedCounts = static_cast<int32_t>(physicalPos & 0x00FFFFFFu) - static_cast<int32_t>(ENC_MIDRANGE);
                int32_t potADC = static_cast<int32_t>(32768) + (signedCounts >> 8); // coarse scale: counts/256 -> ADC

                if (potADC < 0)
                {
                    potADC = 0;
                }

                if (potADC > 65535)
                {
                    potADC = 65535;
                }

                uint32_t packed = (static_cast<uint32_t>(potADC) << 16) | (state.Axes[i].MotorCurrent & 0x0000FFFFu);
                rdata[4 + i] = bswap_32(packed);
            }
        }
        // 8-11: Encoder Position
        for (int i = 0; i < 4; ++i)
        {
            if (numQuads > 8 + i)
            {
                rdata[8 + i] = bswap_32(state.Axes[i].EncoderPos);
            }
        }
        // 12-15: Encoder Velocity
        for (int i = 0; i < 4; ++i)
        {
            if (numQuads > 12 + i)
            {
                // Convert counts/sec to period in clock ticks (49.152 MHz)
                // Period for 4 counts (quadrature cycle)
                int32_t vel = state.Axes[i].EncoderVel;
                uint32_t regVal = 0;
                if (vel == 0)
                {
                    regVal = OVF_BIT; // Zero velocity -> overflow
                }
                else
                {
                    double absVel = std::abs((double)vel);
                    constexpr double CLK_HZ = 49152000.0;
                    double period = (4.0 * CLK_HZ) / absVel;
                    uint32_t ticks = static_cast<uint32_t>(period);
                    if (ticks > PERIOD_MASK)
                    {
                        regVal = OVF_BIT | PERIOD_MASK; // Saturate and flag overflow
                    }
                    else
                    {
                        regVal = (ticks & PERIOD_MASK);
                    }

                    if (vel >= 0)
                    {
                        regVal |= DIR_BIT; // Direction bit
                    }
                }
                rdata[12 + i] = bswap_32(regVal);
            }
        }
        // 16-19: QTR1
        for (int i = 0; i < 4; ++i)
        {
            if (numQuads > 16 + i)
            {
                // QTR1 is quarter cycle period (1 count)
                int32_t vel = state.Axes[i].EncoderVel;
                uint32_t regVal = 0;
                if (vel == 0)
                {
                    regVal = OVF_BIT; // Overflow
                }
                else
                {
                    double absVel = std::abs((double)vel);
                    constexpr double CLK_HZ = 49152000.0;
                    double period = (1.0 * CLK_HZ) / absVel;
                    uint32_t ticks = static_cast<uint32_t>(period);
                    if (ticks > PERIOD_MASK)
                    {
                        regVal = OVF_BIT | PERIOD_MASK; // Saturate and flag overflow
                    }
                    else
                    {
                        regVal = (ticks & PERIOD_MASK);
                    }

                    if (vel >= 0)
                    {
                        regVal |= DIR_BIT; // Direction bit
                    }
                }
                rdata[16 + i] = bswap_32(regVal);
            }
        }
        // 20-23: QTR5
        for (int i = 0; i < 4; ++i)
        {
            if (numQuads > 20 + i)
            {
                // QTR5 is previous quarter cycle period (same as QTR1 for constant vel)
                int32_t vel = state.Axes[i].EncoderVel;
                uint32_t regVal = 0;
                if (vel == 0)
                {
                    regVal = OVF_BIT; // Overflow
                }
                else
                {
                    double absVel = std::abs((double)vel);
                    constexpr double CLK_HZ = 49152000.0;
                    double period = (1.0 * CLK_HZ) / absVel;
                    uint32_t ticks = static_cast<uint32_t>(period);
                    if (ticks > PERIOD_MASK)
                    {
                        regVal = OVF_BIT | PERIOD_MASK; // Saturate and flag overflow
                    }
                    else
                    {
                        regVal = (ticks & PERIOD_MASK);
                    }

                    if (vel >= 0)
                    {
                        regVal |= DIR_BIT; // Direction bit
                    }
                }
                rdata[20 + i] = bswap_32(regVal);
            }
        }
        // 24-27: Running Counter
        for (int i = 0; i < 4; ++i)
        {
            if (numQuads > 24 + i)
            {
                // Convert microseconds to clock ticks (49.152 MHz)
                double runUs = (double)state.Axes[i].EncoderRun;
                double runTicks = runUs * 49.152;
                uint32_t regVal = static_cast<uint32_t>(runTicks);
                if (regVal > 0x03FFFFFF)
                    regVal = 0x03FFFFFF; // Cap at max 26 bits
                rdata[24 + i] = bswap_32(regVal);
            }
        }
        // 28-31: Motor Status
        for (int i = 0; i < 4; ++i)
        {
            if (numQuads > 28 + i)
            {
                rdata[28 + i] = bswap_32(state.Axes[i].MotorStatus);
            }
        }

        return true;
    }

    // Simulate FPGA buffer read used for PROM readback
    // FpgaIO::PromReadData reads from 0x2000 (rev >=4) or 0x00c0 (older), we'll support 0x2000
    if (addr == 0x2000)
    {
        // Copy nbytes from simulated PROM starting at SimPromCurrentAddr
        // rdata is a quadlet_t* but nbytes is byte count; use memcpy safely
        std::vector<uint8_t> tmp(nbytes, 0);
        for (unsigned int i = 0; i < nbytes; ++i)
        {
            tmp[i] = GetSimPromByte(0x00000000u + (SimPromCurrentAddr + i));
        }
        memcpy(reinterpret_cast<void *>(rdata), tmp.data(), nbytes);
        return true;
    }

    return true;
}

bool SimulationPort::WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes, unsigned char)
{
    // std::cout << "[Simulation Port] WriteBlockNode called for node " << node << ", addr " << std::hex << addr << std::dec << ", nbytes " << nbytes << std::endl;

    if (addr == 0x0000)
    {
        // Constants from AmpIO.cpp
        const uint32_t VALID_BIT = 0x80000000;
        const uint32_t MOTOR_ENABLE_MASK = 0x20000000;
        const uint32_t MOTOR_ENABLE_BIT = 0x10000000;
        const uint32_t MSTAT_AMP_STATUS = 0x20000000;
        const uint32_t MSTAT_AMP_REQ = 0x10000000;
        const uint32_t DAC_MASK = 0x0000ffff;
        const uint32_t PWR_ENABLE_MASK = 0x00080000;
        const uint32_t PWR_ENABLE_BIT = 0x00040000;
        const uint32_t RELAY_MASK = 0x00020000;
        const uint32_t RELAY_BIT = 0x00010000;
        const uint32_t RELAY_FB = 0x00020000;

        // Firmware Rev 8/9 write block format:
        // [0] Header (BoardId << 8 | Length)
        // [1..4] Motor DAC/Control
        // [5] Control/Status

        unsigned int numQuads = nbytes / 4;
        // We expect at least 4 motors + 2 quadlets = 6 quadlets
        if (numQuads < 6)
        {
            std::cerr << "[Simulation Port] WriteBlockNode: buffer too small for addr 0x0000" << std::endl;
            return false;
        }

        // Take the board id from the header for verification
        quadlet_t header = bswap_32(wdata[0]);
        unsigned char boardId = (header >> 8) & 0x0F;
        if (boardId != static_cast<unsigned char>(node))
        {
            std::cerr << "[Simulation Port] WriteBlockNode: boardId/header mismatch: "
                      << static_cast<int>(boardId) << " vs " << static_cast<int>(node) << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lock(stateMutex);
        BoardState &state = mBoardStates[node];

        // Process Motor Commands (Motors 0-3)
        for (int i = 0; i < 4; ++i)
        {
            // wdata is big-endian
            quadlet_t cmd = bswap_32(wdata[1 + i]);

            // Update Motor Current / DAC
            if (cmd & VALID_BIT)
            {
                state.Axes[i].MotorCurrent = (cmd & DAC_MASK);
            }

            // Update Motor Enable
            if (cmd & MOTOR_ENABLE_MASK)
            {
                if (cmd & MOTOR_ENABLE_BIT)
                {
                    // Request Enable
                    state.Axes[i].MotorStatus |= MSTAT_AMP_REQ;
                    // Reflect request in QLA status lower nibble
                    state.Status |= (1u << i);
                }
                else
                {
                    // Request Disable
                    state.Axes[i].MotorStatus &= ~MSTAT_AMP_REQ;
                    state.Axes[i].MotorStatus &= ~MSTAT_AMP_STATUS;
                    // Reflect request clear in QLA status lower nibble
                    state.Status &= ~(1u << i);
                }
            }
        }

        // Process Control Word (last quadlet for 4 motors is at index 5)
        quadlet_t ctrl = bswap_32(wdata[5]);

        // Handle Power Enable
        if (ctrl & PWR_ENABLE_MASK)
        {
            if (ctrl & PWR_ENABLE_BIT)
            {
                state.Status |= PWR_ENABLE_BIT;
            }
            else
            {
                state.Status &= ~PWR_ENABLE_BIT;
            }
        }

        // Handle Safety Relay
        if (ctrl & RELAY_MASK)
        {
            if (ctrl & RELAY_BIT)
            {
                state.Status |= RELAY_BIT;
                state.Status |= RELAY_FB;
            }
            else
            {
                state.Status &= ~RELAY_BIT;
                state.Status &= ~RELAY_FB;
            }
        }

        // Update Amp Status based on Request + Power + Relay
        bool powerOn = (state.Status & PWR_ENABLE_BIT);
        bool relayOn = (state.Status & RELAY_BIT);

        for (int i = 0; i < 4; ++i)
        {
            if ((state.Axes[i].MotorStatus & MSTAT_AMP_REQ) && powerOn && relayOn)
            {
                state.Axes[i].MotorStatus |= MSTAT_AMP_STATUS;
            }
            else
            {
                state.Axes[i].MotorStatus &= ~MSTAT_AMP_STATUS;
            }
        }

        return true;
    }

    std::cout << "[Simulation Port] WriteBlockNode called for node " << node << ", addr " << std::hex << addr << std::dec << ", nbytes " << nbytes << std::endl;
    printBacktrace();
    throw std::runtime_error("WriteBlockNode not implemented in simulation port");
    return true;
}

bool SimulationPort::ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags)
{
    // std::cout << "[Simulation Port] ReadQuadletNode called for node " << node << ", addr " << std::hex << addr << std::dec << std::endl;

    // Check for per-axis registers (channels 1-4)
    unsigned int channel = (addr >> 4);
    unsigned int reg = (addr & 0x0F);

    if (channel >= 1 && channel <= 4)
    {
        int axisIndex = channel - 1;
        if (reg == 4)
        {
            // ENC_LOAD_REG
            data = static_cast<quadlet_t>(mBoardStates[node].Axes[axisIndex].EncoderPreload);
            return true;
        }
    }

    switch (addr)
    {
    case BoardIO::BOARD_STATUS: // 0
    {
        // let's use node id as board id but we have to pack the data
        // so that bits 27-24 contain the board id since BOARD_ID_MASK = 0x0F000000
        BoardState &state = mBoardStates[node];
        if (state.Status == 0)
        {
            state.Status = 0x40080000 | (node << 24);
        }
        data = state.Status;
        break;
    }
    case BoardIO::HARDWARE_VERSION: // 4
        // Simulate hardware version for QLA1
        data = QLA1_String;
        break;
    case BoardIO::FIRMWARE_VERSION: // 7
        // Simulate firmware version 9 (Rev9). Sequential protocol remains compatible.
        data = 9;
        break;
    case BoardIO::ETH_STATUS: // 12 (hex 0x0C)
        // virtual status of the FPGA so that it can be properly parsed by the
        // board IO class to determine FPGA version etc.
        data = 0x40000000;
        break;
    case BoardIO::GIT_DESC: // 15 (hex 0x0F)
        // simulation of the dirty bit and commit count
        data = 0;
        break;
    case 0x0008:
        // FPGA status register used by PROM operations
        // In our simulation, PROM command finishes immediately, lower 4 bits 0
        data = 0x00000000;
        break;
    case 0x0009:
        // PROM result register for M25P16
        // Return number of quadlets read/written; for read, firmware returns 64 (256 bytes)
        data = 64;
        break;
    case 0x3002:
        // this is the result address for the PROM results for PROM_25AA128
        data = ProcessPROM(node);
        break;
    default:
        printBacktrace();
        throw std::runtime_error("[Simulation Port]Unknown address in ReadQuadletNode");
        break;
    }

    return true;
}

bool SimulationPort::WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags)
{
    // std::cout << "[Simulation Port] WriteQuadletNode called for node " << node << ", addr " << std::hex << addr << std::dec << std::endl;

    // Check for per-axis registers (channels 1-4)
    unsigned int channel = (addr >> 4);
    unsigned int reg = (addr & 0x0F);

    if (channel >= 1 && channel <= 4)
    {
        int axisIndex = channel - 1;
        if (reg == 4)
        {
            // ENC_LOAD_REG
            std::lock_guard<std::mutex> lock(stateMutex);
            int32_t val = static_cast<int32_t>(data);

            // Simulation uses 24-bit encoders. Ensure the loaded value is masked to 24 bits.
            // This prevents a jump in EncoderPos in the next simulation cycle if val was negative
            // (e.g. small negative value becoming large positive value due to mask in UpdateAxisDynamics).
            val = val & 0x00FFFFFF;

            std::cout << "[Simulation Port] Encoder bias for node " << static_cast<int>(node)
                      << ", axis " << axisIndex << " set to " << val << std::endl;

            // Preserve physical position (simulated potentiometer value)
            // PhysicalPos = (EncoderPos - EncoderOffset)
            // NewOffset = NewEncoderPos - PhysicalPos
            //           = val - (EncoderPos - EncoderOffset)
            //           = val - EncoderPos + EncoderOffset
            int32_t currentPos = mBoardStates[node].Axes[axisIndex].EncoderPos;
            int32_t currentOffset = mBoardStates[node].Axes[axisIndex].EncoderOffset;
            mBoardStates[node].Axes[axisIndex].EncoderOffset = val - currentPos + currentOffset;

            mBoardStates[node].Axes[axisIndex].EncoderPreload = val;
            mBoardStates[node].Axes[axisIndex].EncoderPos = val;

            std::cout << "  Updated EncoderOffset to " << mBoardStates[node].Axes[axisIndex].EncoderOffset;
            std::cout << "  Current Pos is " << currentPos << std::endl;
            std::cout << "  Current Offset is " << currentOffset << std::endl;
            
            return true;
        }
        else if (reg == 1)
        {
            // DAC_CTRL_REG: handle per-axis amp enable requests (Firmware Rev 8+)
            std::lock_guard<std::mutex> lock(stateMutex);
            BoardState &state = mBoardStates[node];
            const uint32_t MOTOR_ENABLE_MASK = 0x20000000u;
            const uint32_t MOTOR_ENABLE_BIT = 0x10000000u;
            const uint32_t MSTAT_AMP_STATUS = 0x20000000u;
            const uint32_t MSTAT_AMP_REQ = 0x10000000u;
            const uint32_t PWR_ENABLE_BIT = 0x00040000u;
            const uint32_t RELAY_BIT = 0x00010000u;

            if (data & MOTOR_ENABLE_MASK)
            {
                if (data & MOTOR_ENABLE_BIT)
                {
                    state.Axes[axisIndex].MotorStatus |= MSTAT_AMP_REQ;
                    // Reflect request in QLA status lower nibble
                    state.Status |= (1u << axisIndex);
                }
                else
                {
                    state.Axes[axisIndex].MotorStatus &= ~MSTAT_AMP_REQ;
                    state.Axes[axisIndex].MotorStatus &= ~MSTAT_AMP_STATUS;
                    // Reflect request clear in QLA status lower nibble
                    state.Status &= ~(1u << axisIndex);
                }

                // Update STATUS bit based on current power/relay state
                bool powerOn = (state.Status & PWR_ENABLE_BIT);
                bool relayOn = (state.Status & RELAY_BIT);
                if ((state.Axes[axisIndex].MotorStatus & MSTAT_AMP_REQ) && powerOn && relayOn)
                {
                    state.Axes[axisIndex].MotorStatus |= MSTAT_AMP_STATUS;
                }
                else
                {
                    state.Axes[axisIndex].MotorStatus &= ~MSTAT_AMP_STATUS;
                }
            }
            return true;
        }
    }

    switch (addr)
    {
    case BoardIO::BOARD_STATUS: // 0
    {
        BoardState &state = mBoardStates[node];

        // Initialize status if needed (same logic as in ReadBlockNode)
        if (state.Status == 0)
        {
            state.Status = 0x40080000 | (node << 24);
        }

        // Handle Power Enable
        if (data & 0x00080000)
        { // PWR_ENABLE_MASK
            if (data & 0x00040000)
            { // PWR_ENABLE_BIT
                state.Status |= 0x00040000;
            }
            else
            {
                state.Status &= ~0x00040000;
            }
        }

        // Handle Safety Relay
        if (data & 0x00020000)
        {
            // RELAY_MASK
            if (data & 0x00010000)
            {                               // RELAY_BIT
                state.Status |= 0x00010000; // Set RELAY_BIT
                state.Status |= 0x00020000; // Set RELAY_FB
            }
            else
            {
                state.Status &= ~0x00010000; // Clear RELAY_BIT
                state.Status &= ~0x00020000; // Clear RELAY_FB
            }
        }

        // Update Amp Status based on Request + Power + Relay
        bool powerOn = (state.Status & 0x00040000); // PWR_ENABLE_BIT
        bool relayOn = (state.Status & 0x00010000); // RELAY_BIT

        // Handle Amp Enable (QLA1 specific protocol)
        // Mask is in bits 8-11 (0x0F00)
        // State is in bit 0 (0x0001)
        uint8_t mask = (data >> 8) & 0x0F;
        if (mask != 0)
        {
            bool enable = (data & 0x01);
            for (int i = 0; i < 4; ++i)
            {
                if (mask & (1 << i))
                {
                    if (enable)
                    {
                        // Set MSTAT_AMP_REQ (0x10000000)
                        state.Axes[i].MotorStatus |= 0x10000000;
                        // Reflect request in QLA status lower nibble
                        state.Status |= (1u << i);
                    }
                    else
                    {
                        // Clear MSTAT_AMP_REQ
                        state.Axes[i].MotorStatus &= ~0x10000000;
                        // Reflect request clear in QLA status lower nibble
                        state.Status &= ~(1u << i);
                    }
                }
            }
        }

        // Update MSTAT_AMP_STATUS based on REQ, Power, and Relay
        for (int i = 0; i < 4; ++i)
        {
            if ((state.Axes[i].MotorStatus & 0x10000000) && powerOn && relayOn)
            {
                state.Axes[i].MotorStatus |= 0x20000000; // MSTAT_AMP_STATUS
            }
            else
            {
                state.Axes[i].MotorStatus &= ~0x20000000;
            }
        }

        return true;
    }

    case 0x0003:
    { // Setting the watchdog period
        std::cout << "[Simulation Port] WriteWatchdogPeriod: " << std::hex << data << std::dec << std::endl;
        return true;
    }
    case 0x0008:
    {
        // Handle PROM read command for M25P16 (data top byte contains command)
        uint32_t cmd = (data & 0xFF000000u);

        if (cmd == 0x03000000u)
        {
            // Store current address for subsequent ReadBlock at 0x2000
            SimPromCurrentAddr = (data & 0x00FFFFFFu);
        }
        else
        {
            printBacktrace();
            throw std::runtime_error("[Simulation Port] Unknown PROM command in WriteQuadletNode");
        }
        // Other commands can be ignored for now
        return true;
    }

    case 0x3000:
    { // Enqueue the write request for the PROM commands for PROM_25AA128 to be processed later
        WriteRequestQueues[node].emplace(node, addr, data, flags);
        return true;
    }

    default:
    {
        printBacktrace();
        throw std::runtime_error("[Simulation Port] Unknown address in WriteQuadletNode");
        break;
    }
    }

    return true;
}

quadlet_t SimulationPort::ProcessPROM(nodeid_t node)
{
    quadlet_t data = 0;

    // std::cout << "[Simulation Port] ProcessPROM called" << std::endl;

    // process the oldest write request for this node
    // let's check if there is any write request for this node
    auto &req_queue = WriteRequestQueues[node];
    if (!req_queue.empty())
    {
        WriteRequest req = req_queue.front();
        req_queue.pop();

        std::cout << "[Simulation Port] Processing PROM write request for node " << req.node
                  << ", addr " << std::hex << req.addr << std::dec
                  << ", data " << std::hex << req.data << std::dec << std::endl;

        switch (req.addr)
        {
        case 0x3000:
        {
            // this is the write address for the PROM commands for PROM_25AA128
            // Store the PROM read command (cmd + addr) to process later
            // Extract the address from bits 8-23 (16-bit addr with 2 MSBs ignored)
            uint16_t prom_addr = (req.data >> 8) & 0x3FFF;
            // Simulate PROM contents for QLA serial number reads
            data = static_cast<quadlet_t>(GetSimPromByte(prom_addr));
            break;
        }
        default:
            printBacktrace();
            throw std::runtime_error("[Simulation Port] Unknown address in ProcessPROM");
            break;
        }
    }

    return data;
}

// Return a simulated PROM byte at absolute 24-bit address. Unused locations are 0xFF.
uint8_t SimulationPort::GetSimPromByte(uint32_t abs_addr) const
{
    // Handle 25AA128 PROM (QLA serial number) - low addresses
    if (abs_addr < sizeof(kSimQLASN))
    {
        return static_cast<uint8_t>(kSimQLASN[abs_addr]);
    }

    // Handle FPGA M25P16 PROM serial sector at 0x001FFF00
    const uint32_t fpga_base = 0x001FFF00u;

    if (abs_addr >= fpga_base)
    {
        uint32_t offset = abs_addr - fpga_base;

        // SimFPGASerialString already contains "FPGA " prefix
        if (offset < SimFPGASerialString.size())
        {
            return static_cast<uint8_t>(SimFPGASerialString[offset]);
        }

        // After string, bytes are 0xFF
        return 0xFF;
    }

    // All other locations are 0xFF (erased PROM state)
    return 0xFF;
}

void SimulationPort::UpdateAxisDynamics(nodeid_t node, BoardState &state, double dt)
{
    for (int i = 0; i < 4; ++i)
    {
        // reference to axis state
        auto &ax = state.Axes[i];

        // Convert DAC bits to Amps using XML-compatible scaling
        // XML AmpsToBits: Offset=32768, Scale=5242.88 -> bits = 32768 + amps * 5242.88
        // Invert: amps = (bits - 32768) / 5242.88
        const double DAC_BITS_PER_AMP = 5242.88;
        double current = (static_cast<int>(ax.MotorCurrent) - 32768) / DAC_BITS_PER_AMP;

        // check if the motor is enabled
        const uint32_t MSTAT_AMP_STATUS = 0x20000000;
        if ((ax.MotorStatus & MSTAT_AMP_STATUS) == 0)
        {
            current = 0.0;
        }

        // Physics simulation in SI units (radians, radians/sec, Nm)
        // Use RK4 integration with sub-stepping for robustness
        const unsigned int steps = std::max(1u, ax.params.integration_steps);
        const double h = dt / static_cast<double>(steps);

        auto accel = [&](double pos, double vel)
        {
            double torque = current * ax.params.torque_constant;
            return (torque - ax.params.viscous_damping * vel) / ax.params.motor_inertia;
        };

        for (unsigned int s = 0; s < steps; ++s)
        {
            // RK4 on the state [position, velocity]
            double k1_pos = ax.SimVelocity;
            double k1_vel = accel(ax.SimPosition, ax.SimVelocity);

            double k2_pos = ax.SimVelocity + 0.5 * h * k1_vel;
            double k2_vel = accel(ax.SimPosition + 0.5 * h * k1_pos,
                                  ax.SimVelocity + 0.5 * h * k1_vel);

            double k3_pos = ax.SimVelocity + 0.5 * h * k2_vel;
            double k3_vel = accel(ax.SimPosition + 0.5 * h * k2_pos,
                                  ax.SimVelocity + 0.5 * h * k2_vel);

            double k4_pos = ax.SimVelocity + h * k3_vel;
            double k4_vel = accel(ax.SimPosition + h * k3_pos,
                                  ax.SimVelocity + h * k3_vel);

            ax.SimPosition += (h / 6.0) * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos);
            ax.SimVelocity += (h / 6.0) * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel);
        }

        // Simple temperature model per axis: heating ~ I^2, cooling to ambient
        // this is not used in the dynamics but just for reporting temperature
        // so we don't really need to RK4 this
        double dTdt_axis = ax.params.thermal_heating_coeff * (current * current) -
                           ax.params.thermal_cooling_coeff * (ax.TemperatureC - ax.params.ambient_temp_c);

        ax.TemperatureC += dTdt_axis * dt;

        // Pack temperature into the state register
        // AmpIO expects 2x Celsius in 8 bits.
        // Packing: T2(MSB), T3, T0, T1(LSB)
        // i=0 -> T0 (bits 8-15)
        // i=1 -> T1 (bits 0-7)
        // i=2 -> T2 (bits 24-31)
        // i=3 -> T3 (bits 16-23)
        uint8_t val = static_cast<uint8_t>(std::round(ax.TemperatureC * 2.0));

        if (i == 0)
        {
            state.Temperature |= (static_cast<uint32_t>(val) << 8);
        }
        else if (i == 1)
        {
            state.Temperature |= static_cast<uint32_t>(val);
        }
        else if (i == 2)
        {
            state.Temperature |= (static_cast<uint32_t>(val) << 24);
        }
        else if (i == 3)
        {
            state.Temperature |= (static_cast<uint32_t>(val) << 16);
        }

        // Convert to encoder units for register emulation
        // Position: counts = (rad / 2pi) * counts_per_turn
        // We add the midrange bias (0x800000) to match the initial state
        double counts_from_zero = (ax.SimPosition / (2.0 * M_PI)) * ax.params.counts_per_turn;

        // Velocity: counts/sec
        double vel_counts = (ax.SimVelocity / (2.0 * M_PI)) * ax.params.counts_per_turn;
        ax.EncoderVel = static_cast<int32_t>(vel_counts);

        // Position with bias and offset
        // PhysicalPos = counts_from_zero + 0x800000
        // EncoderPos = PhysicalPos + EncoderOffset
        double physical_pos_counts = counts_from_zero + 0x800000;
        double encoder_pos_counts = physical_pos_counts + ax.EncoderOffset;

        // Mask to 24-bit encoder position (wrap-around like hardware)
        uint32_t pos24 = static_cast<uint32_t>(static_cast<int64_t>(encoder_pos_counts)) & 0x00FFFFFFu;

        int32_t oldPos = ax.EncoderPos;
        ax.EncoderPos = static_cast<int32_t>(pos24);

        // Very simple quarter tracking: toggle quarters by position changes
        ax.EncoderQtr1 = (ax.EncoderPos & 0x1) ? 1 : 0;
        ax.EncoderQtr5 = (ax.EncoderPos & 0x1) ? 0 : 1;

        // Running counter: time since last edge (increment by dt in microseconds)
        if (ax.EncoderPos != oldPos)
        {
            ax.EncoderRun = 0;
        }
        else
        {
            ax.EncoderRun += static_cast<int32_t>(dt * 1e6);
        }
    }
}

void SimulationPort::DynamicsThreadFunc()
{
    // Simple fixed-rate loop
    while (dynamicsRun.load())
    {
        auto start = std::chrono::steady_clock::now();
        {
            // Update all board states
            std::lock_guard<std::mutex> lock(stateMutex);
            for (auto &kv : mBoardStates)
            {
                UpdateAxisDynamics(kv.first, kv.second, joint_params.dynamics_dt_sec);
            }
        }
        auto end = std::chrono::steady_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        auto target_us = static_cast<long>(joint_params.dynamics_dt_sec * 1e6);
        long sleep_us = std::max(0L, target_us - elapsed);

        // Sleep to maintain period (simple approach)
        // if the remaining time is negative, we are running behind
        // let's notify about it
        if (sleep_us <= 0)
        {
            std::cerr << "[Simulation Port] Dynamics thread is running behind!" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
}