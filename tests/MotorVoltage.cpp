/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include "BasePort.h"
#include "AmpIO.h"
#include "Amp1394Time.h"

const double bits2volts = (2.5/65535)*21.0;

double MeasureMotorSupplyVoltage(BasePort *curPort, AmpIO *curBoard, unsigned int index)
{
    // Firmware version must be >= 8
    if (curBoard->GetFirmwareVersion() < 8)
        return 0.0;

    // QLA IO expander must be present (QLA 1.5+)
    if (!curBoard->IsQLAExpanded(index))
        return 0.0;

    // Make sure board power is on and amplifier power is off
    curBoard->WritePowerEnable(true);
    curBoard->WriteAmpEnable(0xff, 0);

    double V = 0.0;    // Measured voltage (in volts)
    uint32_t volts;    // Measured voltage (in bits)
    uint32_t vinc = 0x1000;
    nodeaddr_t dac_addr = (index == 2) ? 0x81 : 0x41;
    for (volts = 0; volts <= 0x0000ffff; volts += vinc) {
        // Write voltage to DAC 4 or 8 (for DQLA)
        curPort->WriteQuadlet(curBoard->GetBoardId(), dac_addr, volts | 0x80000000);
        Amp1394_Sleep(0.001);
        bool dac_lt_supply = curBoard->ReadMotorSupplyVoltageBit(index);
        if (!dac_lt_supply) {
            // if DAC voltage greater than supply,
            // backup and reduce resolution
            if (vinc <= 1) break;
            volts -= vinc;
            vinc >>= 1;
        }
    }
    curPort->WriteQuadlet(curBoard->GetBoardId(), dac_addr, 0x80008000);
    curBoard->WritePowerEnable(false);
    if (volts <= 0x0000ffff)
        V = volts*bits2volts;
    return V;
}
