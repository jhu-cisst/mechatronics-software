/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Jie Ying Wu

  (C) Copyright 2011-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "BoardIO.h"
#include "BasePort.h"

/*******************************************************************************
 * Get commands
 */

uint32_t BoardIO::GetFirmwareVersion(void) const
{
    return (port ? port->GetFirmwareVersion(BoardId) : 0);
}

/*******************************************************************************
 * Read commands
 */

uint32_t BoardIO::ReadStatus(void) const
{
    uint32_t read_data = 0;
    if (port) port->ReadQuadlet(BoardId, BoardIO::BOARD_STATUS, read_data);
    return read_data;
}

uint32_t BoardIO::ReadIPv4Address(void) const
{
    if (GetFirmwareVersion() < 7) {
        std::cerr << "AmpIO::ReadIPv4Address: requires firmware 7 or above" << std::endl;
        return 0;
    }
    uint32_t read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, BoardIO::IP_ADDR, read_data);
    return read_data;
}

/*******************************************************************************
 * Write commands
 */

bool BoardIO::WriteIPv4Address(uint32_t IPaddr)
{
    if (GetFirmwareVersion() < 7) {
        std::cerr << "BoardIO::WriteIPv4Address: requires firmware 7 or above" << std::endl;
        return false;
    }
    return (port ? port->WriteQuadlet(BoardId, BoardIO::IP_ADDR, IPaddr) : false);
}
