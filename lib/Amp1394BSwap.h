/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2020-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __AMP1394BSWAP_H__
#define __AMP1394BSWAP_H__

#include "Amp1394Types.h"

#ifdef _MSC_VER
#include <stdlib.h>   // for byteswap functions
inline uint16_t bswap_16(uint16_t data) { return _byteswap_ushort(data); }
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }

#elif defined(__APPLE__)
#include <libkern/OSByteOrder.h>
#define bswap_32(x) OSSwapInt32(x)
#define bswap_16(x) OSSwapInt16(x)

#else
#include <byteswap.h>
#endif

#endif
