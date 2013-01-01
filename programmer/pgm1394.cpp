/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <iostream>
#include "mcsFile.h"

int main()
{
    mcsFile promFile;
    if (promFile.OpenFile("FPGA1394qla.mcs")) {
        while (promFile.ReadNextSector()) {
            std::cout << "Read sector, address = " << std::hex << promFile.GetSectorAddress() << std::endl;
        }
        promFile.CloseFile();
    }
}
