/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015

  (C) Copyright 2015-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>

int main(int, char**)
{
    std::cerr << "qlacloserelays is deprecated, use `qlacommand` instead.  Examples:" << std::endl
              << "> qlacommand -c close-relays" << std::endl
              << "> qlacommand -pfw:0 -c close-relays" << std::endl
              << "> qlacommand -c open-relays" << std::endl;
    return 0;
}
