#include <iostream>
#include "Eth1394Port.h"
#include "AmpIO.h"

int main(int argc, char *argv[])
{
    Eth1394Port Port(0, std::cerr);

//    std::cout << "IsOK() " << Port.IsOK() << std::endl;

    AmpIO board1(0);
    Port.AddBoard(&board1);
//    std::cerr << "AddBoard" << std::endl;

    for (int i = 0; i < 1; i++) {
        Port.ReadAllBoards();
        board1.SetMotorCurrent(0, 0x8000);
        board1.SetMotorCurrent(1, 0x8111);
        board1.SetMotorCurrent(2, 0x8222);
        board1.SetMotorCurrent(3, 0x8333);
        Port.WriteAllBoards();
        std::cerr << "cycle = " << i << std::endl;
    }
//    board1.WriteSafetyRelay(true);

    Port.RemoveBoard(0);
    return 0;
}
