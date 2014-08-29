#include <iostream>
#include "Eth1394Port.h"
#include "AmpIO.h"

int main()
{
    Eth1394Port Port(0, std::cerr);

    AmpIO board1(1);
    AmpIO board2(13);
    AmpIO board3(0);
//    AmpIO board4(4);
    Port.AddBoard(&board1);
    Port.AddBoard(&board2);
    Port.AddBoard(&board3);
//    Port.AddBoard(&board4);
    Port.ReadAllBoardsBroadcast();

//    for (int i = 0; i < 1; i++) {
//        Port.ReadAllBoards();
//        board1.SetMotorCurrent(0, 0x8000);
//        board1.SetMotorCurrent(1, 0x8111);
//        board1.SetMotorCurrent(2, 0x8222);
//        board1.SetMotorCurrent(3, 0x8333);
//        Port.WriteAllBoards();
//        std::cerr << "cycle = " << i << std::endl;
//    }


    Port.RemoveBoard(1);
    Port.RemoveBoard(13);
    Port.RemoveBoard(0);
    //    Port.RemoveBoard(4);
    return 0;
}
