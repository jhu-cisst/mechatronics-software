#!/bin/bash
#
# Script to run pgm1394 on multiple boards.  This script doesn't reboot the boards.
# Usage: pgm1394multi 0 1 6 7 # parameters are board IDs
#

boards="$@"

for boardId in $boards
do
    echo "Programming board $boardId"
    pgm1394 $boardId -a
    result=$?
    echo "Result: $result (0 is good)"
    if [[ "$result" -ne 0 ]]; then
       echo "------> pgm1394 -a failed for board $boardId"
       echo "------> DO NOT REBOOT OR POWER OFF this board"
       echo "------> Try to reprogram this board using pgm1394"
       exit $result
    fi
done

echo "------> You now need to reboot your controllers.  You can either"
echo "------> power cycle them or use: qlacommand -c reboot"
echo ""
echo "------> If your PC is connected to the controllers via FireWire"
echo "------> you might also need to unplug/replug the FireWire cable to"
echo "------> rediscover the boards (or use sudo qlareloadfw.bash)"

exit 0
