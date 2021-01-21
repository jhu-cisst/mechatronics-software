#!/bin/bash
for boardId in "$@"
do
    echo "Programming board $boardId"
    pgm1394 $boardId -a 
done
