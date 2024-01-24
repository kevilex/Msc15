#!/bin/bash
echo "1 for sim, 2 for real"
read num
if [ $num -lt 2 ]; then
	terminator -l sim -p default
else
	terminator -l control -p default
fi
