#! /usr/bin/env bash
../../Tools/autotest/sim_vehicle.py -v ArduCopter --model JSON --enable-dds -D -G --add-param-file=$PWD/config/suction.parm -A --synthetic-clock -A --serial5=sim:sf45b