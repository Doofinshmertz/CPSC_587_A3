#!/bin/sh
rm -r build
cmake -Bbuild
cd build
make
./cpsc587_a3_hh

