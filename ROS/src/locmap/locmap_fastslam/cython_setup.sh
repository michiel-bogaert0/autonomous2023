#!/bin/sh
cd $(echo $PWD | sed 's/build/src/g')
python3 ./setup.py build_ext --inplace  
test=$PWD