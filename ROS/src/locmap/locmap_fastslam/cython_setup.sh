#!/bin/sh
cd $(echo $PWD | sed 's/build/src\/locmap/g')
python3 ./setup.py build_ext --inplace  