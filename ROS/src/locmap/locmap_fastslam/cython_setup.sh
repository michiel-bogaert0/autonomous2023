#!/bin/sh
echo $PWD
cd ./../../../src/locmap/locmap_fastslam
python3 ./setup.py build_ext --inplace  