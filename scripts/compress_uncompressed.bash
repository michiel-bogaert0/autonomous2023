#!/bin/bash

for file in $@
do
	echo $file
	rosbag info -yk compression $file | grep lz4
	retVal=$?
	if [ $retVal -ne 0 ]; then
		echo Compressing $file, was not compressed...
		rosbag compress --lz4 $file
		filename=${file%.bag}
        	original_file=$(echo $filename.orig.bag)
        	rm -i $original_file

	fi
done
