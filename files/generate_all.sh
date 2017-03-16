#!/usr/bin/env bash
for i in {100..102}
do
	echo 'Generating images for problem '$i'.'
	./generate_seq.sh $i $1
done