for i in {1..32}
do
	printf "Problem %d.\n" $i
	setsid ./bin/main $i $1 &
done
wait
printf "Testing done.\n"