for i in {24..26}
do
	printf "Problem %d.\n" $i
	setsid ./bin/main $i $1 &
done
wait
printf "Testing done.\n"