#!/bin/bash
directories=$(ls)
for d in ${directories[@]}
do
  if [ -e $d/statistics.csv ]
  then
    cd $d
    fails=$(cat statistics.csv | grep false | wc -l)
    count=$(cat statistics.csv | wc -l)
    perc=$(python -c "print ($fails/($count * 1.0))*100")
    echo -n "Percent of failed routes = "
    echo $perc
  fi
done
exit
