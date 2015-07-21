#!/bin/bash
directories=$(cat OUTDIRS.txt)
for d in ${directories[@]}
do
  if [ -d $d ]
  then
    cd $d
    echo $d
    fails=$(cat statistics.csv | grep false | wc -l)
    count=$(cat statistics.csv | wc -l)
    perc=$(python -c "print ($fails/($count * 1.0))*100")
    echo -n "  Percent of failed routes = "
    echo $perc
    cd ..
  fi
done
exit
