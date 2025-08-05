#!/bin/bash
directories=$(cat OUTDIRS.txt)
for d in ${directories[@]}
do
  if [ -d $d ]
  then
    cd $d
    echo $d
    fails=$(cat statistics.csv | grep fail | wc -l)
    count=$(cat statistics.csv | wc -l)
    let count=count-1
    perc=$(python3 -c "print(($fails/($count * 1.0))*100)")
    echo -n "  Percent of failed routes = "
    echo $perc
    cd ..
  fi
done
exit
