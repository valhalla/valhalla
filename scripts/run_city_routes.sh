#!/bin/bash
files=$(ls -b city_to_city)
rm OUTDIRS.txt

for f in ${files[@]}
do
  PATH=../:${PATH} ./batch.sh city_to_city/$f
  cat outdir.txt >> OUTDIRS.txt
  rm outdir.txt
done

exit
