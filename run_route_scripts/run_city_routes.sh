#!/bin/bash
files=$(ls -b requests/city_to_city)
rm results/OUTDIRS.txt

for f in ${files[@]}
do
  PATH=../:${PATH} ./batch.sh city_to_city/$f
  cat outdir.txt >> results/OUTDIRS.txt
  rm outdir.txt
done

exit
