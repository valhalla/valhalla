#!/bin/bash
number_to_save=$2
tile_dirs=`ls -d $1.* | sort -r`
let count=0
for tile_dir in ${tile_dirs} 
do
   if [ $count -ge $number_to_save ]; then
      rm -rf $tile_dir
   fi
   let count=count+1
done

