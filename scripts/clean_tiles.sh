#!/bin/bash

tile_dirs=`ls -d $1.* | sort -r`
let count=0
for tile_dir in ${tile_dirs} 
do
   if [ $count -ge 2 ]; then
      rm -rf $tile_dir
   fi
   let count=count+1
done

