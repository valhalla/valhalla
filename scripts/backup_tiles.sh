#!/bin/bash

date_time=`date +%Y_%m_%d_%H%M%S`
mv $1 $1.$date_time
mkdir -p $1
