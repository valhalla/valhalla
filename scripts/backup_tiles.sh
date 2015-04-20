#!/bin/bash

TIME=`date +%Y_%m_%d_%H%M%S`
mv $1 $1.$TIME
mkdir -p $1
