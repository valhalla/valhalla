#!/bin/bash

TIME=`date +%Y_%m_%d_%H%M%S`
rsync -avPc $1 $1.$TIME
