#!/bin/bash
set -e
shopt -s expand_aliases

if [ -z $4 ]; then
  echo "You can visually diff two runs of RAD with differing request parameters in the following way.
        First create two RAD request files with the appropriate request parameter changes. Run RAD:
        ./run_with_server.py --test-file a.txt --url http://localhost:8002/route --concurrency 24 --format csv
        ./run_with_server.py --test-file b.txt --url http://localhost:8002/route --concurrency 24 --format csv
        This will create two directories, 20210125_103436_a 20210125_103550_b respectively.
        To visually inspect the differences run this script as follows:
        ${0} 20210125_103436_a 20210125_103550_b a.txt b.txt"
  exit 1;
fi;

#args are
output_dir1=${1}
output_dir2=${2}
input_file1=${3}
input_file2=${4}

# this will urlencode strings from pipe
alias urlencode='python3 -c "import sys; import os; import requests; print(*(requests.utils.quote(line.strip()) for line in sys.stdin), sep=os.linesep)"'

# diff to get the file names, these have the line number in the original input
for i in $(diff -qr ${output_dir1} ${output_dir2} | sed -e "s/.*\///g" -e "s/\..*$//g" | sort -n | grep -v "statistics"); do
  anchor_before=$(echo -e "[$(sed "${i}q;d" ${input_file1} | sed -e "s/^[^{]*//g" -e "s/}[^}]*$/}/g" | jq -rc '. + {id: "770000"}')," | urlencode)
  anchor_after=$(echo -e "$(sed "${i}q;d" ${input_file2} | sed -e "s/^[^{]*//g" -e "s/}[^}]*$/}/g" | jq -rc '. + {id: "007700"}')]" | urlencode)
  echo "http://valhalla.github.io/demos/routing/simple.html#${anchor_before}${anchor_after}";
done
