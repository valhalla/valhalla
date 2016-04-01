#!/bin/bash

for lc in ${@}; do
  LC_ALL=C sudo locale-gen ${lc}
  if [ ${?} -ne 0 ]; then
    echo "Failed to install ${lc}"
    exit 1
  fi
  #the above can fail with but still returns 0:
  #  Generating locales...
  #  ko_KR.UTF-8... hash collision (1701936715) ko_KR.utf8, es_CO.utf8
  #  failed
  #  Generation complete.
  if [ "$(locale -a | grep -cF ${lc})" == "0" ]; then
    echo "Failed to install ${lc}.. retrying.."
    LC_ALL=C sudo locale-gen ${lc}
  fi
  #second time was not the charm
  if [ "$(locale -a | grep -cF ${lc})" == "0" ]; then
    echo "Retry failed"
    exit 1
  fi
done
