#!/bin/bash
set -e

#output the code
echo '#include <string_view>'
echo '#include "midgard/const_map.h"'

for f in "${@}"; do
	var="$(echo ${f} | sed -e "s/[-.]/_/g")"
	echo "constexpr std::string_view ${var} = R\"L0CAL3("
	cat ${f}
	echo ")L0CAL3\";"
	echo
done

# Build the constexpr map
echo "constexpr std::pair<std::string_view, std::string_view> locales_json_data[] = {"
for f in "${@}"; do
	key="${f%.*}"
	var="$(echo ${f} | sed -e "s/[-.]/_/g")"
	echo "  {\"${key}\", ${var}},"
done
echo "};"
echo "inline constexpr auto locales_json = valhalla::midgard::ConstFlatMap(locales_json_data);"

#install locales locally for testing
if command -v jq &> /dev/null && command -v localedef &> /dev/null; then
	for loc in $(jq ".posix_locale" *.json | sed -e 's/"//g'); do
		localedef -i "${loc%.*}" -f "${loc##*.}" "./${loc}" 2>/dev/null || true
	done
fi
