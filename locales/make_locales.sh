#!/bin/bash
set -e

#throw all the text into one big header
code=
for f in "${@}"; do
	code="$(xxd -i ${f})
${code}"
done

#output the code
echo "#include <unordered_map>"
echo "${code}"
echo "const std::unordered_map<std::string, std::string> locales_json = {";
for f in "${@}"; do
	key="${f%.*}"
	var="$(echo ${f} | sed -e "s/[-.]/_/g")"
	echo "  {\"${key}\", {${var}, ${var} + ${var}_len}},"
done
echo "};";

#install locales locally for testing
for loc in $(jq ".posix_locale" *.json | sed -e 's/"//g'); do
	localedef -i "${loc%.*}" -f "${loc##*.}" "./${loc}"
done
