#!/bin/bash
set -e

#install locales locally for testing
if [[ "$OSTYPE" == "linux"* ]]; then
	for loc in $(grep -F posix_locale *.json | sed -e "s/.*locale[^a-z^A-Z]\+//g" -e "s/[^a-z^A-Z^0-9^.^_]\+//g"); do
		localedef -i ${loc%.*} -f UTF-8 ./${loc}
	done
fi

#throw all the text into one big header
code=
for f in "${@}"; do
	code="$(xxd -i ${f})
${code}"
done

#have a map to iterate over
map="$(echo "${code}" | grep -F unsigned | sed -e "s/.* \([a-zA-Z_]\+\)\[\] =.*/{\"\1\",{\1,\1+/g" -e "s/.* \([a-zA-Z_]\+\) =.*/\1}},/g" | tr -d '\n' | sed -e "s/}},{/}},\n  {/g")"
for key in $(echo "${map}" | grep -F '_json"' | sed -e "s/.*{\"//g" -e "s/\",.*//g"); do
	k="$(echo "${key}" | sed -e "s/_json//g" -e "s/_/-/g")"
	map="$(echo "${map}" | sed -e "s/\"${key}\"/\"${k}\"/g")"
done

#output the code
echo "#include <unordered_map>"
echo "${code}"
echo "const std::unordered_map<std::string, std::string> locales_json = {";
echo "  ${map}"
echo "};";
