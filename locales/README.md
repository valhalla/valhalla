# Narrative language files

Valhalla supports localized instructions in multiple languages for both textual and verbal phrases. All localized text is managed in JSON files in the [locales](https://github.com/valhalla/valhalla/tree/master/locales) directory. We rely on external contributors to provide translations of these phrases to other languages.

## Contributing translations

The recommended way to contribute translations is on [Transifex](https://www.transifex.com/). If you prefer you may also skip Transifex and edit the language files directly.

### Translating on Transifex (Recommended)

Follow these steps to start contributing a translation:

1. Sign up for a free account at https://www.transifex.com/.
2. Navigate to https://www.transifex.com/explore/projects/ and search for the `Valhalla Phrases` project.
3. Submit a request to join the team for the languages you know. You may also request a new language if yours does not appear in the list.
4. Wait for an email with an invitation to start translating.
5. From the [dashboard](https://www.transifex.com/valhalla/valhalla-phrases/dashboard/), navigate to the Translate page for your chosen language.
6. Select the `en-US.json` source file to translate from.
7. Before you start translating make sure to filter out any phrases with the `notranslate` tag. Apply this filter by selecting Tag > Doesn't contain tag > notranslate.
8. For more information on how to use Transifex, check out the [getting started guide](https://docs.transifex.com/getting-started-1/translators).

Once a language is 100% translated, the project maintainers will be notified and we will make a PR with your updates. Thank you for your contribution!

### Translating language files directly (Advanced)

Follow these instructions if you prefer to edit the JSON files directly without Transifex.

#### How to contribute a new narrative language file

1. Copy the [en-US.json](en-US.json) to `<NEW_LANGUAGE_TAG>.json`
Using [IETF BCP 47](https://tools.ietf.org/html/bcp47) as reference - the typical format for the `<NEW_LANGUAGE_TAG>` is:
<[ISO 639 two-letter language code](https://en.wikipedia.org/wiki/List_of_ISO_639-1_codes)>-<[ISO 3166 two-letter country code](https://en.wikipedia.org/wiki/ISO_3166-1_alpha-2)>
Czech/Czech Republic example:
`cs-CZ`

2. Update the `posix_locale` value in your new file. The character encoding must be UTF-8. The typical format is:
<[ISO 639 two-letter language code](https://en.wikipedia.org/wiki/List_of_ISO_639-1_codes)>_<[ISO 3166 two-letter country code](https://en.wikipedia.org/wiki/ISO_3166-1_alpha-2)>.UTF-8
Czech/Czech Republic `posix_locale` example:
`cs_CZ.UTF-8`

3. Update the `aliases` array in your new file. A typical alias entry is the [ISO 639 two-letter language code](https://en.wikipedia.org/wiki/List_of_ISO_639-1_codes) without the
[ISO 3166 two-letter country code](https://en.wikipedia.org/wiki/ISO_3166-1_alpha-2). The alias entry must be unique across language files.
Czech `aliases` entry example:
`cs`

4. Do not translate the JSON keys or phrase tags. An example using the ramp instruction:
![Alt text](img/do_not_translate.png)

5. Please translate the JSON values. As needed, reorder the phrase words and tags - the tags must remain in the phrase. An example using the ramp instruction:
![Alt text](img/translate.png)

6. Run `make check` to verify the tests pass OR move on to step#7 and we can help verify.

7. Submit a pull request for review. Thank you!

#### Adding new instruction strings

1. First add the new strings in en-US.json. The JSON keys are used by narrative_builder to select the instruction template string.

2. Sync the new strings to each new narrative language file by running `./merge-en.sh`. This will copy the new English strings to each new language.

3. Update the English strings in each language file.

#### Instruction descriptions
`TODO`

#### Tag descriptions
`TODO`

## Abbreviations

Note: abbreviations are not translated in Transifex, these should be added to the locale JSON files directly.

Locale files also include data about common abbreviations for cardinal directions, place names, and street types. These may be used by clients to shorten street names that appear in a text instruction displayed visually to a user.

The format consists of key-value pairs where the key is the long form of the word and the value is the shortened form in the same language.

The abbreviations come in three types:
- `cardinal_directions`: Cardinal direction words as they appear in street names, e.g. 16th Street Northwest.
- `road_labels`: [Street type designations](https://en.wikipedia.org/wiki/Street_or_road_name#Street_type_designations), such as those that appear as suffixes in English-speaking regions or prefixes in Spanish-speaking regions.
- `miscellaneous`: Miscellaneous words that are frequently abbreviated in street or place names.

The lists only include words commonly found in road names, and they only include abbreviations that a user would recognize instantly and unambiguously.
