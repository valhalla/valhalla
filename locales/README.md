# How to contribute a new narrative language file

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

## Adding new instruction strings

1. First add the new strings in en-US.json. The JSON keys are used by narrative_builder to select the instruction template string.

2. Sync the new strings to each new narrative language file by running `./merge-en.sh`. This will copy the new English strings to each new language.

3. Update the English strings in each language file.

#### Instruction descriptions
`TODO`

#### Tag descriptions
`TODO`

