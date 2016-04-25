# How to contribute a new narrative language file

1. Copy the en-US.json to `<NEW_LANGUAGE_TAG>.json`
Using [IETF BCP 47](https://tools.ietf.org/html/bcp47) as reference - the typical format for the `<NEW_LANGUAGE_TAG>` is:  
<[ISO 639 two-letter language code](TBD)>-<[ISO 3166 two-letter country code](TBD)>  
Czech/Czech Republic example:  
cs-CZ

2. Update the `posix_locale` value - typical format is:
    <[ISO 639 two-letter language code](TBD)>_<[ISO 3166 two-letter country code](TBD)>.<character encoding>
Czech/Czech Republic example:
    cs_CZ.utf8

3. Do not translate the JSON keys or phrase tags, for example:
PIC

4. Please translate the JSON values. As needed, reorder the phrases words and tags - the tags must remain in the phrase.
PIC

### Instruction descriptions
TBD

### Tag descriptions
TBD

