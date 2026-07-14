# Narrative language files

Valhalla supports localized instructions in multiple languages for both textual and verbal phrases. Translations are managed as gettext `.po` files in the [locales](https://github.com/valhalla/valhalla/tree/master/locales) directory — one per language, e.g. `de-DE.po`. We rely on external contributors to provide translations of these phrases.

The `.po` files are the only committed translation artifact. At build time, CMake converts them into per-language JSONs (`locales/po_tools.py po2json`, which parses the `.po` files with the `third_party/polib` submodule) and embeds those into `libvalhalla` — there are no committed JSONs to keep in sync. `en-US.json` is the exception: it is the hand-maintained English source all other languages translate from.

## Contributing translations

Edit your language's `.po` file with [Poedit](https://poedit.net/) (recommended), any other gettext editor, or a plain text editor, then open a PR with just that file.

What to know while translating:

* Each entry shows the English source (`msgid`), your translation (`msgstr`), the JSON path as context (`msgctxt`, e.g. `instructions.bear.phrases.1`) and English example phrases as comments.
* Phrase tags like `<STREET_NAMES>` are replaced with real values at runtime. Reorder them as your grammar requires, but keep them spelled exactly as in the English source — a misspelled tag ends up verbatim in user-facing instructions. CI checks this (`po_tools.py lint`).
* Entries flagged **fuzzy** (Poedit: "Needs work") are ignored at runtime — the English source is used instead. They mark translations that need review, typically because the English phrase changed since they were translated. Filter for them to see what your language needs.
* Untranslated entries likewise fall back to English.

### Contributing a new language

1. Determine the language tag per [IETF BCP 47](https://tools.ietf.org/html/bcp47), typically `<`[ISO 639 two-letter language code](https://en.wikipedia.org/wiki/List_of_ISO_639-1_codes)`>-<`[ISO 3166 two-letter country code](https://en.wikipedia.org/wiki/ISO_3166-1_alpha-2)`>`, e.g. `cs-CZ`.
2. Create `locales/<tag>.po` from the template: `msginit --no-translator -i locales/valhalla.pot -o locales/cs-CZ.po -l cs_CZ.UTF-8`
3. In the `.po` header, set `X-Valhalla-Posix-Locale` (e.g. `cs_CZ.UTF-8`) and `X-Valhalla-Aliases` (comma-separated, typically the bare language code, e.g. `cs`; must be unique across languages; may be empty).
4. Translate.
5. Add a phrase for the new language to the `lang_phrase` vector in [`test/gurka/test_route_with_narrative_languages.cc`](https://github.com/valhalla/valhalla/blob/master/test/gurka/test_route_with_narrative_languages.cc) (easiest: add a bogus phrase, run the test, copy the expected one from the failure output).
6. Submit a pull request. Thank you!

## Maintainer workflow

### Changing or adding English phrases

1. Edit `locales/en-US.json` — the JSON keys are used by `narrative_builder` to select the instruction template.
2. Propagate to the template and all languages (requires gettext):
   ```
   python3 locales/po_tools.py update
   ```
   `msgmerge` keeps every existing translation. Entries whose English changed keep the old translation but are flagged fuzzy (with the previous English kept as a `#|` comment), so each language's translators see exactly what needs review; new phrases appear untranslated. Both fall back to English until translated.
3. Commit the changed `en-US.json`, `valhalla.pot` and `*.po` files together.

### Tooling reference

All state lives in the `.po` files — no external service involved.

| Command | Purpose |
|---------|---------|
| `po_tools.py update` | Regenerate `valhalla.pot` from `en-US.json` and `msgmerge` it into every `.po` |
| `po_tools.py po2json [--out DIR]` | Generate the JSONs from the `.po` files (fuzzy/empty → English); run by CMake at build time |
| `po_tools.py lint` | Check placeholder tokens; errors on tokens Odin would never substitute |
| `po_tools.py posix-locales` | Print every language's POSIX locale; used by the `localedef` test target |
| `po_tools.py json2pot` | Regenerate only the `.pot` template |
| `po_tools.py json2po [lang ...]` | One-time import of a translated JSON into a `.po` (used for the Transifex migration) |
| `msgattrib --untranslated --fuzzy <lang>.po` | List what needs work in a language |
| `msgfmt --check --statistics <lang>.po` | Validate syntax, show translation coverage |

CI (`lint.yml`, `locales` job) enforces: valid `.po` syntax, placeholder correctness, and that `valhalla.pot` is in sync with `en-US.json`.
