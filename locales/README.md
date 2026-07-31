# Narrative language files

Valhalla supports localized instructions in multiple languages for both textual and verbal phrases. Translations are managed as gettext `.po` files in the [locales](https://github.com/valhalla/valhalla/tree/master/locales) directory — one per language, e.g. `de-DE.po`. We rely on external contributors to provide translations of these phrases.

The `gettext` files are the only committed translation artifacts: `valhalla.pot` is the hand-maintained English source (`msgid`s plus `#. e.g. ...` example-phrase comments; its header carries the en-US metadata), and each language has a `.po` with the translations. At build time, CMake reconstructs the per-language JSONs odin expects from them (`locales/po_tools.py po2json`, which parses the gettext files with [polib](https://pypi.org/project/polib/) — `pip install polib`) and embeds those into `libvalhalla` — no JSON exists in the repo at all.

## Contributing translations

Edit your language's `.po` file with [Poedit](https://poedit.net/) (recommended), any other gettext editor, or a plain text editor, then open a PR with just that file.

What to know while translating:

* Each entry shows the English source (`msgid`), your translation (`msgstr`), the JSON path as context (`msgctxt`, e.g. `instructions.bear.phrases.1`) and English example phrases as comments.
* Phrase tags like `<STREET_NAMES>` are replaced with real values at runtime. Reorder them as your grammar requires, but keep them spelled exactly as in the English source — a misspelled tag ends up verbatim in user-facing instructions. CI checks this (`po_tools.py lint`).
* Entries flagged **fuzzy** (Poedit: "Needs work") are ignored at runtime — the English source is used instead. They mark translations that need review, typically because the English phrase changed since they were translated. Filter for them to see what your language needs.
* Untranslated entries likewise fall back to English.

### Contributing a new language

1. Determine the language tag per [IETF BCP 47](https://tools.ietf.org/html/bcp47), typically `<`[ISO 639 two-letter language code](https://en.wikipedia.org/wiki/List_of_ISO_639-1_codes)`>-<`[ISO 3166 two-letter country code](https://en.wikipedia.org/wiki/ISO_3166-1_alpha-2)`>`, e.g. `cs-CZ`.
2. Create `locales/<tag>.po` from the template: `python3 locales/po_tools.py init cs-CZ`. It fills the header for you: `X-Valhalla-Posix-Locale` (default `cs_CZ.UTF-8`, override with `--posix-locale`) and `X-Valhalla-Aliases` (default the bare language code `cs`, unique across languages; override with `--aliases`, comma-separated or empty).
3. Translate.
4. Add a phrase for the new language to the `lang_phrase` vector in [`test/gurka/test_route_with_narrative_languages.cc`](https://github.com/valhalla/valhalla/blob/master/test/gurka/test_route_with_narrative_languages.cc) (easiest: add a bogus phrase, run the test, copy the expected one from the failure output).
5. Submit a pull request. Thank you!

## Maintainer workflow

### Changing or adding English phrases

1. Edit `locales/valhalla.pot` directly. Each entry is the `msgctxt` path (`instructions.<instruction>.phrases.<n>` — `narrative_builder` selects the phrase by that key), the English `msgid`, an empty `msgstr`, and optional `#. e.g. ...` example-phrase comments for translators. Path segments that are numbers become JSON arrays in the generated files, except under `phrases`, which odin reads by numeric string key.
2. Propagate to all languages (requires gettext):
   ```
   python3 locales/po_tools.py update
   ```
   `msgmerge` keeps every existing translation. Entries whose English changed keep the old translation but are flagged fuzzy (with the previous English kept as a `#|` comment), so each language's translators see exactly what needs review; new phrases appear untranslated. Both fall back to English until translated.
3. Commit the changed `valhalla.pot` and `*.po` files together.

### Tooling reference

All state lives in the `.pot`/`.po` files — no external service involved.

| Command | Purpose |
|---------|---------|
| `po_tools.py init <lang>` | Start a new language: create `<lang>.po` from the template with the header filled in |
| `po_tools.py update` | `msgmerge` the `valhalla.pot` template into every `.po` |
| `po_tools.py po2json [--out DIR]` | Generate the JSONs from the gettext files (fuzzy/empty → English); run by CMake at build time |
| `po_tools.py lint` | Check placeholder tokens; errors on tokens Odin would never substitute |
| `po_tools.py print-posix-locales` | Print every language's POSIX locale; used by the `localedef` test target |
| `msgattrib --untranslated --fuzzy <lang>.po` | List what needs work in a language |
| `msgfmt --check --statistics <lang>.po` | Validate syntax, show translation coverage |

CI (`lint.yml`, `locales` job) enforces: valid `.pot`/`.po` syntax, placeholder correctness, and that the JSONs generate cleanly.
