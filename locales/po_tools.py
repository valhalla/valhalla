#!/usr/bin/env python3
"""gettext tooling for valhalla's narrative locales.

valhalla.pot is the hand-maintained English source: msgctxt holds the JSON
path (e.g. instructions.bear.phrases.1), msgid the English phrase, and
"#. e.g. ..." comments carry example phrases shown to translators. The
per-language .po files hold the translations; en-US metadata (posix locale,
aliases) lives in the .pot header, every other language's in its .po header
(X-Valhalla-* fields). The JSONs odin embeds at build time are generated
from all of that by po2json; fuzzy ("needs work") and empty entries fall
back to English.

  po_tools.py init <lang>                    start a new language: <lang>.po from the template
  po_tools.py po2json [--out DIR] [lang ...] .pot/.po -> locale JSONs (build step)
  po_tools.py update                         msgmerge valhalla.pot into every .po
  po_tools.py lint [lang ...] [--fix]        placeholder token check + sort check (--fix sorts)
  po_tools.py stats [lang ...]               per-language translation coverage
  po_tools.py print-posix-locales            print every language's posix_locale (for localedef)

po2json, lint, stats and prune-english accept language tags (e.g. de-DE) to
limit the run to those files; without any, all locales/*.po are processed.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

LOCALES_DIR = Path(__file__).parent

try:
    import polib
except ImportError:
    sys.exit("polib not found - install it: pip install polib")

# the hand-maintained English source; also the msginit template for new languages
POT_FILE = LOCALES_DIR / "valhalla.pot"

# matches the phrase placeholders, e.g. <STREET_NAMES> or <TIME>
TOKEN = re.compile(r"<[A-Z][A-Z_0-9]*>")

# NarrativeDictionary looks these up by numeric string key; every other
# container whose path segments are numeric is a real JSON array
NUMERIC_KEY_DICTS = ("phrases",)


def parse_po(path: Path) -> tuple[dict[str, polib.POEntry], dict[str, str]]:
    """Returns ({msgctxt: POEntry}, header metadata dict), skipping obsolete entries."""
    po = polib.pofile(str(path))
    return {e.msgctxt: e for e in po if not e.obsolete and e.msgctxt}, po.metadata


def parse_pot() -> tuple[list[polib.POEntry], dict[str, str]]:
    """Returns ([POEntry] in file order, header metadata dict), skipping obsolete entries."""
    pot = polib.pofile(str(POT_FILE))
    # filters obsolete entries, but keeps fuzzy (untranslated) entries (defaults to en-US)
    return [e for e in pot if not e.obsolete and e.msgctxt], pot.metadata


def convert_to_json(key: str | None, node: Any) -> Any:
    """Turn all-numeric-key dicts into arrays, except the ones odin reads by key."""
    if not isinstance(node, dict):
        return node
    if node and all(k.isdigit() for k in node) and key not in NUMERIC_KEY_DICTS:
        return [convert_to_json(key, node[k]) for k in sorted(node, key=int)]
    return {k: convert_to_json(k, v) for k, v in node.items()}


def build_locale(
    pot_entries: list[polib.POEntry], entries: dict[str, polib.POEntry], meta: dict[str, str]
) -> dict[str, Any]:
    """Rebuild one language's JSON structure from the msgctxt paths."""
    po_root = {}

    # parse the .po files, but only if that string is not "fuzzy", i.e. not translated
    for pot_entry in pot_entries:
        e = entries.get(pot_entry.msgctxt)
        # fuzzy = translation needs review after its English source changed
        # use English until then
        translated = e.msgstr if e and e.msgstr and not e.fuzzy else pot_entry.msgid
        parts = pot_entry.msgctxt.split(".")
        node = po_root
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = translated

    # generate the output json compatible with our odin headers
    out_json = convert_to_json(None, po_root)
    out_json["posix_locale"] = meta["X-Valhalla-Posix-Locale"]
    out_json["aliases"] = [a for a in meta.get("X-Valhalla-Aliases", "").split(",") if a]

    return out_json


def po_paths(langs: list[str] | None) -> list[Path]:
    """
    Returns the .po files for ``langs``, or all of LOCALES_DIR's .po files if empty.
    Bails on language tags with no .po file.
    """
    if not langs:
        return sorted(p for p in LOCALES_DIR.glob("*.po"))
    paths = [p for lang in langs if (p := LOCALES_DIR / f"{lang}.po").exists()]

    # be nice and print what's actually available
    if len(paths) != len(langs):
        missing = sorted(set(langs).difference(p.stem for p in paths))
        available = ", ".join(sorted(p.stem for p in LOCALES_DIR.glob("*.po")))
        sys.exit(f"no .po file for: {', '.join(missing)}\navailable: {available}")
    return paths


def ctxt_key(msgctxt: str) -> list[int | str]:
    # natural order: numeric path segments sort as ints, so phrases.2 precedes phrases.10
    return [int(s) if s.isdigit() else s for s in msgctxt.split(".")]


def ctxt_sort_key(entry: polib.POEntry) -> list[int | str]:
    return ctxt_key(entry.msgctxt or "")


def is_sorted(path: Path) -> bool:
    """True if the file's entries are already in natural msgctxt order."""
    ctxts = [e.msgctxt for e in polib.pofile(str(path)) if e.msgctxt and not e.obsolete]
    return ctxts == sorted(ctxts, key=ctxt_key)


def sort_file(path: Path) -> bool:
    """Sort a .pot/.po in place by msgctxt; returns True if the order changed.

    Sorting the .pot matters too: msgmerge (update) reorders every .po to match it.
    wrapwidth=0 preserves no-wrap style.
    """
    po = polib.pofile(str(path), wrapwidth=0)
    before = [e.msgctxt for e in po]
    po.sort(key=ctxt_sort_key)
    if [e.msgctxt for e in po] == before:
        return False
    po.save(str(path))
    return True


def write_json(out_dir: Path, name: str, locale: dict[str, Any]) -> None:
    """Writes the output json the odin headers consume."""
    out_path = out_dir / f"{name}.json"
    out_path.write_text(json.dumps(locale, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out_path.name}")


def cmd_po2json(args: argparse.Namespace) -> None:
    """
    Converts .po files to our previous .json structure so that
    odin headers are staying compatible with old source code.
    """
    pot_entries, pot_meta = parse_pot()
    args.out.mkdir(parents=True, exist_ok=True)
    # first drop all .json before regenerating from actual .po files
    for stale in args.out.glob("*.json"):
        if stale.stem != "en-US" and not (LOCALES_DIR / f"{stale.stem}.po").exists():
            stale.unlink()

    # write the en-US first with _all_ entries, not filtered since it's the root
    write_json(args.out, "en-US", build_locale(pot_entries, {}, pot_meta))
    for po in po_paths(args.langs):
        entries, meta = parse_po(po)
        write_json(args.out, po.stem, build_locale(pot_entries, entries, meta))


def cmd_print_posix_locales(_: argparse.Namespace) -> None:
    "Parses X-Valhalla-Posix-Locale header and prints each locale"
    locales = {parse_pot()[1]["X-Valhalla-Posix-Locale"]}
    for po in po_paths(None):
        locales.add(polib.pofile(str(po)).metadata["X-Valhalla-Posix-Locale"])
    print("\n".join(sorted(locales)))


def cmd_init(args: argparse.Namespace) -> None:
    """Creates locales/<lang>.po from valhalla.pot, replacing msginit (no gettext needed)."""
    path = LOCALES_DIR / f"{args.lang}.po"
    if path.exists():
        sys.exit(f"{path.name} already exists")

    # locales must be unique; first add en-US and its alias(es)
    found_locales = set(("en-US",))
    found_locales.update(a for a in parse_pot()[1].get("X-Valhalla-Aliases", "").split(",") if a)

    # then add all langs + their alias(es)
    for po_file in po_paths(None):
        found_locales.add(po_file.stem)
        po_meta = polib.pofile(str(po_file)).metadata
        found_locales.update(a for a in po_meta.get("X-Valhalla-Aliases", "").split(",") if a)

    # verify that the new alias(es) don't clash with existing ones
    new_aliases = args.aliases if args.aliases is not None else args.lang.split("-")[0]
    if clashes := set(a for a in new_aliases.split(",") if a).intersection(found_locales):
        sys.exit(f"alias(es) already taken: {', '.join(sorted(clashes))} - pass --aliases (may be '')")

    # generate the new .po file from the valhalla.pot file
    po = polib.pofile(str(POT_FILE), wrapwidth=0)
    po.metadata["Language"] = args.lang.replace("-", "_")
    po.metadata["X-Valhalla-Posix-Locale"] = args.posix_locale or f"{args.lang.replace('-', '_')}.UTF-8"
    po.metadata["X-Valhalla-Aliases"] = new_aliases
    po.save(str(path))
    print(f"wrote {path.name}, see docs/docs/contributing/locales.md for the next steps")


def cmd_update(_: argparse.Namespace) -> None:
    """Merges valhalla.pot changes into every .po, flagging changed entries fuzzy."""
    if not shutil.which("msgmerge"):
        sys.exit("msgmerge not found - install gettext")
    for po in po_paths(None):
        subprocess.run(
            ["msgmerge", "--update", "--no-wrap", "--backup=off", str(po), str(POT_FILE)], check=True
        )
        print(f"merged {po.name}")


def cmd_lint(args: argparse.Namespace) -> None:
    # we lint specifically for NarrativeBuilder here:
    # - when replacing the tokens in e.g. phrase-dependent logic, we replace _all_ available
    #   tokens for _any_ phrase
    # - means a translation can use any tokens from any phrase, even if the english phrase
    #   doesn't have them: "cross-phrase warning"
    # - tokens appearing in translations which are not part of its phrase group: ERROR
    # - tokens being dropped in translations: warning

    pot_entries, _ = parse_pot()

    # every token an instruction knows, e.g. "instructions.bear" -> {<RELATIVE_DIRECTION>, ...},
    # collected over all of its English phrases
    subset_tokens = {}
    for pot_entry in pot_entries:
        # e.g. "instructions.bear_verbal.phrases.2" -> "instructions.bear_verbal"
        subset = ".".join(pot_entry.msgctxt.split(".")[:2])
        # collect all tokens/placeholders which are held by this subset
        subset_tokens.setdefault(subset, set()).update(TOKEN.findall(pot_entry.msgid))

    errors = warnings = 0
    for po in po_paths(args.langs):
        entries, _ = parse_po(po)
        for path, e in entries.items():
            # untranslated/fuzzy entries fall back to English, nothing to check
            if not e.msgstr or e.fuzzy:
                continue
            # tokens in the English phrase vs in its translation
            po_en_tokens, po_lang_tokens = set(TOKEN.findall(e.msgid)), set(TOKEN.findall(e.msgstr))
            # the instruction's full token set, e.g. "instructions.bear_verbal.phrases.2"
            # -> subset_tokens["instructions.bear_verbal"]
            allowed_pot_tokens = subset_tokens.get(".".join(path.split(".")[:2]), set())

            # if there's a diff between the .pot (i.e. en-US.po) and the lang tokens -> bail
            if po_lang_tokens.difference(allowed_pot_tokens):
                print(
                    f"ERROR {po.name} [{path}]: unknown placeholder(s) {sorted(po_lang_tokens.difference(allowed_pot_tokens))}"
                )
                errors += 1
            # if there is a diff between the .po's "msgid" and the "msgtxt" -> warn
            # msgid = en-US, msgtxt = lang; this is core "gettext" design
            elif po_lang_tokens.difference(po_en_tokens):
                print(
                    f"warning {po.name} [{path}]: cross-phrase placeholder(s) {sorted(po_lang_tokens.difference(po_en_tokens))}"
                )
                warnings += 1
            # we dropped a token compared to english -> warn
            if po_en_tokens.difference(po_lang_tokens):
                print(
                    f"warning {po.name} [{path}]: dropped placeholder(s) {sorted(po_en_tokens.difference(po_lang_tokens))}"
                )
                warnings += 1

    # entries must stay in natural msgctxt order; --fix sorts in place, otherwise it's an error
    for path in [POT_FILE, *po_paths(args.langs)]:
        if is_sorted(path):
            continue
        if args.fix:
            sort_file(path)
            print(f"sorted {path.name}")
        else:
            print(f"ERROR {path.name}: not sorted - run: po_tools.py lint --fix")
            errors += 1

    print(f"{errors} errors, {warnings} warnings")
    sys.exit(1 if errors else 0)


def cmd_stats(args: argparse.Namespace) -> None:
    # en-X variants are horribly underestimated (seem untranslated)
    pot_entries, _ = parse_pot()
    total = len(pot_entries)

    stats = {}
    for po in po_paths(args.langs):
        entries, _ = parse_po(po)
        translated = fuzzy = 0
        for pot_entry in pot_entries:
            e = entries.get(pot_entry.msgctxt)
            if e and e.fuzzy:
                fuzzy += 1
            elif e and e.msgstr and e.msgstr != e.msgid:
                translated += 1
        stats[po.stem] = {
            "translated": translated,
            "fuzzy": fuzzy,
            "untranslated": total - translated - fuzzy,
            "total": total,
            "percent": round(100 * translated / total, 1),
        }

    print(json.dumps(stats, indent=2))


def main() -> None:
    # add the file's docstring as program description
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )

    # add sub-parsers for all subcommands
    sub = parser.add_subparsers(required=True)
    # neat way to pass args directly to a function
    sub.add_parser("update").set_defaults(func=cmd_update)
    sub.add_parser("print-posix-locales").set_defaults(func=cmd_print_posix_locales)

    # verifies input is not colliding with existing locales/aliases
    # and writes the new .po file from the valhalla.pot template
    init = sub.add_parser("init")
    init.add_argument("lang")
    init.add_argument("--aliases", help="comma-separated, defaults to the bare language code")
    init.add_argument("--posix-locale", help="defaults to <lang>.UTF-8 with underscores")
    init.set_defaults(func=cmd_init)

    po2json = sub.add_parser("po2json")
    # if there's no args, we assume the files in LOCALES_DIR as input
    po2json.add_argument("langs", nargs="*")
    po2json.add_argument("--out", type=Path, default=LOCALES_DIR)
    po2json.set_defaults(func=cmd_po2json)

    # compare .pot tokens with lang tokens
    # also cross-compare msgid & msgtxt of each entry
    lint = sub.add_parser("lint")
    lint.add_argument("langs", nargs="*")
    lint.add_argument(
        "--fix", action="store_true", help="sort unsorted files in place instead of erroring"
    )
    lint.set_defaults(func=cmd_lint)

    # per-language translation coverage (fuzzy counted separately, not as translated)
    stats = sub.add_parser("stats")
    stats.add_argument("langs", nargs="*")
    stats.set_defaults(func=cmd_stats)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
