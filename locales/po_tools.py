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

  po_tools.py po2json [--out DIR] [lang ...] .pot/.po -> locale JSONs (build step)
  po_tools.py update                         msgmerge valhalla.pot into every .po
  po_tools.py lint [lang ...]                placeholder token check
  po_tools.py posix-locales                  print every language's posix_locale (for localedef)

po2json and lint accept language tags (e.g. de-DE) to limit the run to those
files; without any, all locales/*.po are processed.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

LOCALES_DIR = Path(__file__).parent

# try to import the submodule polib
sys.path.insert(0, str(LOCALES_DIR.parent / "third_party" / "polib"))
try:
    import polib
except ImportError:
    sys.exit("polib not found - initialize submodules: git submodule update --init third_party/polib")

# the hand-maintained English source; also the msginit template for new languages
POT_FILE = LOCALES_DIR / "valhalla.pot"
TOKEN = re.compile(r"<[A-Z][A-Z_0-9]*>")
# NarrativeDictionary looks these up by numeric string key; every other
# container whose path segments are numeric is a real JSON array
NUMERIC_KEY_DICTS = ("phrases",)


def parse_po(path: Path) -> tuple[dict[str, polib.POEntry], dict[str, str]]:
    """Returns ({msgctxt: POEntry}, header metadata dict), skipping obsolete entries."""
    po = polib.pofile(str(path))
    return {e.msgctxt: e for e in po if not e.obsolete and e.msgctxt}, po.metadata


def parse_pot() -> tuple[list[polib.POEntry], dict[str, str]]:
    """Returns ([POEntry] in file order, header metadata dict)."""
    pot = polib.pofile(str(POT_FILE))
    return [e for e in pot if not e.obsolete and e.msgctxt], pot.metadata


def listify(key: str | None, node: Any) -> Any:
    """Turn all-numeric-key dicts into arrays, except the ones odin reads by key."""
    if not isinstance(node, dict):
        return node
    if node and all(k.isdigit() for k in node) and key not in NUMERIC_KEY_DICTS:
        return [listify(key, node[k]) for k in sorted(node, key=int)]
    return {k: listify(k, v) for k, v in node.items()}


def build_locale(
    pot_entries: list[polib.POEntry], entries: dict[str, polib.POEntry], meta: dict[str, str]
) -> dict[str, Any]:
    """Rebuild one language's JSON structure from the msgctxt paths."""
    root = {}
    for pot_entry in pot_entries:
        e = entries.get(pot_entry.msgctxt)
        translated = e.msgstr if e and e.msgstr and not e.fuzzy else pot_entry.msgid
        parts = pot_entry.msgctxt.split(".")
        node = root
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = translated
    out = listify(None, root)
    out["posix_locale"] = meta["X-Valhalla-Posix-Locale"]
    out["aliases"] = [a for a in meta.get("X-Valhalla-Aliases", "").split(",") if a]
    return out


def po_paths(langs: list[str] | None) -> list[Path]:
    if langs:
        return [LOCALES_DIR / f"{lang}.po" for lang in langs]
    return sorted(p for p in LOCALES_DIR.glob("*.po"))


def write_json(out_dir: Path, name: str, locale: dict[str, Any]) -> None:
    out_path = out_dir / f"{name}.json"
    out_path.write_text(json.dumps(locale, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out_path.name}")


def cmd_po2json(args: argparse.Namespace) -> None:
    pot_entries, pot_meta = parse_pot()
    args.out.mkdir(parents=True, exist_ok=True)
    # drop leftovers of removed languages so the build doesn't embed them
    for stale in args.out.glob("*.json"):
        if stale.stem != "en-US" and not (LOCALES_DIR / f"{stale.stem}.po").exists():
            stale.unlink()
    write_json(args.out, "en-US", build_locale(pot_entries, {}, pot_meta))
    for po in po_paths(args.langs):
        entries, meta = parse_po(po)
        write_json(args.out, po.stem, build_locale(pot_entries, entries, meta))


def cmd_posix_locales(_: argparse.Namespace) -> None:
    "Parses X-Valhalla-Posix-Locale header and returns"
    locales = {parse_pot()[1]["X-Valhalla-Posix-Locale"]}
    for po in po_paths(None):
        locales.add(polib.pofile(str(po)).metadata["X-Valhalla-Posix-Locale"])
    print("\n".join(sorted(locales)))


def cmd_update(_: argparse.Namespace) -> None:
    for po in po_paths(None):
        subprocess.run(
            ["msgmerge", "--update", "--no-wrap", "--backup=off", str(po), str(POT_FILE)], check=True
        )
        print(f"merged {po.name}")


def cmd_lint(args: argparse.Namespace) -> None:
    # NarrativeBuilder replaces every tag of an instruction subset regardless of
    # which phrase was selected, so a translation may legitimately use any token
    # that appears somewhere in its subset (error only outside that union, since
    # such tokens end up verbatim in user-facing instructions).
    pot_entries, _ = parse_pot()
    subset_tokens = {}
    for pot_entry in pot_entries:
        subset = ".".join(pot_entry.msgctxt.split(".")[:2])
        subset_tokens.setdefault(subset, set()).update(TOKEN.findall(pot_entry.msgid))

    errors = warnings = 0
    for po in po_paths(args.langs):
        entries, _ = parse_po(po)
        for path, e in entries.items():
            if not e.msgstr or e.fuzzy:
                continue
            want, got = set(TOKEN.findall(e.msgid)), set(TOKEN.findall(e.msgstr))
            allowed = subset_tokens.get(".".join(path.split(".")[:2]), set())
            if got - allowed:
                print(f"ERROR {po.name} [{path}]: unknown placeholder(s) {sorted(got - allowed)}")
                errors += 1
            elif got - want:
                print(f"warning {po.name} [{path}]: cross-phrase placeholder(s) {sorted(got - want)}")
                warnings += 1
            if want - got:
                print(f"warning {po.name} [{path}]: dropped placeholder(s) {sorted(want - got)}")
                warnings += 1
    print(f"{errors} errors, {warnings} warnings")
    sys.exit(1 if errors else 0)


def main() -> None:
    # add the file's docstring as program description
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )

    # add sub-parsers for all subcommands
    sub = parser.add_subparsers(required=True)
    # neat way to pass args directly to a function
    sub.add_parser("update").set_defaults(func=cmd_update)
    sub.add_parser("posix-locales").set_defaults(func=cmd_posix_locales)

    po2json = sub.add_parser("po2json")
    # if there's no args, we assume the files in LOCALES_DIR as input
    po2json.add_argument("langs", nargs="*")
    po2json.add_argument("--out", type=Path, default=LOCALES_DIR)
    po2json.set_defaults(func=cmd_po2json)

    lint = sub.add_parser("lint")
    lint.add_argument("langs", nargs="*")
    lint.set_defaults(func=cmd_lint)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
