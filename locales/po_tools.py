#!/usr/bin/env python3
"""gettext .po tooling for valhalla's locale JSONs.

The translation source of truth are the per-language .po files; the JSONs odin
embeds at build time are generated from them against the en-US.json template.

  po_tools.py json2pot                 en-US.json -> valhalla.pot
  po_tools.py json2po [locale ...]     translated JSON -> .po (initial migration)
  po_tools.py po2json [--out DIR]      .po -> locale JSONs (build step, embedded into locales.h)
  po_tools.py update                   json2pot + msgmerge --update into every .po
  po_tools.py lint                     placeholder token check on every .po
  po_tools.py posix-locales            print the posix_locale of every language (for localedef)

Entries are keyed by msgctxt = JSON path (e.g. instructions.bear.phrases.1),
msgid = English source string. example_phrases are attached as translator
comments, posix_locale/aliases live in the .po header (X-Valhalla-* fields).
Fuzzy ("needs work") and empty entries fall back to English in po2json.
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

LOCALES_DIR = Path(__file__).parent

# try to import the submodule polib
sys.path.insert(0, str(LOCALES_DIR.parent / "third_party" / "polib"))
try:
    import polib
except ImportError:
    sys.exit("polib not found - initialize submodules: git submodule update --init third_party/polib")
SOURCE_JSON = LOCALES_DIR / "en-US.json"
POT_FILE = LOCALES_DIR / "valhalla.pot"
# per-file metadata and English-only documentation, not translatable content
NON_TRANSLATABLE = ("posix_locale", "aliases", "example_phrases")
TOKEN = re.compile(r"<[A-Z][A-Z_0-9]*>")


def flatten(obj, prefix=""):
    """Yield (json_path, value) for every translatable leaf string."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k in NON_TRANSLATABLE:
                continue
            yield from flatten(v, f"{prefix}.{k}" if prefix else k)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            yield from flatten(v, f"{prefix}.{i}")
    elif isinstance(obj, str):
        yield prefix, obj


def examples_for(source, path):
    """English example_phrases documenting instructions.X.phrases.N, if any."""
    parts = path.split(".")
    if len(parts) == 4 and parts[2] == "phrases":
        ex = source[parts[0]][parts[1]].get("example_phrases", {})
        return ex.get(parts[3], [])
    return []


def po_escape(s):
    return s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def write_po(path, source, translations, metadata):
    """Write a .po (or .pot when translations is None) in canonical no-wrap form."""
    metadata = dict(metadata)
    lines = ['msgid ""', 'msgstr ""', '"Project-Id-Version: valhalla\\n"',
             '"Report-Msgid-Bugs-To: https://github.com/valhalla/valhalla/issues\\n"',
             f'"Language: {po_escape(metadata.pop("Language"))}\\n"',
             '"MIME-Version: 1.0\\n"', '"Content-Type: text/plain; charset=UTF-8\\n"',
             '"Content-Transfer-Encoding: 8bit\\n"']
    for k, v in metadata.items():
        lines.append(f'"{k}: {po_escape(v)}\\n"')
    lines.append("")

    for json_path, en_str in flatten(source):
        for ex in examples_for(source, json_path):
            lines.append(f"#. e.g. {ex}")
        lines.append(f"#: {json_path}")
        lines.append(f'msgctxt "{po_escape(json_path)}"')
        lines.append(f'msgid "{po_escape(en_str)}"')
        msgstr = translations.get(json_path, "") if translations is not None else ""
        lines.append(f'msgstr "{po_escape(msgstr)}"')
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_po(path):
    """Returns ({msgctxt: POEntry}, header metadata dict), skipping obsolete entries."""
    po = polib.pofile(str(path))
    return {e.msgctxt: e for e in po if not e.obsolete and e.msgctxt}, po.metadata


def substitute(obj, entries, prefix=""):
    """Deep-copy the en-US structure, replacing leaves with translations where available."""
    if isinstance(obj, dict):
        return {k: v if k in NON_TRANSLATABLE else
                substitute(v, entries, f"{prefix}.{k}" if prefix else k)
                for k, v in obj.items()}
    if isinstance(obj, list):
        return [substitute(v, entries, f"{prefix}.{i}") for i, v in enumerate(obj)]
    e = entries.get(prefix)
    return e.msgstr if e and e.msgstr and not e.fuzzy else obj


def po_paths(locales):
    if locales:
        return [LOCALES_DIR / f"{loc}.po" for loc in locales]
    return sorted(p for p in LOCALES_DIR.glob("*.po"))


def cmd_json2pot(_):
    source = json.loads(SOURCE_JSON.read_text(encoding="utf-8"))
    write_po(POT_FILE, source, None, {"Language": ""})
    print(f"wrote {POT_FILE.name}: {sum(1 for _ in flatten(source))} entries")


def cmd_json2po(args):
    source = json.loads(SOURCE_JSON.read_text(encoding="utf-8"))
    locales = args.locales or sorted(
        p.stem for p in LOCALES_DIR.glob("*.json") if p != SOURCE_JSON)
    for loc in locales:
        target = json.loads((LOCALES_DIR / f"{loc}.json").read_text(encoding="utf-8"))
        translations = dict(flatten(target))
        metadata = {
            "Language": loc.replace("-", "_"),
            "X-Valhalla-Posix-Locale": target["posix_locale"],
            "X-Valhalla-Aliases": ",".join(target.get("aliases", [])),
        }
        write_po(LOCALES_DIR / f"{loc}.po", source, translations, metadata)
        print(f"wrote {loc}.po: {len(translations)} entries")


def cmd_po2json(args):
    source = json.loads(SOURCE_JSON.read_text(encoding="utf-8"))
    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    # drop leftovers of removed languages so the build doesn't embed them
    for stale in out_dir.glob("*.json"):
        if stale.name != SOURCE_JSON.name and not (LOCALES_DIR / f"{stale.stem}.po").exists():
            stale.unlink()
    if out_dir.resolve() != LOCALES_DIR.resolve():
        shutil.copyfile(SOURCE_JSON, out_dir / SOURCE_JSON.name)
    for po in po_paths(args.locales):
        entries, meta = parse_po(po)
        out = substitute(source, entries)
        out["posix_locale"] = meta["X-Valhalla-Posix-Locale"]
        out["aliases"] = [a for a in meta.get("X-Valhalla-Aliases", "").split(",") if a]
        out_path = out_dir / f"{po.stem}.json"
        out_path.write_text(
            json.dumps(out, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"wrote {out_path.name}")


def cmd_posix_locales(_):
    locales = {json.loads(SOURCE_JSON.read_text(encoding="utf-8"))["posix_locale"]}
    for po in po_paths(None):
        locales.add(polib.pofile(str(po)).metadata["X-Valhalla-Posix-Locale"])
    print("\n".join(sorted(locales)))


def cmd_update(_):
    cmd_json2pot(_)
    for po in po_paths(None):
        subprocess.run(["msgmerge", "--update", "--no-wrap", "--backup=off",
                        str(po), str(POT_FILE)], check=True)
        print(f"merged {po.name}")


def cmd_lint(args):
    # NarrativeBuilder replaces every tag of an instruction subset regardless of
    # which phrase was selected, so a translation may legitimately use any token
    # that appears somewhere in its subset (error only outside that union, since
    # such tokens end up verbatim in user-facing instructions).
    source = json.loads(SOURCE_JSON.read_text(encoding="utf-8"))
    subset_tokens = {}
    for path, en_str in flatten(source):
        subset = ".".join(path.split(".")[:2])
        subset_tokens.setdefault(subset, set()).update(TOKEN.findall(en_str))

    errors = warnings = 0
    for po in po_paths(args.locales):
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


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("json2pot")
    for name in ("json2po", "po2json", "lint"):
        p = sub.add_parser(name)
        p.add_argument("locales", nargs="*")
        if name == "po2json":
            p.add_argument("--out", type=Path, default=LOCALES_DIR)
    sub.add_parser("update")
    sub.add_parser("posix-locales")
    args = parser.parse_args()
    {"json2pot": cmd_json2pot, "json2po": cmd_json2po, "po2json": cmd_po2json,
     "update": cmd_update, "lint": cmd_lint,
     "posix-locales": cmd_posix_locales}[args.cmd](args)


if __name__ == "__main__":
    main()
