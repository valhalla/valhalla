"""Query-time name normalization.

One set of functions used on both the hint and the candidate names - the invariant that matters, and
the reason none of this is baked into tiles.

Written to port directly: an explicit UTF-8 decoder, a table lookup per codepoint, and character
loops rather than regex. `str.replace` in _collapse is the one Python convenience; in C++ it is a
find/replace loop.
"""

from translit import FOLD

REPLACEMENT = 0xFFFD

# a hint typed by a human uses these interchangeably with the plain vowel; applied only in the
# second, penalised pass because they discard real information
AGGRESSIVE_PAIRS = (("ue", "u"), ("oe", "o"), ("ae", "a"), ("ss", "s"))


class Normalized:
    """A name reduced to comparable form.

    `tokens` keeps word boundaries for the token-level rules; `stripped` throws them away, which is
    what the trigram bag wants - it is why `Hermannstrasse` and `Hermann Strasse` become the same
    string.
    """

    __slots__ = ("tokens", "joined", "stripped")

    def __init__(self, tokens):
        self.tokens = tokens
        self.joined = " ".join(tokens)
        self.stripped = "".join(tokens)

    def is_empty(self):
        return len(self.stripped) == 0


def decode_utf8(data):
    """Decode bytes to a list of codepoints. Malformed input yields U+FFFD, as in the C++ version."""
    codepoints = []
    index = 0
    length = len(data)
    while index < length:
        byte = data[index]
        if byte < 0x80:
            codepoints.append(byte)
            index += 1
            continue

        if 0xC2 <= byte <= 0xDF:
            needed, value = 1, byte & 0x1F
        elif 0xE0 <= byte <= 0xEF:
            needed, value = 2, byte & 0x0F
        elif 0xF0 <= byte <= 0xF4:
            needed, value = 3, byte & 0x07
        else:
            codepoints.append(REPLACEMENT)
            index += 1
            continue

        consumed = 0
        while consumed < needed:
            position = index + 1 + consumed
            if position >= length or (data[position] & 0xC0) != 0x80:
                break
            value = (value << 6) | (data[position] & 0x3F)
            consumed += 1

        if consumed != needed:
            codepoints.append(REPLACEMENT)
            index += 1
            continue

        codepoints.append(value)
        index += 1 + needed
    return codepoints


def _fold_codepoint(codepoint):
    """Return the ASCII form, or None when the codepoint separates tokens."""
    if codepoint < 0x80:
        if 0x30 <= codepoint <= 0x39:  # 0-9
            return chr(codepoint)
        if 0x61 <= codepoint <= 0x7A:  # a-z
            return chr(codepoint)
        if 0x41 <= codepoint <= 0x5A:  # A-Z
            return chr(codepoint + 0x20)
        # every other ASCII character is a separator: hyphens, dots, apostrophes, slashes, brackets
        return None

    folded = FOLD.get(codepoint)
    if folded is not None:
        return folded
    # outside the fold table - Greek, Cyrillic, CJK. Passed through so normalized-exact and trigram
    # comparison still work within one script; no case folding, which needs Unicode data we do not
    # ship. See README.
    return chr(codepoint)


def fold(text):
    """Lowercase, transliterate and split on anything that is not alphanumeric."""
    tokens = []
    current = []
    for codepoint in decode_utf8(text.encode("utf-8")):
        piece = _fold_codepoint(codepoint)
        if piece is None:
            if current:
                tokens.append("".join(current))
                current = []
        else:
            current.append(piece)
    if current:
        tokens.append("".join(current))
    return tokens


def _collapse(token):
    for digraph, single in AGGRESSIVE_PAIRS:
        token = token.replace(digraph, single)
    return token


def normalize_light(text):
    return Normalized(fold(text))


def normalize_aggressive(text):
    """The light pass plus vowel-digraph collapse, for the retry that carries a penalty."""
    tokens = []
    for token in fold(text):
        collapsed = _collapse(token)
        if collapsed:
            tokens.append(collapsed)
    return Normalized(tokens)


def split_on_comma(text):
    """Return (street part, discarded rest). `Doblerstr 4, Tuebingen` -> the street is before it."""
    position = text.find(",")
    if position < 0:
        return text, ""
    return text[:position].strip(), text[position + 1 :].strip()


def _starts_with_digit(token):
    return len(token) > 0 and "0" <= token[0] <= "9"


def _has_letter(token):
    for char in token:
        if not ("0" <= char <= "9"):
            return True
    return False


def strip_house_number(text):
    """Return (street part, house number). Handles `Hauptstr 12a` and `12 Main St`.

    Refuses to strip when only a short token would remain, so a ref like `B 28` survives. Stage 4
    checks for a ref before this runs anyway.
    """
    tokens = text.split()
    if len(tokens) < 2:
        return text, ""

    for index in (len(tokens) - 1, 0):
        if not _starts_with_digit(tokens[index]):
            continue
        rest = tokens[:index] + tokens[index + 1 :]
        if not any(_has_letter(token) for token in rest):
            continue
        if len(rest) == 1 and len(rest[0]) <= 3:
            continue
        return " ".join(rest), tokens[index]

    return text, ""
