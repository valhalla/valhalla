#pragma once

#include <algorithm>
#include <iterator>
#include <string_view>
#include <utility>

namespace valhalla {
namespace loki {
namespace street_name {

// lowercase ASCII form of the Latin letters that don't decompose in utf8proc, sorted by codepoint
// only the lowercase forms are listed, it assumes case folding has been done before searching here
// clang-format off
constexpr std::pair<char32_t, std::string_view> kDecomposeExtra[] = {
    {0x00E6, "ae"}, // æ
    {0x00F0, "d"}, // ð
    {0x00F8, "o"}, // ø
    {0x00FE, "th"}, // þ
    {0x0111, "d"}, // đ
    {0x0127, "h"}, // ħ
    {0x0131, "i"}, // ı
    {0x0138, "k"}, // ĸ
    {0x0142, "l"}, // ł
    {0x014B, "ng"}, // ŋ
    {0x0153, "oe"}, // œ
    {0x0167, "t"}, // ŧ
    {0x0180, "b"}, // ƀ
    {0x0183, "b"}, // ƃ
    {0x0188, "c"}, // ƈ
    {0x018C, "d"}, // ƌ
    {0x018D, "d"}, // ƍ
    {0x0192, "f"}, // ƒ
    {0x0195, "hv"}, // ƕ
    {0x0199, "k"}, // ƙ
    {0x019A, "l"}, // ƚ
    {0x019B, "l"}, // ƛ
    {0x019E, "n"}, // ƞ
    {0x01A3, "oi"}, // ƣ
    {0x01A5, "p"}, // ƥ
    {0x01AA, "s"}, // ƪ
    {0x01AB, "t"}, // ƫ
    {0x01AD, "t"}, // ƭ
    {0x01B4, "y"}, // ƴ
    {0x01B6, "z"}, // ƶ
    {0x01B9, "z"}, // ƹ
    {0x01BA, "z"}, // ƺ
    {0x01BF, "w"}, // ƿ
    {0x01DD, "e"}, // ǝ
    {0x01E5, "g"}, // ǥ
    {0x021D, "gh"}, // ȝ
    {0x0221, "d"}, // ȡ
    {0x0223, "ou"}, // ȣ
    {0x0225, "z"}, // ȥ
    {0x0234, "l"}, // ȴ
    {0x0235, "n"}, // ȵ
    {0x0236, "t"}, // ȶ
    {0x0237, "j"}, // ȷ
    {0x0238, "db"}, // ȸ
    {0x0239, "qp"}, // ȹ
    {0x023C, "c"}, // ȼ
    {0x023F, "s"}, // ȿ
    {0x0240, "z"}, // ɀ
    {0x0247, "e"}, // ɇ
    {0x0249, "j"}, // ɉ
    {0x024B, "q"}, // ɋ
    {0x024D, "r"}, // ɍ
    {0x024F, "y"}, // ɏ
    {0x0253, "b"}, // ɓ
    {0x0254, "o"}, // ɔ
    {0x0256, "d"}, // ɖ
    {0x0257, "d"}, // ɗ
    {0x0259, "e"}, // ə
    {0x025B, "e"}, // ɛ
    {0x0260, "g"}, // ɠ
    {0x0263, "g"}, // ɣ
    {0x0268, "i"}, // ɨ
    {0x0269, "i"}, // ɩ
    {0x026F, "m"}, // ɯ
    {0x0272, "n"}, // ɲ
    {0x0275, "o"}, // ɵ
    {0x0280, "r"}, // ʀ
    {0x0283, "s"}, // ʃ
    {0x0288, "t"}, // ʈ
    {0x0289, "u"}, // ʉ
    {0x028A, "u"}, // ʊ
    {0x028B, "v"}, // ʋ
    {0x028C, "v"}, // ʌ
    {0x0292, "z"}, // ʒ
    {0x2C65, "a"}, // ⱥ
    {0x2C66, "t"}, // ⱦ
};
// clang-format on
static_assert(std::is_sorted(std::begin(kDecomposeExtra), std::end(kDecomposeExtra)));

} // namespace street_name
} // namespace loki
} // namespace valhalla
