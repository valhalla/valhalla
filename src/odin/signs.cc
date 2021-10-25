#include <algorithm>
#include <cmath>
#include <utility>

#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"

#include "odin/signs.h"

using namespace valhalla::baldr;

namespace {
// The number of guide sign types (i.e. branch and toward)
constexpr uint32_t kNumberOfGuideSignTypes = 2;
} // namespace

namespace valhalla {
namespace odin {

Signs::Signs() {
}

void Signs::Sort(std::vector<Sign>* signs) {
  // Sort signs by descending consecutive count order
  std::sort(signs->begin(), signs->end(),
            [](Sign a, Sign b) { return b.consecutive_count() < a.consecutive_count(); });
}

void Signs::CountAndSort(std::vector<Sign>* prev_signs, std::vector<Sign>* curr_signs) {
  // Increment count for consecutive exit signs
  for (Sign& curr_sign : *curr_signs) {
    for (Sign& prev_sign : *prev_signs) {
      if (curr_sign.text() == prev_sign.text()) {
        curr_sign.set_consecutive_count(curr_sign.consecutive_count() + 1);
        prev_sign.set_consecutive_count(curr_sign.consecutive_count());
      }
    }
  }

  // Sort the previous and current exit signs by descending consecutive count
  Sort(prev_signs);
  Sort(curr_signs);
}

const std::vector<Sign>& Signs::exit_number_list() const {
  return exit_number_list_;
}

std::vector<Sign>* Signs::mutable_exit_number_list() {
  return &exit_number_list_;
}

std::string Signs::GetExitNumberString(uint32_t max_count,
                                       bool limit_by_consecutive_count,
                                       const std::string& delim,
                                       const VerbalTextFormatter* verbal_formatter,
                                       const MarkupFormatter* markup_formatter) const {
  return ListToString(exit_number_list_, max_count, limit_by_consecutive_count, delim,
                      verbal_formatter, markup_formatter);
}

const std::vector<Sign>& Signs::exit_branch_list() const {
  return exit_branch_list_;
}

std::vector<Sign>* Signs::mutable_exit_branch_list() {
  return &exit_branch_list_;
}

std::string Signs::GetExitBranchString(uint32_t max_count,
                                       bool limit_by_consecutive_count,
                                       const std::string& delim,
                                       const VerbalTextFormatter* verbal_formatter,
                                       const MarkupFormatter* markup_formatter) const {
  return ListToString(exit_branch_list_, max_count, limit_by_consecutive_count, delim,
                      verbal_formatter, markup_formatter);
}

const std::vector<Sign>& Signs::exit_toward_list() const {
  return exit_toward_list_;
}

std::vector<Sign>* Signs::mutable_exit_toward_list() {
  return &exit_toward_list_;
}

std::string Signs::GetExitTowardString(uint32_t max_count,
                                       bool limit_by_consecutive_count,
                                       const std::string& delim,
                                       const VerbalTextFormatter* verbal_formatter,
                                       const MarkupFormatter* markup_formatter) const {
  return ListToString(exit_toward_list_, max_count, limit_by_consecutive_count, delim,
                      verbal_formatter, markup_formatter);
}

const std::vector<Sign>& Signs::exit_name_list() const {
  return exit_name_list_;
}

std::vector<Sign>* Signs::mutable_exit_name_list() {
  return &exit_name_list_;
}

std::string Signs::GetExitNameString(uint32_t max_count,
                                     bool limit_by_consecutive_count,
                                     const std::string& delim,
                                     const VerbalTextFormatter* verbal_formatter,
                                     const MarkupFormatter* markup_formatter) const {
  return ListToString(exit_name_list_, max_count, limit_by_consecutive_count, delim, verbal_formatter,
                      markup_formatter);
}

const std::vector<Sign>& Signs::guide_branch_list() const {
  return guide_branch_list_;
}

std::vector<Sign>* Signs::mutable_guide_branch_list() {
  return &guide_branch_list_;
}

std::string Signs::GetGuideBranchString(uint32_t max_count,
                                        bool limit_by_consecutive_count,
                                        const std::string& delim,
                                        const VerbalTextFormatter* verbal_formatter,
                                        const MarkupFormatter* markup_formatter) const {
  return ListToString(guide_branch_list_, max_count, limit_by_consecutive_count, delim,
                      verbal_formatter, markup_formatter);
}

const std::vector<Sign>& Signs::guide_toward_list() const {
  return guide_toward_list_;
}

std::vector<Sign>* Signs::mutable_guide_toward_list() {
  return &guide_toward_list_;
}

std::string Signs::GetGuideTowardString(uint32_t max_count,
                                        bool limit_by_consecutive_count,
                                        const std::string& delim,
                                        const VerbalTextFormatter* verbal_formatter,
                                        const MarkupFormatter* markup_formatter) const {
  return ListToString(guide_toward_list_, max_count, limit_by_consecutive_count, delim,
                      verbal_formatter, markup_formatter);
}

/* NOTE: This is functionally similar to GetGuideString() except that it
 * returns a new list of merged guide signs. Its implemented separately here
 * due to performance concerns (since it creates a new vector on each
 * invocation).
 *
 * Any change to functionally of GetGuideString() should be made here as well.
 */
std::vector<Sign> Signs::GetGuideSigns(uint32_t max_count, bool limit_by_consecutive_count) const {
  // If both branch and toward exist
  // and either unlimited max count or max count is greater than 1
  // then process guide sign info splitting between branch and toward signs
  if (HasGuideBranch() && HasGuideToward() && (max_count != 1)) {
    // Round using floating point division
    std::vector<Sign> guide_branch =
        TrimSigns(guide_branch_list(),
                  static_cast<uint32_t>(
                      std::round(static_cast<float>(max_count) / kNumberOfGuideSignTypes)),
                  limit_by_consecutive_count);
    // Truncate using integer division
    std::vector<Sign> guide_toward =
        TrimSigns(guide_toward_list(), (max_count / kNumberOfGuideSignTypes),
                  limit_by_consecutive_count);

    std::vector<Sign> guide_signs;
    guide_signs.reserve(guide_branch.size() + guide_toward.size());

    guide_signs.insert(guide_signs.end(), guide_branch.cbegin(), guide_branch.cend());
    guide_signs.insert(guide_signs.end(), guide_toward.cbegin(), guide_toward.cend());
    return guide_signs;
  } else if (HasGuideBranch()) {
    return TrimSigns(guide_branch_list(), max_count, limit_by_consecutive_count);
  } else if (HasGuideToward()) {
    return TrimSigns(guide_toward_list(), max_count, limit_by_consecutive_count);
  }
  return {};
}

std::string Signs::GetGuideString(uint32_t max_count,
                                  bool limit_by_consecutive_count,
                                  const std::string& delim,
                                  const VerbalTextFormatter* verbal_formatter,
                                  const MarkupFormatter* markup_formatter) const {
  std::string guide_string;
  // If both branch and toward exist
  // and either unlimited max count or max count is greater than 1
  // then process guide sign info splitting between branch and toward signs
  if (HasGuideBranch() && HasGuideToward() && ((max_count == 0) || (max_count > 1))) {
    // Round using floating point division
    std::string guide_branch =
        GetGuideBranchString(static_cast<uint32_t>(
                                 std::round(static_cast<float>(max_count) / kNumberOfGuideSignTypes)),
                             limit_by_consecutive_count, delim, verbal_formatter, markup_formatter);
    // Truncate using integer division
    std::string guide_toward =
        GetGuideTowardString((max_count / kNumberOfGuideSignTypes), limit_by_consecutive_count, delim,
                             verbal_formatter, markup_formatter);
    guide_string = guide_branch + delim + guide_toward;
  } else if (HasGuideBranch()) {
    guide_string = GetGuideBranchString(max_count, limit_by_consecutive_count, delim,
                                        verbal_formatter, markup_formatter);
  } else if (HasGuideToward()) {
    guide_string = GetGuideTowardString(max_count, limit_by_consecutive_count, delim,
                                        verbal_formatter, markup_formatter);
  }
  return guide_string;
}

const std::vector<Sign>& Signs::junction_name_list() const {
  return junction_name_list_;
}

std::vector<Sign>* Signs::mutable_junction_name_list() {
  return &junction_name_list_;
}

std::string Signs::GetJunctionNameString(uint32_t max_count,
                                         bool limit_by_consecutive_count,
                                         const std::string& delim,
                                         const VerbalTextFormatter* verbal_formatter,
                                         const MarkupFormatter* markup_formatter) const {
  return ListToString(junction_name_list_, max_count, limit_by_consecutive_count, delim,
                      verbal_formatter, markup_formatter);
}

bool Signs::HasExit() const {
  return (HasExitNumber() || HasExitBranch() || HasExitToward() || HasExitName());
}

bool Signs::HasExitNumber() const {
  return (exit_number_list_.size() > 0);
}

bool Signs::HasExitBranch() const {
  return (exit_branch_list_.size() > 0);
}

bool Signs::HasExitToward() const {
  return (exit_toward_list_.size() > 0);
}

bool Signs::HasExitName() const {
  return (exit_name_list_.size() > 0);
}

bool Signs::HasGuide() const {
  return (HasGuideBranch() || HasGuideToward());
}

bool Signs::HasGuideBranch() const {
  return (guide_branch_list_.size() > 0);
}

bool Signs::HasGuideToward() const {
  return (guide_toward_list_.size() > 0);
}

bool Signs::HasJunctionName() const {
  return (junction_name_list_.size() > 0);
}

std::string Signs::ToString() const {
  std::string signs_string;

  signs_string += "exit_numbers=";
  signs_string += GetExitNumberString();

  signs_string += " | exit_onto_streets=";
  signs_string += GetExitBranchString();

  signs_string += " | exit_toward_locations=";
  signs_string += GetExitTowardString();

  signs_string += " | exit_names=";
  signs_string += GetExitNameString();

  signs_string += " | guide_onto_streets=";
  signs_string += GetGuideBranchString();

  signs_string += " | guide_toward_locations=";
  signs_string += GetGuideTowardString();

  signs_string += " | junction_names=";
  signs_string += GetJunctionNameString();

  return signs_string;
}

#ifdef LOGGING_LEVEL_TRACE
std::string Signs::ToParameterString() const {
  const std::string delim = ", ";
  std::string signs_string;

  signs_string += ListToParameterString(exit_number_list_);

  signs_string += delim;
  signs_string += ListToParameterString(exit_branch_list_);

  signs_string += delim;
  signs_string += ListToParameterString(exit_toward_list_);

  signs_string += delim;
  signs_string += ListToParameterString(exit_name_list_);

  signs_string += delim;
  signs_string += ListToParameterString(guide_branch_list_);

  signs_string += delim;
  signs_string += ListToParameterString(guide_toward_list_);

  signs_string += delim;
  signs_string += ListToParameterString(junction_name_list_);

  return signs_string;
}
#endif

/* NOTE: This is functionally similar to ListToString() except that it
 * creates a new list of signs. Its implemented separately here due to
 * performance concerns (due to creation of a new vector).
 *
 * Any change to functionally of ListToString() should be made here as well.
 */
std::vector<Sign> Signs::TrimSigns(const std::vector<Sign>& signs,
                                   uint32_t max_count,
                                   bool limit_by_consecutive_count) {
  std::vector<Sign> trimmed_signs;

  uint32_t count = 0;
  uint32_t consecutive_count = 0;

  for (auto& sign : signs) {
    // If supplied, limit by max count
    if ((max_count > 0) && (count == max_count)) {
      break;
    }

    // if requested, process consecutive exit counts
    if (limit_by_consecutive_count) {

      // Set consecutive count of first sign
      if (count == 0) {
        consecutive_count = sign.consecutive_count();
      }
      // Limit if consecutive count does not match
      // Therefore, the most consistent information is displayed for user
      // and reduces clutter
      else if (sign.consecutive_count() != consecutive_count) {
        break;
      }
    }

    trimmed_signs.emplace_back(sign);
    ++count;
  }
  return trimmed_signs;
}

std::string Signs::ListToString(const std::vector<Sign>& signs,
                                uint32_t max_count,
                                bool limit_by_consecutive_count,
                                const std::string& delim,
                                const VerbalTextFormatter* verbal_formatter,
                                const MarkupFormatter* markup_formatter) const {
  std::string sign_string;
  uint32_t count = 0;
  uint32_t consecutive_count = -1;

  for (auto& sign : signs) {
    // If supplied, limit by max count
    if ((max_count > 0) && (count == max_count)) {
      break;
    }

    // if requested, process consecutive exit counts
    if (limit_by_consecutive_count) {

      // Set consecutive count of first sign
      if (count == 0) {
        consecutive_count = sign.consecutive_count();
      }
      // Limit if consecutive count does not match
      // Therefore, the most consistent information is displayed for user
      // and reduces clutter
      else if (sign.consecutive_count() != consecutive_count) {
        break;
      }
    }

    // Add delimiter
    if (!sign_string.empty()) {
      sign_string += delim;
    }

    // Concatenate exit text and update count
    sign_string +=
        (verbal_formatter) ? verbal_formatter->Format(sign, markup_formatter) : sign.text();
    ++count;
  }

  return sign_string;
}

#ifdef LOGGING_LEVEL_TRACE
std::string Signs::ListToParameterString(const std::vector<Sign>& signs) const {
  std::string sign_string;
  std::string param_list;

  sign_string += "{ ";
  for (const auto& sign : signs) {
    if (!param_list.empty()) {
      param_list += ", ";
    }
    param_list += sign.ToParameterString();
  }
  sign_string += param_list;
  sign_string += " }";

  return sign_string;
}
#endif

bool Signs::operator==(const Signs& rhs) const {
  return ((exit_number_list_ == rhs.exit_number_list_) &&
          (exit_branch_list_ == rhs.exit_branch_list_) &&
          (exit_toward_list_ == rhs.exit_toward_list_) && (exit_name_list_ == rhs.exit_name_list_) &&
          (guide_branch_list_ == rhs.guide_branch_list_) &&
          (guide_toward_list_ == rhs.guide_toward_list_) &&
          (junction_name_list_ == rhs.junction_name_list_));
}

} // namespace odin
} // namespace valhalla
