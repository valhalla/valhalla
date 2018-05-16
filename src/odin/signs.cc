#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"

#include "odin/signs.h"

#include <utility>

using namespace valhalla::baldr;

namespace valhalla {
namespace odin {

Signs::Signs() {
}

const std::vector<Sign>& Signs::exit_number_list() const {
  return exit_number_list_;
}

std::vector<Sign>* Signs::mutable_exit_number_list() {
  return &exit_number_list_;
}

const std::string Signs::GetExitNumberString(uint32_t max_count,
                                             bool limit_by_consecutive_count,
                                             std::string delim,
                                             const VerbalTextFormatter* verbal_formatter) const {
  return ListToString(exit_number_list_, max_count, limit_by_consecutive_count, std::move(delim),
                      verbal_formatter);
}

const std::vector<Sign>& Signs::exit_branch_list() const {
  return exit_branch_list_;
}

std::vector<Sign>* Signs::mutable_exit_branch_list() {
  return &exit_branch_list_;
}

const std::string Signs::GetExitBranchString(uint32_t max_count,
                                             bool limit_by_consecutive_count,
                                             std::string delim,
                                             const VerbalTextFormatter* verbal_formatter) const {
  return ListToString(exit_branch_list_, max_count, limit_by_consecutive_count, std::move(delim),
                      verbal_formatter);
}

const std::vector<Sign>& Signs::exit_toward_list() const {
  return exit_toward_list_;
}

std::vector<Sign>* Signs::mutable_exit_toward_list() {
  return &exit_toward_list_;
}

const std::string Signs::GetExitTowardString(uint32_t max_count,
                                             bool limit_by_consecutive_count,
                                             std::string delim,
                                             const VerbalTextFormatter* verbal_formatter) const {
  return ListToString(exit_toward_list_, max_count, limit_by_consecutive_count, std::move(delim),
                      verbal_formatter);
}

const std::vector<Sign>& Signs::exit_name_list() const {
  return exit_name_list_;
}

std::vector<Sign>* Signs::mutable_exit_name_list() {
  return &exit_name_list_;
}

const std::string Signs::GetExitNameString(uint32_t max_count,
                                           bool limit_by_consecutive_count,
                                           std::string delim,
                                           const VerbalTextFormatter* verbal_formatter) const {
  return ListToString(exit_name_list_, max_count, limit_by_consecutive_count, std::move(delim),
                      verbal_formatter);
}

bool Signs::HasExit() const {
  return ((exit_number_list_.size() > 0) || (exit_branch_list_.size() > 0) ||
          (exit_toward_list_.size() > 0) || (exit_name_list_.size() > 0));
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

std::string Signs::ToString() const {
  std::string signs_string;

  signs_string += "exit.number=";
  signs_string += GetExitNumberString();

  signs_string += " | exit.branch=";
  signs_string += GetExitBranchString();

  signs_string += " | exit.toward=";
  signs_string += GetExitTowardString();

  signs_string += " | exit.name=";
  signs_string += GetExitNameString();

  return signs_string;
}

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

  return signs_string;
}

const std::string Signs::ListToString(const std::vector<Sign>& signs,
                                      uint32_t max_count,
                                      bool limit_by_consecutive_count,
                                      const std::string& delim,
                                      const VerbalTextFormatter* verbal_formatter) const {
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

      // Set consecutive count if first sign
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
    sign_string += (verbal_formatter) ? verbal_formatter->Format(sign.text()) : sign.text();
    ++count;
  }

  return sign_string;
}

const std::string Signs::ListToParameterString(const std::vector<Sign>& signs) const {
  const std::string delim = ", ";
  std::string sign_string;
  bool is_first = true;
  sign_string += "{ ";
  for (auto& sign : signs) {
    if (is_first) {
      is_first = false;
    } else {
      sign_string += delim;
    }
    sign_string += sign.ToParameterString();
  }
  sign_string += " }";

  return sign_string;
}

bool Signs::operator==(const Signs& rhs) const {
  return ((exit_number_list_ == rhs.exit_number_list_) &&
          (exit_branch_list_ == rhs.exit_branch_list_) &&
          (exit_toward_list_ == rhs.exit_toward_list_) && (exit_name_list_ == rhs.exit_name_list_));
}

} // namespace odin
} // namespace valhalla
