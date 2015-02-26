#include "odin/signs.h"

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

const std::string Signs::GetExitNumberString() const {
  return ListToString(exit_number_list_);
}

const std::vector<Sign>& Signs::exit_branch_list() const {
  return exit_branch_list_;
}

std::vector<Sign>* Signs::mutable_exit_branch_list() {
  return &exit_branch_list_;
}

const std::string Signs::GetExitBranchString() const {
  return ListToString(exit_branch_list_);
}

const std::vector<Sign>& Signs::exit_toward_list() const {
  return exit_toward_list_;
}

std::vector<Sign>* Signs::mutable_exit_toward_list() {
  return &exit_toward_list_;
}

const std::string Signs::GetExitTowardString() const {
  return ListToString(exit_toward_list_);
}

const std::vector<Sign>& Signs::exit_name_list() const {
  return exit_name_list_;
}

std::vector<Sign>* Signs::mutable_exit_name_list() {
  return &exit_name_list_;
}

const std::string Signs::GetExitNameString() const {
  return ListToString(exit_name_list_);
}

bool Signs::HasExit() const {
  return ((exit_number_list_.size() > 0) || (exit_branch_list_.size() > 0)
      || (exit_toward_list_.size() > 0) || (exit_name_list_.size() > 0));
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

std::string Signs::ToUnitTestString() const {
  const std::string delim = ", ";
  std::string signs_string;

  signs_string += ListToUnitTestString(exit_number_list_);

  signs_string += delim;
  signs_string += ListToUnitTestString(exit_branch_list_);

  signs_string += delim;
  signs_string += ListToUnitTestString(exit_toward_list_);

  signs_string += delim;
  signs_string += ListToUnitTestString(exit_name_list_);

  return signs_string;
}

const std::string Signs::ListToString(const std::vector<Sign>& signs) const {
  std::string sign_string;
  for (auto& sign : signs) {
    if (!sign_string.empty()) {
      sign_string += "/";
    }
    sign_string += sign.text();
  }
  return sign_string;
}

const std::string Signs::ListToUnitTestString(
    const std::vector<Sign>& signs) const {
  const std::string delim = ", ";
  std::string sign_string;
  bool is_first = true;
  sign_string += "{ ";
  for (auto& sign : signs) {
    if (is_first)
      is_first = false;
    else
      sign_string += delim;
    sign_string += sign.ToUnitTestString();
  }
  sign_string += " }";

  return sign_string;
}

}
}
