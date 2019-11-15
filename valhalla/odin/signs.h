#ifndef VALHALLA_ODIN_SIGNS_H_
#define VALHALLA_ODIN_SIGNS_H_

#include <cstdint>
#include <vector>

#include <valhalla/baldr/verbal_text_formatter.h>
#include <valhalla/baldr/verbal_text_formatter_us.h>

#include <valhalla/odin/sign.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace odin {

// TODO - comments

class Signs {
public:
  Signs();

  const std::vector<Sign>& exit_number_list() const;
  std::vector<Sign>* mutable_exit_number_list();

  std::string GetExitNumberString(uint32_t max_count = 0,
                                  bool limit_by_consecutive_count = false,
                                  std::string delim = "/",
                                  const VerbalTextFormatter* verbal_formatter = nullptr) const;

  const std::vector<Sign>& exit_branch_list() const;
  std::vector<Sign>* mutable_exit_branch_list();

  std::string GetExitBranchString(uint32_t max_count = 0,
                                  bool limit_by_consecutive_count = false,
                                  std::string delim = "/",
                                  const VerbalTextFormatter* verbal_formatter = nullptr) const;

  const std::vector<Sign>& exit_toward_list() const;
  std::vector<Sign>* mutable_exit_toward_list();

  std::string GetExitTowardString(uint32_t max_count = 0,
                                  bool limit_by_consecutive_count = false,
                                  std::string delim = "/",
                                  const VerbalTextFormatter* verbal_formatter = nullptr) const;

  const std::vector<Sign>& exit_name_list() const;
  std::vector<Sign>* mutable_exit_name_list();

  std::string GetExitNameString(uint32_t max_count = 0,
                                bool limit_by_consecutive_count = false,
                                std::string delim = "/",
                                const VerbalTextFormatter* verbal_formatter = nullptr) const;

  bool HasExit() const;
  bool HasExitNumber() const;
  bool HasExitBranch() const;
  bool HasExitToward() const;
  bool HasExitName() const;

  std::string ToString() const;

#ifdef LOGGING_LEVEL_TRACE
  std::string ToParameterString() const;
#endif

  bool operator==(const Signs& rhs) const;

protected:
  std::string ListToString(const std::vector<Sign>& signs,
                           uint32_t max_count = 0,
                           bool limit_by_consecutive_count = false,
                           const std::string& delim = "/",
                           const VerbalTextFormatter* verbal_formatter = nullptr) const;

#ifdef LOGGING_LEVEL_TRACE
  std::string ListToParameterString(const std::vector<Sign>& signs) const;
#endif

  std::vector<Sign> exit_number_list_;
  std::vector<Sign> exit_branch_list_;
  std::vector<Sign> exit_toward_list_;
  std::vector<Sign> exit_name_list_;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_SIGNS_H_
