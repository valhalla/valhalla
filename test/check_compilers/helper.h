#include <type_traits>

// a little helper to pretty-print _multiple_ static_assert failures
namespace helper {

// intentionally incomplete so it'll show up in the error message with its template args
template <typename...> struct fail_with;

template <typename Tag, bool B> struct named {
  // this will be condition
  static constexpr bool value = B;
  // this will be the "name" of it so we can recognize the static_assert in the error message
  using tag = Tag;
};

template <typename... Ns> inline void static_assert_all(Ns...) {
  constexpr bool all_ok = (Ns::value && ...);
  // bcs it's constexpr, this will be dropped by compiler if it's all fine
  if constexpr (!all_ok) {
    // build a type whose template args are the tags that failed
    // e.g. fail_with<void, std::is_trivially_copyable_v>
    using _ = fail_with<typename std::conditional<Ns::value, void, typename Ns::tag>::type...>;
    // instantiate the struct to trigger error
    (void)sizeof(_);

    static_assert(
        all_ok,
        "static_assert_all failed — see fail_with<> template arguments for the names of failed conditions");
  }
}

#define CT_NAMED(Name, Cond)                                                                         \
  ::helper::named<struct Name, static_cast<bool>(Cond)> {                                            \
  }

} // namespace helper
