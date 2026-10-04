#pragma once

#include <utility>

namespace scan_planner {
// Release a worker's borrowed real-map lease on every return/exception path.
// The guard itself allocates nothing and its cleanup is explicitly noexcept.
template<class Release> class SolveScopeExit {
public:
  explicit SolveScopeExit(Release release) : release_(std::move(release)) {
    static_assert(noexcept(std::declval<Release &>()()),"worker cleanup must not throw");
  }
  SolveScopeExit(const SolveScopeExit &)=delete;
  SolveScopeExit &operator=(const SolveScopeExit &)=delete;
  ~SolveScopeExit() noexcept {release_();}
private:
  Release release_;
};
}  // namespace scan_planner
