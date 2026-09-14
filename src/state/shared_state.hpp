#pragma once

#include <mutex>
#include <utility>

#include "state/vehicle_state.hpp"

namespace mgs {

// A VehicleState shared between the network thread (writer) and the UI thread
// (reader), guarded by one mutex.
//
// The raw state is never exposed. Callers get exactly two operations:
//   modify(fn)  — run a mutation under the lock (network thread)
//   snapshot()  — copy the whole state out under the lock (UI thread)
// Because the lock is held for the duration of each call and released before it
// returns, there is no way to read or write the state without holding it. That
// makes the classic "forgot to lock" data race impossible by construction.
class SharedState {
 public:
  // Apply a mutation to the guarded state while holding the lock. `fn` receives
  // a mutable reference; it must not keep that reference past the call.
  template <typename Fn>
  void modify(Fn&& fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::forward<Fn>(fn)(state_);
  }

  // Return an independent copy of the state, taken atomically under the lock.
  // The UI thread renders from this copy, so a mid-update writer can never be
  // observed as a half-old, half-new snapshot.
  VehicleState snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

 private:
  mutable std::mutex mutex_;  // mutable: snapshot() is const but still locks
  VehicleState state_;
};

}  // namespace mgs
