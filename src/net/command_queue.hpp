#pragma once

#include <mutex>
#include <utility>
#include <vector>

#include <common/mavlink.h>

#include "net/waker.hpp"

namespace mgs {

// A thread-safe hand-off of built MAVLink messages from the UI thread (which
// turns keystrokes into commands) to the network thread (which owns the socket
// and sends them).
//
// The UI thread calls push(), which also rings the Waker so the network thread
// leaves poll() at once; the network thread calls drain() and sends whatever it
// took. The lock is held only to move messages in or out — never across the
// socket send — so neither thread waits on the other's I/O.
class CommandQueue {
 public:
  explicit CommandQueue(Waker& wake) : wake_(wake) {}

  // Enqueue a ready-to-send message and wake the network thread (UI thread).
  void push(const mavlink_message_t& msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.push_back(msg);
    }
    // Outside the lock: the woken thread's first act is to drain(), which takes
    // this mutex. Notifying while holding it would wake it straight into a wait.
    wake_.notify();
  }

  // Take everything queued so far, leaving the queue empty (network thread).
  std::vector<mavlink_message_t> drain() {
    std::vector<mavlink_message_t> taken;
    std::lock_guard<std::mutex> lock(mutex_);
    std::swap(taken, pending_);
    return taken;
  }

 private:
  Waker& wake_;
  std::mutex mutex_;
  std::vector<mavlink_message_t> pending_;
};

}  // namespace mgs
