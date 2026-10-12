#pragma once

#include <mutex>
#include <utility>
#include <vector>

#include <common/mavlink.h>

namespace mgs {

// A thread-safe hand-off of built MAVLink messages from the UI thread (which
// turns keystrokes into commands) to the network thread (which owns the socket
// and sends them).
//
// The UI thread calls push(); the network thread calls drain() once per loop
// and sends whatever it took. The lock is held only to move messages in or out
// — never across the socket send — so neither thread waits on the other's I/O.
class CommandQueue {
 public:
  // Enqueue a ready-to-send message (UI thread).
  void push(const mavlink_message_t& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.push_back(msg);
  }

  // Take everything queued so far, leaving the queue empty (network thread).
  std::vector<mavlink_message_t> drain() {
    std::vector<mavlink_message_t> taken;
    std::lock_guard<std::mutex> lock(mutex_);
    std::swap(taken, pending_);
    return taken;
  }

 private:
  std::mutex mutex_;
  std::vector<mavlink_message_t> pending_;
};

}  // namespace mgs
