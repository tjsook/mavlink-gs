#pragma once

#include <mutex>
#include <utility>
#include <vector>

#include <common/mavlink.h>

#include "net/waker.hpp"

namespace mgs {

// A thread-safe hand-off of work items from the UI thread (which turns
// keystrokes into requests) to the network thread (which owns the socket and
// acts on them).
//
// The UI thread calls push(), which also rings the Waker so the network thread
// leaves poll() at once; the network thread calls drain() and handles whatever
// it took. The lock is held only to move items in or out — never across the
// socket send — so neither thread waits on the other's I/O.
//
// A template because the UI hands over more than one kind of work: single
// ready-built messages (CommandQueue), and whole missions to upload. Several
// queues can share one Waker; any push wakes the same poll().
template <typename T>
class HandoffQueue {
 public:
  explicit HandoffQueue(Waker& wake) : wake_(wake) {}

  // Enqueue an item and wake the network thread (UI thread).
  void push(T item) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.push_back(std::move(item));
    }
    // Outside the lock: the woken thread's first act is to drain(), which takes
    // this mutex. Notifying while holding it would wake it straight into a wait.
    wake_.notify();
  }

  // Take everything queued so far, leaving the queue empty (network thread).
  std::vector<T> drain() {
    std::vector<T> taken;
    std::lock_guard<std::mutex> lock(mutex_);
    std::swap(taken, pending_);
    return taken;
  }

 private:
  Waker& wake_;
  std::mutex mutex_;
  std::vector<T> pending_;
};

// Built MAVLink messages for the network thread to send as-is.
using CommandQueue = HandoffQueue<mavlink_message_t>;

}  // namespace mgs
