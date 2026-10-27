#pragma once

namespace mgs {

// A self-pipe: lets one thread knock another thread out of poll().
//
// The network thread sleeps in poll() on the UDP socket. A condition variable
// cannot wake it, because it is blocked in the kernel, not in cv.wait(). So we
// give poll() a second file descriptor to watch: the read end of a pipe. Any
// thread calls notify() to write a byte into the pipe; poll() sees the read end
// become readable and returns. The woken thread calls clear() to empty the pipe
// so the next poll() sleeps again.
//
// Both ends are non-blocking: notify() never stalls the UI thread, and clear()
// never stalls the network thread.
class Waker {
 public:
  // Opens the pipe. Throws std::system_error if the OS refuses.
  Waker();
  ~Waker();

  // Owns two file descriptors; copying would close them twice.
  Waker(const Waker&) = delete;
  Waker& operator=(const Waker&) = delete;

  // Wake the thread sleeping in poll() on fd(). Safe from any thread.
  void notify() noexcept;

  // Empty the pipe, so fd() stops reading as "ready". Call it on the woken
  // thread, before it checks for the work that notify() announced.
  void clear() noexcept;

  // The read end, for poll() to watch.
  int fd() const noexcept { return read_fd_; }

 private:
  int read_fd_ = -1;
  int write_fd_ = -1;
};

}  // namespace mgs
