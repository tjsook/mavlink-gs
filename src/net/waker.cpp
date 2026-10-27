#include "net/waker.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <system_error>

namespace mgs {

Waker::Waker() {
  int fds[2];
  if (::pipe(fds) != 0) {
    throw std::system_error(errno, std::generic_category(), "pipe");
  }
  read_fd_ = fds[0];
  write_fd_ = fds[1];
  for (const int fd : fds) {
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
    ::fcntl(fd, F_SETFD, FD_CLOEXEC);  // don't leak into child processes
  }
}

Waker::~Waker() {
  ::close(read_fd_);
  ::close(write_fd_);
}

void Waker::notify() noexcept {
  // One byte is enough: poll() only asks "is anything there?". If the pipe is
  // full, write() fails with EAGAIN — and that is fine, because a full pipe is
  // already readable, so the sleeper will wake anyway. Nothing to handle.
  const char byte = 1;
  [[maybe_unused]] const auto n = ::write(write_fd_, &byte, 1);
}

void Waker::clear() noexcept {
  // poll() is level-triggered: one leftover byte and it returns at once forever.
  // So read until the pipe is empty (EAGAIN), retrying only on a signal.
  char buf[256];
  for (;;) {
    const auto n = ::read(read_fd_, buf, sizeof(buf));
    if (n > 0) continue;               // got some; there may be more
    if (n < 0 && errno == EINTR) continue;
    return;                            // EAGAIN: empty (or an error we can't fix)
  }
}

}  // namespace mgs
