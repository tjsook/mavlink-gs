#include "mavlink/command_tracker.hpp"

namespace mgs {

void CommandTracker::begin(std::uint16_t command,
                           std::chrono::steady_clock::time_point now) {
  status_ = CommandStatus::Pending;
  command_ = command;
  attempts_ = 1;  // the caller already sent it once
  acknowledged_ = false;
  last_sent_ = now;
}

void CommandTracker::on_ack(const mavlink_command_ack_t& ack) {
  if (status_ != CommandStatus::Pending) return;  // nothing awaiting an ack
  if (ack.command != command_) return;            // ack for some other command

  acknowledged_ = true;  // the vehicle heard us; stop retransmitting either way
  switch (ack.result) {
    case MAV_RESULT_ACCEPTED:
      status_ = CommandStatus::Accepted;
      break;
    case MAV_RESULT_IN_PROGRESS:
      status_ = CommandStatus::Pending;  // received, still working on it
      break;
    default:  // TEMPORARILY_REJECTED, DENIED, UNSUPPORTED, FAILED, ...
      status_ = CommandStatus::Rejected;
      break;
  }
}

bool CommandTracker::should_resend(std::chrono::steady_clock::time_point now) {
  if (status_ != CommandStatus::Pending || acknowledged_) return false;
  if (now - last_sent_ < kRetryInterval) return false;
  if (attempts_ >= kMaxAttempts) {
    status_ = CommandStatus::TimedOut;
    return false;
  }
  ++attempts_;
  last_sent_ = now;
  return true;
}

}  // namespace mgs
