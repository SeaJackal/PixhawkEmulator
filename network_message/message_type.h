#pragma once

enum class MessageType {
  kHeartbeat = 0,
  kAttitude = 1,
  kGlobalPosition = 2,
  kBatteryStatus = 3,
  kCommand = 4,
  kControl = 5
};
