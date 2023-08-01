#pragma once

#include <ViconDataStreamSDK_CPP/DataStreamClient.h>

namespace vicon_bridge {

using namespace ViconDataStreamSDK::CPP;

std::string Adapt(const Direction::Enum i_Direction) {
  switch (i_Direction) {
  case Direction::Forward:
    return "Forward";
  case Direction::Backward:
    return "Backward";
  case Direction::Left:
    return "Left";
  case Direction::Right:
    return "Right";
  case Direction::Up:
    return "Up";
  case Direction::Down:
    return "Down";
  default:
    return "Unknown";
  }
}

std::string Adapt(const Result::Enum i_result) {
  switch (i_result) {
  case Result::ClientAlreadyConnected:
    return "ClientAlreadyConnected";
  case Result::ClientConnectionFailed:
    return "ClientConnectionFailed";
  case Result::CoLinearAxes:
    return "CoLinearAxes";
  case Result::InvalidDeviceName:
    return "InvalidDeviceName";
  case Result::InvalidDeviceOutputName:
    return "InvalidDeviceOutputName";
  case Result::InvalidHostName:
    return "InvalidHostName";
  case Result::InvalidIndex:
    return "InvalidIndex";
  case Result::InvalidLatencySampleName:
    return "InvalidLatencySampleName";
  case Result::InvalidMarkerName:
    return "InvalidMarkerName";
  case Result::InvalidMulticastIP:
    return "InvalidMulticastIP";
  case Result::InvalidSegmentName:
    return "InvalidSegmentName";
  case Result::InvalidSubjectName:
    return "InvalidSubjectName";
  case Result::LeftHandedAxes:
    return "LeftHandedAxes";
  case Result::NoFrame:
    return "NoFrame";
  case Result::NotConnected:
    return "NotConnected";
  case Result::NotImplemented:
    return "NotImplemented";
  case Result::ServerAlreadyTransmittingMulticast:
    return "ServerAlreadyTransmittingMulticast";
  case Result::ServerNotTransmittingMulticast:
    return "ServerNotTransmittingMulticast";
  case Result::Success:
    return "Success";
  case Result::Unknown:
    return "Unknown";
  default:
    return "unknown, couldn't parse";
  }
}

} // namespace vicon_bridge
