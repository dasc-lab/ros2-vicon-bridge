
//////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2017 Vicon Motion Systems Ltd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//////////////////////////////////////////////////////////////////////////////////

#include <ViconDataStreamSDK_CPP/DataStreamClient.h>

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifdef WIN32
#include <conio.h>   // For _kbhit()
#include <cstdio>    // For getchar()
#include <windows.h> // For Sleep()
#else
#include <unistd.h> // For sleep()
#endif              // WIN32

#include <string.h>
#include <time.h>

using namespace ViconDataStreamSDK::CPP;

namespace {
std::string Adapt(const bool i_Value) { return i_Value ? "True" : "False"; }

std::string Adapt(const TimecodeStandard::Enum i_Standard) {
  switch (i_Standard) {
  default:
  case TimecodeStandard::None:
    return "0";
  case TimecodeStandard::PAL:
    return "1";
  case TimecodeStandard::NTSC:
    return "2";
  case TimecodeStandard::NTSCDrop:
    return "3";
  case TimecodeStandard::Film:
    return "4";
  case TimecodeStandard::NTSCFilm:
    return "5";
  case TimecodeStandard::ATSC:
    return "6";
  }
}

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

std::string Adapt(const DeviceType::Enum i_DeviceType) {
  switch (i_DeviceType) {
  case DeviceType::ForcePlate:
    return "ForcePlate";
  case DeviceType::Unknown:
  default:
    return "Unknown";
  }
}

std::string Adapt(const Unit::Enum i_Unit) {
  switch (i_Unit) {
  case Unit::Meter:
    return "Meter";
  case Unit::Volt:
    return "Volt";
  case Unit::NewtonMeter:
    return "NewtonMeter";
  case Unit::Newton:
    return "Newton";
  case Unit::Kilogram:
    return "Kilogram";
  case Unit::Second:
    return "Second";
  case Unit::Ampere:
    return "Ampere";
  case Unit::Kelvin:
    return "Kelvin";
  case Unit::Mole:
    return "Mole";
  case Unit::Candela:
    return "Candela";
  case Unit::Radian:
    return "Radian";
  case Unit::Steradian:
    return "Steradian";
  case Unit::MeterSquared:
    return "MeterSquared";
  case Unit::MeterCubed:
    return "MeterCubed";
  case Unit::MeterPerSecond:
    return "MeterPerSecond";
  case Unit::MeterPerSecondSquared:
    return "MeterPerSecondSquared";
  case Unit::RadianPerSecond:
    return "RadianPerSecond";
  case Unit::RadianPerSecondSquared:
    return "RadianPerSecondSquared";
  case Unit::Hertz:
    return "Hertz";
  case Unit::Joule:
    return "Joule";
  case Unit::Watt:
    return "Watt";
  case Unit::Pascal:
    return "Pascal";
  case Unit::Lumen:
    return "Lumen";
  case Unit::Lux:
    return "Lux";
  case Unit::Coulomb:
    return "Coulomb";
  case Unit::Ohm:
    return "Ohm";
  case Unit::Farad:
    return "Farad";
  case Unit::Weber:
    return "Weber";
  case Unit::Tesla:
    return "Tesla";
  case Unit::Henry:
    return "Henry";
  case Unit::Siemens:
    return "Siemens";
  case Unit::Becquerel:
    return "Becquerel";
  case Unit::Gray:
    return "Gray";
  case Unit::Sievert:
    return "Sievert";
  case Unit::Katal:
    return "Katal";

  case Unit::Unknown:
  default:
    return "Unknown";
  }
}
#ifdef WIN32
bool Hit() {
  bool hit = false;
  while (_kbhit()) {
    getchar();
    hit = true;
  }
  return hit;
}
#endif

class NullBuffer : public std::streambuf {
public:
  int overflow(int c) { return c; }
};

NullBuffer Null;
std::ostream NullStream(&Null);

} // namespace

int main(int argc, char *argv[]) {
  // Program options

  std::vector<std::string> Hosts;
  int Arg = 1;
  for (Arg; Arg < argc; ++Arg) {
    if (strncmp(argv[Arg], "--", 2) == 0) {
      break;
    }
    Hosts.push_back(argv[Arg]);
  }

  if (Hosts.empty()) {
    Hosts.push_back("localhost:801");
  }

  // log contains:
  // version number
  // log of framerate over time
  // --multicast
  // kill off internal app
  std::string LogFile = "";
  std::string MulticastAddress = "239.0.0.0:44801";
  std::string MulticastAdapter;
  bool ConnectToMultiCast = false;
  bool EnableMultiCast = false;
  bool EnableHapticTest = false;
  bool bReadCentroids = false;
  bool bReadRayData = false;
  bool bReadGreyscaleData = false;
  bool bReadVideoData = false;
  bool bMarkerTrajIds = false;
  bool bLightweight = false;
  bool bSegmentsOnly = false;
  bool bFetch = false;
  bool bPreFetch = false;
  bool bQuiet = false;
  bool bUnlabelled = false;
  bool bOptimizeWireless = false;

  std::vector<std::string> HapticOnList(0);
  unsigned int ClientBufferSize = 0;
  std::string AxisMapping = "ZUp";
  std::vector<std::string> FilteredSubjects;
  std::vector<std::string> LocalAdapters;

  for (int a = Arg; a < argc; ++a) {
    std::string arg = argv[a];
    if (arg == "--help") {
      std::cout << argv[0]
                << " <HostName>: allowed options include:" << std::endl;
      std::cout << " --log_file <FileName>" << std::endl;
      std::cout << " --enable_multicast <MulticastAddress:Port>" << std::endl;
      std::cout
          << " --connect_to_multicast <MulticastAddress:Port> <LocalAdapter>"
          << std::endl;
      std::cout << " --help" << std::endl;
      std::cout << " --enable_haptic_test <DeviceName>" << std::endl;
      std::cout << " --centroids" << std::endl;
      std::cout << " --client-buffer-size <size>" << std::endl;
      std::cout << " --quiet" << std::endl;
      std::cout << " --fetch" << std::endl;
      std::cout << " --pre-fetch" << std::endl;
      std::cout << " --stream" << std::endl;
      std::cout << " --optimize-wireless" << std::endl;

      return 0;
    } else if (arg == "--log_file" || arg == "--log-file") {
      if (a + 1 < argc) {
        LogFile = argv[a + 1];
        std::cout << "Using log file <" << LogFile << "> ..." << std::endl;
        ++a;
      }
    } else if (arg == "--enable_multicast" || arg == "--enable-multicast") {
      if (a + 1 < argc) {
        MulticastAddress = argv[a + 1];
        std::cout << "Enabling multicast address <" << MulticastAddress
                  << "> ..." << std::endl;
        ++a;
      }
    } else if (arg == "--connect_to_multicast" ||
               arg == "--connect-to-multicast") {
      ConnectToMultiCast = true;
      if (a + 2 < argc) {
        MulticastAddress = argv[a + 1];
        MulticastAdapter = argv[a + 2];
        std::cout << "connecting to multicast address <" << MulticastAddress
                  << "> on adapter <" << MulticastAdapter << "> ..."
                  << std::endl;
        a += 2;
      }
    } else if (arg == "--enable_haptic_test" || arg == "--enable-haptic-test") {
      EnableHapticTest = true;
      ++a;
      if (a < argc) {
        // assuming no haptic device name starts with "--"
        while (a < argc && strncmp(argv[a], "--", 2) != 0) {
          HapticOnList.push_back(argv[a]);
          ++a;
        }
      }
    } else if (arg == "--centroids") {
      bReadCentroids = true;
    } else if (arg == "--rays") {
      bReadRayData = true;
    } else if (arg == "--greyscale") {
      bReadGreyscaleData = true;
      bReadCentroids = true;
      std::cout << "Enabling greyscale data also enables centroid output"
                << std::endl;
    } else if (arg == "--video") {
      bReadVideoData = true;
      bReadCentroids = true;
      std::cout << "Enabling video data also enables centroid output"
                << std::endl;
    } else if (arg == "--marker-traj-id") {
      bMarkerTrajIds = true;
    } else if (arg == "--client-buffer-size") {
      ++a;
      if (a < argc) {
        ClientBufferSize = atoi(argv[a]);
      }
    } else if (arg == "--set-axis-mapping") {
      ++a;
      if (a < argc) {
        AxisMapping = argv[a];

        if (AxisMapping == "XUp" || AxisMapping == "YUp" ||
            AxisMapping == "ZUp") {
          std::cout << "Setting Axis to " << AxisMapping << std::endl;
        } else {
          std::cout << "Unknown axis setting: " << AxisMapping
                    << " . Should be XUp, YUp, or ZUp" << std::endl;
          return 1;
        }
      }
    } else if (arg == "--segments-only") {
      bSegmentsOnly = true;
    } else if (arg == "--lightweight") {
      bLightweight = true;
    } else if (arg == "--unlabeled") {
      bUnlabelled = true;
    } else if (arg == "--subjects") {
      ++a;
      // assuming no subject name starts with "--"
      while (a < argc) {
        if (strncmp(argv[a], "--", 2) == 0) {
          --a;
          break;
        }
        FilteredSubjects.push_back(argv[a]);
        ++a;
      }
    } else if (arg == "--fetch") {
      bFetch = true;
      bPreFetch = false;
    } else if (arg == "--pre-fetch") {
      bFetch = false;
      bPreFetch = true;
    } else if (arg == "--stream") {
      bFetch = false;
      bPreFetch = false;
    } else if (arg == "--quiet") {
      bQuiet = true;
    } else if (arg == "--optimize-wireless") {
      bOptimizeWireless = true;
    } else {
      std::cout << "Failed to understand argument <" << argv[a] << ">...exiting"
                << std::endl;
      return 1;
    }
  }

  std::ostream &OutputStream(bQuiet ? NullStream : std::cout);

  ViconDataStreamSDK::CPP::Client MulticastClient;

  if (ConnectToMultiCast) {
    // Connect to a server
    std::cout << "Connecting to multicast group " << MulticastAddress
              << " on adapter " << MulticastAdapter << " ..." << std::flush;

    // Multicast connection
    const bool Ok =
        (MulticastClient.ConnectToMulticast(MulticastAdapter, MulticastAddress)
             .Result == Result::Success);

    if (!Ok) {
      std::cout << "Warning - connect failed..." << std::endl;
    }
  }

  bool First = true;
  std::string HostName;
  for (const auto &rHost : Hosts) {
    if (!First) {
      HostName += ";";
    }
    HostName += rHost;
    First = false;
  }

  // Make a new client
  ViconDataStreamSDK::CPP::Client DirectClient;

  if (bOptimizeWireless) {
    const Output_ConfigureWireless ConfigureWirelessResult =
        DirectClient.ConfigureWireless();

    if (ConfigureWirelessResult.Result != Result::Success) {
      std::cout << "Wireless Config: " << ConfigureWirelessResult.Error
                << std::endl;
    }
  }

  const bool bConnectDirect = !ConnectToMultiCast || EnableMultiCast;

  if (bConnectDirect) {
    // Connect to a server
    std::cout << "Connecting to " << HostName << " ..." << std::flush;
    while (!DirectClient.IsConnected().Connected) {
      // Direct connection

      const Output_Connect ConnectResult = DirectClient.Connect(HostName);
      const bool ok = (ConnectResult.Result == Result::Success);

      if (!ok) {
        std::cout << "Warning - connect failed... ";
        switch (ConnectResult.Result) {
        case Result::ClientAlreadyConnected:
          std::cout << "Client Already Connected" << std::endl;
          break;
        case Result::InvalidHostName:
          std::cout << "Invalid Host Name" << std::endl;
          break;
        case Result::ClientConnectionFailed:
          std::cout << "Client Connection Failed" << std::endl;
          break;
        default:
          std::cout << "Unrecognized Error: " << ConnectResult.Result
                    << std::endl;
          break;
        }
      }

      std::cout << ".";
#ifdef WIN32
      Sleep(1000);
#else
      sleep(1);
#endif
    }

    std::cout << std::endl;

    std::cout << std::endl;
    // Enable some different data types
    DirectClient.EnableSegmentData();

    if (!bSegmentsOnly) {
      DirectClient.EnableMarkerData();
      DirectClient.EnableUnlabeledMarkerData();
      DirectClient.EnableMarkerRayData();
      DirectClient.EnableDeviceData();
      DirectClient.EnableDebugData();
    }
    if (bReadCentroids) {
      DirectClient.EnableCentroidData();
    }
    if (bReadRayData) {
      DirectClient.EnableMarkerRayData();
    }
    if (bReadGreyscaleData) {
      DirectClient.EnableGreyscaleData();
    }
    if (bReadVideoData) {
      DirectClient.EnableVideoData();
    }
    if (bLightweight) {
      if (DirectClient.EnableLightweightSegmentData().Result !=
          Result::Success) {
        std::cout << "Server does not support lightweight segment data"
                  << std::endl;
      }
    }
    if (bUnlabelled) {
      // This must be done after lightweight is enabled, as the call the
      // lightweight will disable the data
      DirectClient.EnableUnlabeledMarkerData();
    }

    std::cout << "Segment Data Enabled: "
              << Adapt(DirectClient.IsSegmentDataEnabled().Enabled)
              << std::endl;
    std::cout << "Lightweight Segment Data Enabled: "
              << Adapt(DirectClient.IsLightweightSegmentDataEnabled().Enabled)
              << std::endl;
    std::cout << "Marker Data Enabled: "
              << Adapt(DirectClient.IsMarkerDataEnabled().Enabled) << std::endl;
    std::cout << "Unlabeled Marker Data Enabled: "
              << Adapt(DirectClient.IsUnlabeledMarkerDataEnabled().Enabled)
              << std::endl;
    std::cout << "Device Data Enabled: "
              << Adapt(DirectClient.IsDeviceDataEnabled().Enabled) << std::endl;
    std::cout << "Centroid Data Enabled: "
              << Adapt(DirectClient.IsCentroidDataEnabled().Enabled)
              << std::endl;
    std::cout << "Marker Ray Data Enabled: "
              << Adapt(DirectClient.IsMarkerRayDataEnabled().Enabled)
              << std::endl;
    std::cout << "Centroid Data Enabled: "
              << Adapt(DirectClient.IsCentroidDataEnabled().Enabled)
              << std::endl;
    std::cout << "Greyscale Data Enabled: "
              << Adapt(DirectClient.IsGreyscaleDataEnabled().Enabled)
              << std::endl;
    std::cout << "Video Data Enabled: "
              << Adapt(DirectClient.IsVideoDataEnabled().Enabled) << std::endl;
    std::cout << "Debug Data Enabled: "
              << Adapt(DirectClient.IsDebugDataEnabled().Enabled) << std::endl;

    // Set the streaming mode
    if (bFetch) {
      DirectClient.SetStreamMode(
          ViconDataStreamSDK::CPP::StreamMode::ClientPull);
    } else if (bPreFetch) {
      DirectClient.SetStreamMode(
          ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch);
    } else {
      DirectClient.SetStreamMode(
          ViconDataStreamSDK::CPP::StreamMode::ServerPush);
    }

    // Set the global up axis
    DirectClient.SetAxisMapping(Direction::Forward, Direction::Left,
                                Direction::Up); // Z-up

    if (AxisMapping == "YUp") {
      DirectClient.SetAxisMapping(Direction::Forward, Direction::Up,
                                  Direction::Right); // Y-up
    } else if (AxisMapping == "XUp") {
      DirectClient.SetAxisMapping(Direction::Up, Direction::Forward,
                                  Direction::Left); // Y-up
    }

    Output_GetAxisMapping _Output_GetAxisMapping =
        DirectClient.GetAxisMapping();
    std::cout << "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis)
              << " Y-" << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
              << Adapt(_Output_GetAxisMapping.ZAxis) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = DirectClient.GetVersion();
    std::cout << "Version: " << _Output_GetVersion.Major << "."
              << _Output_GetVersion.Minor << "." << _Output_GetVersion.Point
              << "." << _Output_GetVersion.Revision << std::endl;

    if (ClientBufferSize > 0) {
      DirectClient.SetBufferSize(ClientBufferSize);
      std::cout << "Setting client buffer size to " << ClientBufferSize
                << std::endl;
    }

    if (EnableMultiCast) {
      if (Hosts.size() != 1) {
        std::cout << "Multicast only requires one hostname." << std::endl;
        return 1;
      }

      assert(DirectClient.IsConnected().Connected);
      DirectClient.StartTransmittingMulticast(Hosts.front(), MulticastAddress);
    }

    if (!LogFile.empty()) {
      std::size_t Pos = LogFile.find_last_of('.');
      std::string ClientLogFile = LogFile;
      std::string StreamLogFile = LogFile;
      ClientLogFile.insert(Pos, "-Client");
      StreamLogFile.insert(Pos, "-Stream");
      Output_SetTimingLogFile SetTimingLogFileOutput =
          DirectClient.SetTimingLogFile(ClientLogFile, StreamLogFile);
      if (SetTimingLogFileOutput.Result != Result::Success) {
        std::cout << "Unable to make log files: " << ClientLogFile << " and "
                  << StreamLogFile << std::endl;
        return 1;
      }
    }
  }

  bool bSubjectFilterApplied = false;

  {
    ViconDataStreamSDK::CPP::Client &MyClient(
        ConnectToMultiCast ? MulticastClient : DirectClient);

    size_t Counter = 0;
    const std::chrono::high_resolution_clock::time_point StartTime =
        std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point LastFrameTime =
        std::chrono::high_resolution_clock::now();
    // Loop until a key is pressed
#ifdef WIN32
    while (!Hit())
#else
    while (true)
#endif
    {
      // Get a frame
      while (MyClient.GetFrame().Result != Result::Success) {
        // Sleep a little so that we don't lumber the CPU with a busy poll
        sleep(1);
        OutputStream << ".";
      }

      const std::chrono::high_resolution_clock::time_point Now =
          std::chrono::high_resolution_clock::now();

      double dt_ms =
          (1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(
                      Now - LastFrameTime)
                      .count());
      std::cout << "time since last frame: " << dt_ms << " ms" << std::endl;

      if (dt_ms < 5.0 || dt_ms > 12.0) {
        std::cout << "\033[31m"
                  << "FRAME_TIME : " << dt_ms << "\033[0m" << std::endl;
      }

      LastFrameTime = Now;

      // // Get the frame number
      // Output_GetFrameNumber _Output_GetFrameNumber =
      // MyClient.GetFrameNumber(); OutputStream << "Frame Number: " <<
      // _Output_GetFrameNumber.FrameNumber
      //              << std::endl;

      // Output_GetFrameRate Rate = MyClient.GetFrameRate();
      // OutputStream << "Frame rate: " << Rate.FrameRateHz << std::endl;

      // // Show frame rates
      // for (unsigned int FramerateIndex = 0;
      //      FramerateIndex < MyClient.GetFrameRateCount().Count;
      //      ++FramerateIndex) {
      //   std::string FramerateName =
      //       MyClient.GetFrameRateName(FramerateIndex).Name;
      //   double FramerateValue =
      //   MyClient.GetFrameRateValue(FramerateName).Value;

      //   OutputStream << FramerateName << ": " << FramerateValue << "Hz"
      //                << std::endl;
      // }
      // OutputStream << std::endl;
      //
      // // Get the latency
      // OutputStream << "Latency: " << MyClient.GetLatencyTotal().Total << "s"
      //              << std::endl;

      // for (unsigned int LatencySampleIndex = 0;
      //      LatencySampleIndex < MyClient.GetLatencySampleCount().Count;
      //      ++LatencySampleIndex) {
      //   std::string SampleName =
      //       MyClient.GetLatencySampleName(LatencySampleIndex).Name;
      //   double SampleValue =
      //   MyClient.GetLatencySampleValue(SampleName).Value;

      //   OutputStream << "  " << SampleName << " " << SampleValue << "s"
      //                << std::endl;
      // }
      // OutputStream << std::endl;

      // Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber =
      //     MyClient.GetHardwareFrameNumber();
      // OutputStream << "Hardware Frame Number: "
      //              << _Output_GetHardwareFrameNumber.HardwareFrameNumber
      //              << std::endl;

      // // Count the number of subjects
      // unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      // OutputStream << "Subjects (" << SubjectCount << "):" << std::endl;

      ++Counter;
    }

    if (EnableMultiCast) {
      MyClient.StopTransmittingMulticast();
    }
    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    if (bReadCentroids) {
      MyClient.DisableCentroidData();
    }
    if (bReadRayData) {
      MyClient.DisableMarkerRayData();
    }
    if (bReadGreyscaleData) {
      MyClient.DisableGreyscaleData();
    }
    if (bReadVideoData) {
      MyClient.DisableVideoData();
    }

    // Disconnect and dispose
    int t = clock();
    std::cout << " Disconnecting..." << std::endl;
    MyClient.Disconnect();
    int dt = clock() - t;
    double secs = (double)(dt) / (double)CLOCKS_PER_SEC;
    std::cout << " Disconnect time = " << secs << " secs" << std::endl;
  }
}
