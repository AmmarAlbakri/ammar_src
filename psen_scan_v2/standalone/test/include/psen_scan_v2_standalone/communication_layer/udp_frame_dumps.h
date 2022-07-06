// Copyright (c) 2020-2022 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_V2_UDP_FRAME_DUMPS_H
#define PSEN_SCAN_V2_UDP_FRAME_DUMPS_H

#include <array>
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>

#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"
#include "psen_scan_v2_standalone/io_state.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
namespace scanner_udp_datagram_hexdumps
{
using namespace data_conversion_layer;
using namespace monitoring_frame;

constexpr uint8_t
clearIntensityChannelBits(const size_t index, const size_t begin, const size_t n, const uint8_t hexdump_byte)
{
  if ((index >= begin) && (index < begin + n) && (0 == index % 2))
  {
    return 0b00111111 & hexdump_byte;
  }
  else
  {
    return hexdump_byte;
  }
}

constexpr uint16_t convertHexdumpBytesToUint16_t(const uint8_t msbyte, const uint8_t lsbyte)
{
  return static_cast<uint16_t>(msbyte << 8) + static_cast<uint16_t>(lsbyte);
}

template <size_t ARRAY_SIZE>
inline std::stringstream readIODataToStream(const std::array<uint8_t, ARRAY_SIZE> hex_dump,
                                            const size_t offset_io_field)
{
  std::array<uint8_t, 22> io_dump;
  std::copy(hex_dump.begin() + offset_io_field + 42, hex_dump.begin() + offset_io_field + 64, io_dump.begin());

  std::stringstream ss;
  ss.write(convertToRawData(io_dump).data(), 22);
  return ss;
}

template <size_t ARRAY_SIZE>
inline io::PinData readIOField(const std::array<uint8_t, ARRAY_SIZE> hex_dump, const size_t offset_io_field)
{
  auto ss = readIODataToStream(hex_dump, offset_io_field);
  io::PinData io_pin_data;

  raw_processing::read<std::array<uint8_t, 4>>(ss);
  io::deserializePinField(ss, io_pin_data.input_state);

  raw_processing::read<std::array<uint8_t, 4>>(ss);
  io::deserializePinField(ss, io_pin_data.output_state);

  return io_pin_data;
}

template <size_t ARRAY_SIZE>
inline std::vector<double> readMeasurements(const std::array<uint8_t, ARRAY_SIZE> hex_dump,
                                            const size_t offset_measurements,
                                            const size_t n_measurements)
{
  std::vector<double> measurements;
  for (size_t idx = offset_measurements; idx < (offset_measurements + (n_measurements * 2)); idx = idx + 2)
  {
    uint16_t raw_value = convertHexdumpBytesToUint16_t(hex_dump.at(idx + 1), hex_dump.at(idx));
    double meter = raw_value / 1000.;
    if (raw_value == NO_SIGNAL_ARRIVED || raw_value == SIGNAL_TOO_LATE)
    {
      meter = std::numeric_limits<double>::infinity();
    }
    measurements.push_back(meter);
  }
  return measurements;
}

template <size_t ARRAY_SIZE>
inline std::vector<double> readIntensities(const std::array<uint8_t, ARRAY_SIZE> hex_dump,
                                           const size_t offset_measurements,
                                           const size_t n_measurements)
{
  std::vector<double> measurements;
  for (size_t idx = offset_measurements; idx < (offset_measurements + (n_measurements * 2)); idx = idx + 2)
  {
    uint16_t raw_value = convertHexdumpBytesToUint16_t(hex_dump.at(idx + 1), hex_dump.at(idx));
    measurements.push_back(raw_value & 0b0011111111111111);
  }
  return measurements;
}

class WithIntensitiesAndDiagnostics
{
public:
  WithIntensitiesAndDiagnostics()
  {
    MessageBuilder msg_builder;
    msg_builder.fromTheta(util::TenthOfDegree(0x3e9))
        .resolution(util::TenthOfDegree(0x02))
        .iOPinData(readIOField(hex_dump, 24))
        .scanCounter(0x00055630)
        .activeZoneset(0x02)
        .measurements(readMeasurements(hex_dump, 143, 250))
        .intensities(readIntensities(hex_dump, intensities_offset, 250))
        .diagnosticMessages({ diagnostic::Message(configuration::ScannerId::master, diagnostic::ErrorLocation(2, 0)),
                              diagnostic::Message(configuration::ScannerId::master, diagnostic::ErrorLocation(4, 3)) });
    expected_msg_ = msg_builder.build();
  };

  const size_t intensities_offset{ 143 + (250 * 2) + 3 };

  // This is a dump read with wireshark and transformed with the script test/scripts/parse_dump.py.
  // clang-format off
  const std::array<uint8_t, 1150> hex_dump = {
                                                            0x00, 0x00, 0x00, 0x00, 0xca, 0x00,  // 0020
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xe9, 0x03, 0x02, 0x00, 0x01,  // 0030
0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0040
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0050
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0060
0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x00, 0x00, 0x00,  // 0070
0x02, 0x05, 0x00, 0x30, 0x56, 0x05, 0x00, 0x03, 0x02, 0x00, 0x02, 0x04, 0x29, 0x00, 0x00, 0x00,  // 0080
0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0090
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 00a0
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xf5, 0x01, 0xe9, 0x06, 0xcc, 0x06, 0xae, 0x06, 0x9b,  // 00b0
0x06, 0xcc, 0x06, 0xdf, 0x06, 0xe1, 0x07, 0xd8, 0x07, 0xb8, 0x06, 0xa5, 0x06, 0xa5, 0x06, 0xb8,  // 00c0
0x06, 0xc2, 0x06, 0xde, 0x07, 0xae, 0x06, 0xea, 0x07, 0xa3, 0x06, 0x9f, 0x07, 0xe7, 0x07, 0x9b,  // 00d0
0x06, 0xa9, 0x05, 0xa2, 0x05, 0x91, 0x06, 0x9e, 0x05, 0x8f, 0x05, 0x9f, 0x05, 0x9d, 0x05, 0x83,  // 00e0
0x05, 0x9c, 0x05, 0xa2, 0x05, 0x7e, 0x05, 0x78, 0x05, 0x61, 0x05, 0x82, 0x05, 0x80, 0x05, 0x6e,  // 00f0
0x05, 0x71, 0x05, 0x6f, 0x05, 0x53, 0x05, 0x65, 0x05, 0x51, 0x05, 0x3e, 0x05, 0x58, 0x05, 0x4f,  // 0100
0x05, 0x4d, 0x05, 0x46, 0x05, 0x41, 0x05, 0x3a, 0x05, 0x2c, 0x05, 0x33, 0x05, 0x0c, 0x05, 0x1c,  // 0110
0x05, 0x2c, 0x05, 0x2a, 0x05, 0x27, 0x05, 0x13, 0x05, 0xba, 0x07, 0xa4, 0x07, 0x72, 0x07, 0xb7,  // 0120
0x07, 0xaf, 0x07, 0xfa, 0x04, 0x00, 0x05, 0xf1, 0x04, 0xf1, 0x04, 0xeb, 0x04, 0xe6, 0x04, 0xd9,  // 0130
0x04, 0xe8, 0x04, 0xe6, 0x04, 0xdc, 0x04, 0xc6, 0x04, 0x84, 0x06, 0x8c, 0x06, 0x7b, 0x06, 0xb7,  // 0140
0x04, 0x4f, 0x06, 0xc9, 0x04, 0xb8, 0x04, 0x23, 0x06, 0x09, 0x06, 0x08, 0x06, 0x19, 0x06, 0xee,  // 0150
0x05, 0xdc, 0x05, 0x10, 0x06, 0x2b, 0x06, 0xf6, 0x05, 0xdc, 0x05, 0xbe, 0x05, 0x97, 0x05, 0xbd,  // 0160
0x05, 0x97, 0x05, 0x24, 0x05, 0xdb, 0x04, 0x22, 0x05, 0x8d, 0x05, 0x46, 0x06, 0x34, 0x06, 0x76,  // 0170
0x05, 0x1b, 0x05, 0x13, 0x05, 0x0b, 0x05, 0xfa, 0x04, 0xc5, 0x05, 0xce, 0x05, 0xac, 0x05, 0x0a,  // 0180
0x03, 0x87, 0x02, 0x3a, 0x02, 0x0f, 0x02, 0x14, 0x02, 0x0c, 0x02, 0x09, 0x02, 0xfc, 0x01, 0x00,  // 0190
0x02, 0x0a, 0x02, 0x0b, 0x02, 0x1c, 0x02, 0xdd, 0x01, 0x66, 0x02, 0xd5, 0x02, 0xe3, 0x03, 0x0c,  // 01a0
0x07, 0x20, 0x07, 0x2c, 0x07, 0xce, 0x05, 0x37, 0x07, 0xf0, 0x05, 0x31, 0x05, 0x4a, 0x05, 0x5b,  // 01b0
0x05, 0x5a, 0x05, 0x6c, 0x05, 0x7f, 0x05, 0x9b, 0x05, 0x9a, 0x05, 0x9a, 0x05, 0xa4, 0x05, 0xc2,  // 01c0
0x05, 0xcd, 0x05, 0x2a, 0x06, 0xf1, 0x05, 0x03, 0x06, 0x14, 0x06, 0x41, 0x06, 0x2f, 0x06, 0x2f,  // 01d0
0x06, 0x38, 0x06, 0x51, 0x06, 0x6c, 0x06, 0xcf, 0x04, 0xcc, 0x04, 0xd9, 0x04, 0xd8, 0x04, 0x66,  // 01e0
0x15, 0x54, 0x04, 0xa8, 0x03, 0xc2, 0x03, 0x4c, 0x03, 0x74, 0x02, 0x54, 0x02, 0x60, 0x02, 0x60,  // 01f0
0x02, 0x68, 0x02, 0x63, 0x02, 0x6c, 0x02, 0x67, 0x02, 0x6e, 0x02, 0x65, 0x02, 0x25, 0x02, 0xdd,  // 0200
0x01, 0xe9, 0x01, 0xe9, 0x01, 0xe9, 0x01, 0xd2, 0x01, 0xc6, 0x01, 0xba, 0x01, 0xd2, 0x01, 0xdd,  // 0210
0x01, 0xe9, 0x01, 0xdf, 0x01, 0xd2, 0x01, 0xf5, 0x01, 0x24, 0x02, 0x51, 0x02, 0x7e, 0x02, 0xea,  // 0220
0x02, 0x34, 0xea, 0x34, 0xea, 0x34, 0xea, 0x46, 0x02, 0xa2, 0x01, 0x6a, 0x01, 0x5e, 0x01, 0x8a,  // 0230
0x01, 0x0c, 0x02, 0x50, 0x02, 0x17, 0x02, 0xd2, 0x01, 0x7f, 0x01, 0x8b, 0x01, 0x46, 0x01, 0x3b,  // 0240
0x01, 0x7f, 0x01, 0x73, 0x01, 0xae, 0x01, 0xba, 0x01, 0xc5, 0x01, 0xb9, 0x01, 0xb9, 0x01, 0xc5,  // 0250
0x01, 0xc5, 0x01, 0xd1, 0x01, 0xc5, 0x01, 0xdd, 0x01, 0xd1, 0x01, 0xdd, 0x01, 0xe7, 0x01, 0xe7,  // 0260
0x01, 0xe7, 0x01, 0xdc, 0x01, 0xe7, 0x01, 0xdc, 0x01, 0xd0, 0x01, 0xb9, 0x01, 0x89, 0x01, 0x73,  // 0270
0x01, 0x80, 0x01, 0x8c, 0x01, 0xa1, 0x01, 0xb8, 0x01, 0x95, 0x01, 0x82, 0x01, 0x5b, 0x01, 0xb8,  // 0280
0x01, 0xf2, 0x01, 0x09, 0x02, 0xfe, 0x01, 0x09, 0x02, 0x14, 0x02, 0x14, 0x02, 0x08, 0x02, 0x08,  // 0290
0x02, 0x08, 0x02, 0x08, 0x02, 0xfd, 0x01, 0x14, 0x02, 0x14, 0x02, 0x14, 0x02, 0x06, 0xf5, 0x01,  // 02a0
0xf0, 0x01, 0x34, 0x02, 0x49, 0x02, 0x63, 0x02, 0x2e, 0x02, 0x4e, 0x02, 0xf1, 0x48, 0xd9, 0x48,  // 02b0
0x65, 0x02, 0x6b, 0x02, 0x4e, 0x02, 0x82, 0x04, 0xdc, 0x05, 0xbe, 0x46, 0xec, 0x05, 0x55, 0x48,  // 02c0
0x7f, 0x08, 0x73, 0x48, 0x9c, 0x46, 0x61, 0x02, 0x93, 0x02, 0xbf, 0x02, 0x6b, 0x02, 0x9d, 0x02,  // 02d0
0xc6, 0x02, 0xc7, 0x02, 0x6f, 0x09, 0x16, 0x09, 0x99, 0x02, 0xa8, 0x02, 0xc3, 0x02, 0xce, 0x02,  // 02e0
0x19, 0x03, 0xe7, 0x02, 0xd5, 0x02, 0xe0, 0x02, 0xe8, 0x02, 0x0e, 0x03, 0xc7, 0x0a, 0x5c, 0x03,  // 02f0
0x48, 0x08, 0xee, 0x07, 0xf5, 0x07, 0x92, 0x08, 0xf9, 0x08, 0xd5, 0x0b, 0x0b, 0x14, 0x4f, 0x14,  // 0300
0x71, 0x0c, 0xe1, 0x09, 0xcf, 0x0a, 0x57, 0x09, 0x33, 0x09, 0x5e, 0x0a, 0x09, 0x0a, 0x25, 0x09,  // 0310
0xf9, 0x44, 0xd8, 0x45, 0x5b, 0x43, 0xbb, 0x42, 0xc6, 0x42, 0x9b, 0x06, 0x8e, 0x06, 0x0d, 0x09,  // 0320
0xcd, 0x09, 0x7b, 0x06, 0xc0, 0x03, 0xc0, 0x03, 0xbd, 0x03, 0xc2, 0x03, 0xc3, 0x03, 0xdc, 0x03,  // 0330
0x2c, 0x41, 0x2c, 0x41, 0x2c, 0x41, 0xe6, 0x03, 0x2c, 0x41, 0xe8, 0x03, 0xef, 0x03, 0x9d, 0x41,  // 0340
0xc0, 0x41, 0xb0, 0x41, 0x93, 0x41, 0xe8, 0x41, 0x1a, 0x42, 0x9d, 0x41, 0x79, 0x41, 0xec, 0x41,  // 0350
0x1a, 0x42, 0x51, 0x42, 0xe2, 0x42, 0x70, 0x42, 0xcb, 0x42, 0x44, 0x45, 0x76, 0x48, 0x68, 0x47,  // 0360
0xf0, 0x42, 0x5c, 0x4b, 0x26, 0x4b, 0x71, 0x47, 0x38, 0x45, 0x6d, 0x45, 0xc7, 0x45, 0x0e, 0x46,  // 0370
0xb8, 0x49, 0x67, 0x49, 0xfd, 0x49, 0xdc, 0x4c, 0xb9, 0x4b, 0x1f, 0x49, 0x8a, 0x86, 0xa5, 0x88,  // 0380
0x9c, 0x8a, 0x39, 0x8c, 0x06, 0x8d, 0x9c, 0x8c, 0x37, 0x8b, 0x51, 0x89, 0x78, 0x87, 0x29, 0x48,  // 0390
0x08, 0x4b, 0xe4, 0x4c, 0xd5, 0x4b, 0xb7, 0x88, 0xa6, 0x89, 0xfa, 0x89, 0xa5, 0x4c, 0x33, 0x8a,  // 03a0
0xa5, 0x4a, 0x9b, 0x45, 0x89, 0x44, 0x14, 0x44, 0xff, 0x43, 0xc6, 0x44, 0xa4, 0x44, 0x73, 0x44,  // 03b0
0x43, 0x43, 0x18, 0x45, 0xf5, 0x42, 0x75, 0x42, 0x7f, 0x44, 0xb2, 0x46, 0xca, 0x44, 0x41, 0x43,  // 03c0
0x9b, 0x41, 0x4a, 0x41, 0x52, 0x41, 0x68, 0x41, 0x59, 0x41, 0x2c, 0x41, 0x2c, 0x41, 0xf0, 0x03,  // 03d0
0xfa, 0x03, 0xcc, 0x03, 0xd9, 0x03, 0xcd, 0x04, 0x52, 0x41, 0x96, 0x42, 0x78, 0x42, 0xfb, 0x42,  // 03e0
0x20, 0x47, 0xac, 0x48, 0x92, 0x48, 0xa6, 0x48, 0x62, 0x49, 0x7e, 0x88, 0x44, 0x87, 0x99, 0x86,  // 03f0
0x37, 0x88, 0x55, 0x88, 0x7a, 0x48, 0x7b, 0x48, 0x75, 0x48, 0x4a, 0x48, 0xa4, 0x48, 0xae, 0x48,  // 0400
0x50, 0x48, 0x61, 0x48, 0xc1, 0x48, 0xa4, 0x48, 0x51, 0x48, 0xa9, 0x47, 0x97, 0x46, 0x0f, 0x45,  // 0410
0x61, 0x43, 0xed, 0x42, 0x79, 0x42, 0x7d, 0x41, 0xfa, 0xff, 0xfa, 0xff, 0xfa, 0xff, 0x92, 0x42,  // 0420
0x3e, 0x45, 0x3c, 0x47, 0x72, 0x47, 0x86, 0x45, 0xe1, 0x42, 0x4f, 0x42, 0xab, 0x42, 0x22, 0x44,  // 0430
0x55, 0x46, 0x60, 0x48, 0x45, 0x48, 0x3c, 0x48, 0x4f, 0x48, 0x39, 0x46, 0xd5, 0x44, 0x7c, 0x44,  // 0440
0x43, 0x44, 0x4e, 0x44, 0x39, 0x44, 0x1e, 0x44, 0xff, 0x43, 0xee, 0x43, 0xdb, 0x43, 0xda, 0x43,  // 0450
0xda, 0x43, 0xb1, 0x43, 0xaf, 0x43, 0xa4, 0x43, 0xa8, 0x43, 0xa8, 0x43, 0x8e, 0x43, 0xbe, 0x43,  // 0460
0x0e, 0x44, 0xd6, 0x44, 0xa3, 0x45, 0x70, 0x46, 0x6a, 0x47, 0x41, 0x47, 0xd4, 0x45, 0xfd, 0x44,  // 0470
0xca, 0x45, 0xeb, 0x46, 0xa3, 0x46, 0x05, 0x45, 0x86, 0x43, 0x33, 0x43, 0x39, 0x43, 0x27, 0x43,  // 0480
0x2a, 0x43, 0x22, 0x43, 0x12, 0x43, 0x1a, 0x43, 0x1e, 0x43, 0x33, 0x43, 0x3e, 0x43, 0x1f, 0x43,  // 0490
                                                0x1c, 0x43, 0x2a, 0x43, 0x09, 0x00, 0x00, 0x00,  // 04a0
  };
  // clang-format on

  data_conversion_layer::monitoring_frame::Message expected_msg_;
};

class WithoutMeasurementsAndIntensities
{
public:
  WithoutMeasurementsAndIntensities()
  {
    MessageBuilder msg_builder;
    msg_builder.fromTheta(util::TenthOfDegree(0x5dc))
        .resolution(util::TenthOfDegree(0x0a))
        .scanCounter(0x0661fc)
        .activeZoneset(0x02)
        .measurements({});
    expected_msg_ = msg_builder.build();
  }

  const std::array<uint8_t, 39> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x03, 0x02, 0x00, 0x02,                                      // Active zoneset
    0x05, 0x01, 0x00,                                            // Measurements
    0x09, 0x00, 0x00, 0x00                                       // End of Frame
  };

  data_conversion_layer::monitoring_frame::Message expected_msg_;
};

class WithUnknownFieldId
{
public:
  const std::array<uint8_t, 35> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x0a, 0x01, 0x00,                                            // Measurements
    0x09, 0x00, 0x00, 0x00                                       // End of Frame
  };
};

class WithTooLargeFieldLength
{
public:
  const std::array<uint8_t, 35> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x05, 0xcf, 0xff,                                            // Measurements
    0x09, 0x00, 0x00, 0x00                                       // End of Frame
  };
};

class WithTooLargeScanCounterLength
{
private:
  unsigned char modv{ 0x06 };

public:
  const std::array<uint8_t, 35> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x02, modv, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x05, 0x00, 0x00,                                            // Measurements
    0x09, 0x00, 0x00, 0x00                                       // End of Frame
  };
};

class WithTooLargeActiveZoneSetLength
{
private:
  unsigned char modv{ 0x03 };

public:
  const std::array<uint8_t, 39> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x03, modv, 0x00, 0x00,                                      // Active zoneset
    0x05, 0x00, 0x00,                                            // Measurements
    0x09, 0x00, 0x00, 0x00                                       // End of Frame
  };
};

class WithTooLargeIntensityLength
{
public:
  const std::array<uint8_t, 38> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x06, 0x00, 0x04,                                            // Intensities
    0x05, 0x00, 0x00,                                            // Measurements
    0x09, 0x00, 0x00, 0x00                                       // End of Frame
  };
};

class WithTooSmallIOStateFieldLength
{
private:
  unsigned char modv{ 0x03 };

public:
  const std::array<uint8_t, 105> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x01, modv, 0x00, 0x00,                                      // IO states
    0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x05, 0x00, 0x00,                                            // Measurements
    0x09, 0x00, 0x00, 0x00                                       // End of Frame
  };
};

class WithMissingIOStateField
{
public:
  const std::array<uint8_t, 105> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x05, 0x00, 0x00,                                            // Measurements
    0x09, 0x00, 0x00, 0x00                                       // End of Frame
  };
};

class WithNoEnd
{
private:
  unsigned char modv{ 0x06 };

public:
  const std::array<uint8_t, 35> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00,  // End of FixedFields
    0x02, modv, 0x00, 0xfc, 0x61, 0x06, 0x00,                    // Scan counter
    0x05, 0x00, 0x00,                                            // Measurements
  };
};
}  // namespace scanner_udp_datagram_hexdumps
}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_UDP_FRAME_DUMPS_H