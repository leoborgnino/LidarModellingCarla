// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/rpc/Location.h"

#include <cstdint>
#include <vector>

namespace carla {
namespace sensor {
namespace s11n {

  /// Helper class to store and serialize the data generated by a Lidar.
  ///
  /// The header of a Lidar measurement consists of an array of uint32_t's in
  /// the following layout
  ///
  ///    {
  ///      Horizontal angle (float),
  ///      Channel count,
  ///      Point count of channel 0,
  ///      ...
  ///      Point count of channel n,
  ///    }
  ///
  /// The points are stored in an array of floats
  ///
  ///    {
  ///      X0, Y0, Z0, I0
  ///      ...
  ///      Xn, Yn, Zn, In
  ///    }
  ///
  /// @warning WritePoint should be called sequentially in the order in which
  /// the points are going to be stored, i.e., starting at channel zero and
  /// increasing steadily.

  class LidarDetection {
    public:
      rpc::Location Point;
      float intensity;
      static const int SIZE = 4;

      LidarDetection(float x, float y, float z, float intensity) :
          Point(x, y, z), intensity{intensity} { }
      LidarDetection(rpc::Location p, float intensity) :
          Point(p), intensity{intensity} { }
  };


  class LidarMeasurement {
    static_assert(sizeof(float) == sizeof(uint32_t), "Invalid float size");

    friend class LidarSerializer;
    friend class LidarHeaderView;

    enum Index : size_t {
      HorizontalAngle,
      ChannelCount,
      SIZE
    };

  public:

    explicit LidarMeasurement(uint32_t ChannelCount = 0u)
      : _header(Index::SIZE + ChannelCount, 0u) {
      _header[Index::ChannelCount] = ChannelCount;
    }

    LidarMeasurement &operator=(LidarMeasurement &&) = default;

    float GetHorizontalAngle() const {
      return reinterpret_cast<const float &>(_header[Index::HorizontalAngle]);
    }

    void SetHorizontalAngle(float angle) {
      std::memcpy(&_header[Index::HorizontalAngle], &angle, sizeof(uint32_t));
    }

    uint32_t GetChannelCount() const {
      return _header[Index::ChannelCount];
    }

    void Reset(uint32_t channels, uint32_t channel_point_count) {
      std::memset(_header.data() + Index::SIZE, 0, sizeof(uint32_t) * GetChannelCount());
      _points.clear();
      _points.reserve(LidarDetection::SIZE * channels * channel_point_count);

      _aux_points.resize(channels);

      for (auto& aux : _aux_points) {
        aux.clear();
        aux.reserve(channel_point_count);
      }
    }

    void WritePointAsync(uint32_t channel, LidarDetection detection) {
      DEBUG_ASSERT(GetChannelCount() > channel);
      _aux_points[channel].emplace_back(detection);
    }

    void SaveDetections() {
      _points.clear();

      for (auto idxChannel = 0u; idxChannel < GetChannelCount(); ++idxChannel) {
        _header[Index::SIZE + idxChannel] = static_cast<uint32_t>(_aux_points.size());
        for (auto& pt : _aux_points[idxChannel]) {
          _points.emplace_back(pt.Point.x);
          _points.emplace_back(pt.Point.y);
          _points.emplace_back(pt.Point.z);
          _points.emplace_back(pt.intensity);
        }
      }

    }

  private:

    std::vector<uint32_t> _header;
    std::vector<std::vector<LidarDetection>> _aux_points;

    std::vector<float> _points;

  };

} // namespace s11n
} // namespace sensor
} // namespace carla
