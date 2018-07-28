// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/rpc/MsgPack.h"

#ifdef LIBCARLA_INCLUDED_FROM_UE4
#  include "Transform.h"
#endif // LIBCARLA_INCLUDED_FROM_UE4

namespace carla {
namespace rpc {

  class Location {
  public:

    Location() = default;

    Location(float ix, float iy, float iz)
      : x(ix),
        y(iy),
        z(iz) {}

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

#ifdef LIBCARLA_INCLUDED_FROM_UE4

    Location(const FVector &vector) // from centimeters to meters.
      : Location(1e-2f * vector.X, 1e-2f * vector.Y, 1e-2f * vector.Z) {}

    operator FVector() const {
      return FVector{1e2f * x, 1e2f * y, 1e2f * z}; // from meters to centimeters.
    }

#endif // LIBCARLA_INCLUDED_FROM_UE4

    // =========================================================================
    /// @todo The following is copy-pasted from MSGPACK_DEFINE_ARRAY.
    /// This is a workaround for an issue in msgpack library. The
    /// MSGPACK_DEFINE_ARRAY macro is shadowing our `z` variable.
    /// https://github.com/msgpack/msgpack-c/issues/709
    // =========================================================================
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        clmdep_msgpack::type::make_define_array(x, y, z).msgpack_pack(pk);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o)
    {
        clmdep_msgpack::type::make_define_array(x, y, z).msgpack_unpack(o);
    }
    template <typename MSGPACK_OBJECT>
    void msgpack_object(MSGPACK_OBJECT* o, clmdep_msgpack::zone& sneaky_variable_that_shadows_z) const
    {
        clmdep_msgpack::type::make_define_array(x, y, z).msgpack_object(o, sneaky_variable_that_shadows_z);
    }
    // =========================================================================
  };

  class Rotation {
  public:

    Rotation() = default;

    Rotation(float p, float y, float r)
      : pitch(p),
        yaw(y),
        roll(r) {}

    float pitch = 0.0f;
    float yaw = 0.0f;
    float roll = 0.0f;

#ifdef LIBCARLA_INCLUDED_FROM_UE4

    Rotation(const FRotator &rotator)
      : Rotation(rotator.Pitch, rotator.Yaw, rotator.Roll) {}

    operator FRotator() const {
      return FRotator{pitch, yaw, roll};
    }

#endif // LIBCARLA_INCLUDED_FROM_UE4

    MSGPACK_DEFINE_ARRAY(pitch, yaw, roll);
  };

  class Transform {
  public:

    Transform() = default;

    Transform(const Location &in_location, const Rotation &in_rotation)
      : location(in_location),
        rotation(in_rotation) {}

    Location location;
    Rotation rotation;

#ifdef LIBCARLA_INCLUDED_FROM_UE4

    Transform(const FTransform &transform)
      : Transform(Location(transform.GetLocation()), Rotation(transform.Rotator())) {}

    operator FTransform() const {
      const FVector scale{1.0f, 1.0f, 1.0f};
      return FTransform{FRotator(rotation), FVector(location), scale};
    }

#endif // LIBCARLA_INCLUDED_FROM_UE4

    MSGPACK_DEFINE_ARRAY(location, rotation);
  };

} // namespace rpc
} // namespace carla
