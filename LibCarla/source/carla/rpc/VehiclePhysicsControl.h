// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/MsgPack.h"
#include "carla/rpc/WheelPhysicsControl.h"
#include "carla/rpc/Vector2D.h"
#include <string>
#include <vector>

namespace carla {
namespace rpc {
  class VehiclePhysicsControl {
  public:

    explicit VehiclePhysicsControl() = default;
    
    explicit VehiclePhysicsControl(
      const std::vector<carla::geom::Vector2D>& in_torque_curve,
      float in_max_rpm,
      float in_moi,
      float in_damping_rate_full_throttle,
      float in_damping_rate_zero_throttle_clutch_engaged,
      float in_damping_rate_zero_throttle_clutch_disengaged,

      bool  in_use_gear_autobox,
      float in_gear_switch_time,
      float in_clutch_strength,

      float in_mass,
      float in_drag_coefficient,
      geom::Vector3D in_inertia_tensor_scale,
      const std::vector<carla::geom::Vector2D>& in_steering_curve,
      std::vector<WheelPhysicsControl>& in_wheels
    ) {

      torque_curve = in_torque_curve;
      max_rpm = in_max_rpm;
      moi = in_moi;
      damping_rate_full_throttle = in_damping_rate_full_throttle;
      damping_rate_zero_throttle_clutch_engaged = in_damping_rate_zero_throttle_clutch_engaged;
      damping_rate_zero_throttle_clutch_disengaged = in_damping_rate_zero_throttle_clutch_disengaged;

      use_gear_autobox = in_use_gear_autobox;
      gear_switch_time = in_gear_switch_time;
      clutch_strength = in_clutch_strength;

      mass = in_mass;
      drag_coefficient = in_drag_coefficient;
      inertia_tensor_scale = in_inertia_tensor_scale;

      steering_curve = in_steering_curve;
      wheels = in_wheels;
    }

    const std::vector<WheelPhysicsControl> GetWheels() const {
      return wheels;
    }

    void SetWheels(std::vector<WheelPhysicsControl> &in_wheels) {
      wheels = in_wheels;
    }

    const std::vector<geom::Vector2D> GetTorqueCurve() const {
      return torque_curve;
    }

    void SetTorqueCurve(std::vector<geom::Vector2D> &in_torque_curve) {
      torque_curve = in_torque_curve;
    }

    const std::vector<geom::Vector2D> GetSteeringCurve() const {
      return steering_curve;
    }

    void SetSteeringCurve(std::vector<geom::Vector2D> &in_steering_curve) {
      steering_curve = in_steering_curve;
    }

    std::vector<geom::Vector2D> torque_curve;
    float max_rpm = 0.0f;
    float moi = 0.0f;
    float damping_rate_full_throttle = 0.0f;
    float damping_rate_zero_throttle_clutch_engaged = 0.0f;
    float damping_rate_zero_throttle_clutch_disengaged = 0.0f;

    bool use_gear_autobox = true;
    float gear_switch_time = 0.0f;
    float clutch_strength = 0.0f;

    float mass = 0.0f;
    float drag_coefficient = 0.0f;
    geom::Vector3D inertia_tensor_scale;

    std::vector<geom::Vector2D> steering_curve;
    std::vector<WheelPhysicsControl> wheels;


    bool operator!=(const VehiclePhysicsControl &rhs) const {
      return
          max_rpm != rhs.max_rpm ||
          moi != rhs.moi ||
          damping_rate_full_throttle != rhs.damping_rate_full_throttle ||
          damping_rate_zero_throttle_clutch_engaged != rhs.damping_rate_zero_throttle_clutch_engaged ||
          damping_rate_zero_throttle_clutch_disengaged != rhs.damping_rate_zero_throttle_clutch_disengaged ||
          
          use_gear_autobox != rhs.use_gear_autobox ||
          gear_switch_time != rhs.gear_switch_time ||
          clutch_strength != rhs.clutch_strength ||
          
          mass != rhs.mass ||
          drag_coefficient != rhs.drag_coefficient ||
          inertia_tensor_scale != rhs.inertia_tensor_scale ||
          steering_curve != rhs.steering_curve ||
          wheels != rhs.wheels;
    }

    bool operator==(const VehiclePhysicsControl &rhs) const {
      return !(*this != rhs);
    }

   #ifdef LIBCARLA_INCLUDED_FROM_UE4

    VehiclePhysicsControl(const FVehiclePhysicsControl &Control) {      
      // Engine Setup
      TArray<FRichCurveKey> TorqueCurveKeys = Control.TorqueCurve.GetCopyOfKeys();
      for(int32 KeyIdx = 0; KeyIdx < TorqueCurveKeys.Num(); KeyIdx++)
      {
        geom::Vector2D point(TorqueCurveKeys[KeyIdx].Time, TorqueCurveKeys[KeyIdx].Value);
        torque_curve.push_back(point);
      }
      max_rpm = Control.MaxRPM;
      moi = Control.MOI;
      damping_rate_full_throttle = Control.DampingRateFullThrottle;
      damping_rate_zero_throttle_clutch_engaged = Control.DampingRateZeroThrottleClutchEngaged;
      damping_rate_zero_throttle_clutch_disengaged = Control.DampingRateZeroThrottleClutchDisengaged;

      // Transmission Setup
      use_gear_autobox = Control.bUseGearAutoBox;
      gear_switch_time = Control.GearSwitchTime;
      clutch_strength = Control.ClutchStrength;

      // Vehicle Setup
      mass = Control.Mass;
      drag_coefficient = Control.DragCoefficient;
      inertia_tensor_scale = Control.InertiaTensorScale;
      
      TArray<FRichCurveKey> SteeringCurveKeys = Control.SteeringCurve.GetCopyOfKeys();
      for(int32 KeyIdx = 0; KeyIdx < SteeringCurveKeys.Num(); KeyIdx++)
      {
          geom::Vector2D point(SteeringCurveKeys[KeyIdx].Time, SteeringCurveKeys[KeyIdx].Value);
          steering_curve.push_back(point);
      }

      // Wheels Setup
      wheels = std::vector<WheelPhysicsControl>();
      for( auto Wheel : Control.Wheels) {
        wheels.push_back(WheelPhysicsControl(Wheel));
      }
    }

    operator FVehiclePhysicsControl() const {
      FVehiclePhysicsControl Control;

      // Engine Setup
      FRichCurve TorqueCurve;
      for (auto point : torque_curve)
        TorqueCurve.AddKey (point.x, point.y);
      Control.TorqueCurve = TorqueCurve;
      Control.MaxRPM = max_rpm;
      Control.MOI = moi;
      Control.DampingRateFullThrottle = damping_rate_full_throttle;
      Control.DampingRateZeroThrottleClutchEngaged= damping_rate_zero_throttle_clutch_engaged;
      Control.DampingRateZeroThrottleClutchDisengaged = damping_rate_zero_throttle_clutch_disengaged;

      // Transmission Setup
      Control.bUseGearAutoBox = use_gear_autobox;
      Control.GearSwitchTime = gear_switch_time;
      Control.ClutchStrength = clutch_strength;

      // Vehicle Setup
      Control.Mass = mass;
      Control.DragCoefficient = drag_coefficient;
      Control.InertiaTensorScale = inertia_tensor_scale;

      // Transmission Setup
      FRichCurve SteeringCurve;
      for (auto point : steering_curve)
        SteeringCurve.AddKey (point.x, point.y);
      Control.SteeringCurve = SteeringCurve;
      
      // Wheels Setup
      TArray<FWheelPhysicsControl> Wheels;
      for (auto wheel : wheels) {
        Wheels.Add(FWheelPhysicsControl(wheel));
      }
      Control.Wheels = Wheels;

      return Control;
    }

    #endif

    MSGPACK_DEFINE_ARRAY(torque_curve, 
                        max_rpm, 
                        moi, 
                        damping_rate_full_throttle, 
                        damping_rate_zero_throttle_clutch_engaged, 
                        damping_rate_zero_throttle_clutch_disengaged,
                        use_gear_autobox,
                        gear_switch_time,
                        clutch_strength,
                        mass,
                        drag_coefficient,
                        inertia_tensor_scale,
                        steering_curve,
                        wheels
                        );
  };

} // namespace rpc
} // namespace carla
