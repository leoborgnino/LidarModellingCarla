// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "LidarDescription.generated.h"

USTRUCT()
struct CARLA_API FLidarDescription
{
  GENERATED_BODY()

  /// Number of lasers.
  UPROPERTY(EditAnywhere)
  uint32 Channels = 32u;

  /// Measure distance in centimeters.
  UPROPERTY(EditAnywhere)
  float Range = 1000.0f;

  /// Points generated by all lasers per second.
  UPROPERTY(EditAnywhere)
  uint32 PointsPerSecond = 56000u;

  /// Lidar rotation frequency.
  UPROPERTY(EditAnywhere)
  float RotationFrequency = 10.0f;

  /// Upper laser angle, counts from horizontal, positive values means above
  /// horizontal line.
  UPROPERTY(EditAnywhere)
  float UpperFovLimit = 10.0f;

  /// Lower laser angle, counts from horizontal, negative values means under
  /// horizontal line.
  UPROPERTY(EditAnywhere)
  float LowerFovLimit = -30.0f;
  
  /// Horizontal field of view
  UPROPERTY(EditAnywhere)
  float HorizontalFov = 360.0f;

  /// Attenuation Rate in the atmosphere in m^-1.
  UPROPERTY(EditAnywhere)
  float AtmospAttenRate = 0.004f;

  /// Random seed for the noise/dropoff used by this sensor.
  UPROPERTY(EditAnywhere)
  int RandomSeed = 0;

  /// General drop off rate.
  UPROPERTY(EditAnywhere)
  float DropOffGenRate = 0.45f;

  /// General drop off rate.
  UPROPERTY(EditAnywhere)
  float DropOffIntensityLimit = 0.8f;

  /// General drop off rate.
  UPROPERTY(EditAnywhere)
  float DropOffAtZeroIntensity = 0.4f;

  /// Wether to show debug points of laser hits in simulator.
  UPROPERTY(EditAnywhere)
  bool ShowDebugPoints = false;

  UPROPERTY(EditAnywhere)
  float NoiseStdDev = 0.0f;

  UPROPERTY(EditAnywhere)
  float LAMBDA0 = 950e-9f;
  
  UPROPERTY(EditAnywhere)
  float MAX_RANGE = 50.0f;
  
  UPROPERTY(EditAnywhere)
  bool DEBUG_GLOBAL = false;
  
  UPROPERTY(EditAnywhere)
  bool LOG_TX = false;
  
  UPROPERTY(EditAnywhere)
  bool LOG_RX = false;
  
  UPROPERTY(EditAnywhere)
  bool LOG_CHANNEL = false;
  
  UPROPERTY(EditAnywhere)
  float PTX = 50e-3f;
  
  UPROPERTY(EditAnywhere)
  float TAU_SIGNAL = 5e-9f;
  
  UPROPERTY(EditAnywhere)
  float TX_FS = 2e9f;
  
  UPROPERTY(EditAnywhere)
  int TX_NOS = 2 ;
  
  UPROPERTY(EditAnywhere)
  float ARX = 1.592e-3f;
  
  UPROPERTY(EditAnywhere)
  float CH_FS = 2e9f;
  
  UPROPERTY(EditAnywhere)
  int CH_NOS = 2 ;
  
  UPROPERTY(EditAnywhere)
  float PRX = 1.0f;
  
  UPROPERTY(EditAnywhere)
  float RPD = 0.8f;
  
  UPROPERTY(EditAnywhere)
  float RX_FS = 2e9f;
  
  UPROPERTY(EditAnywhere)
  int RX_NOS = 2 ;
  
  UPROPERTY(EditAnywhere)
  bool TRANS_ON = true;
  
  UPROPERTY(EditAnywhere)
  bool INTENSITY_CALC = true;

  /// Model angle of incidence in LiDAR
  UPROPERTY(EditAnywhere)
  bool ModelAngleofIncidence = false;

  /// Model material in LiDAR
  UPROPERTY(EditAnywhere)
  bool ModelMaterial = false;

  /// Model Multiple Return in LiDAR
  UPROPERTY(EditAnywhere)
  bool ModelMultipleReturn = false;

  /// Std Dev of noise in intensity
  UPROPERTY(EditAnywhere)
  float NoiseStdDevIntensity = 0.0f;

  /// Model reflectance limits function 
  UPROPERTY(EditAnywhere)
	bool ModelReflectanceLimitsFunction = false;

  /// Coefficient a of reflectance limits function-> R(d) = a + b.d^2
  UPROPERTY(EditAnywhere)
  float ReflectanceLimitsFunctionCoeffA = 0.0f;
  
  /// Coefficient b of reflectance limits function-> R(d) = a + b.d^2
  UPROPERTY(EditAnywhere)
  float ReflectanceLimitsFunctionCoeffB = 0.0f;

  /// Model HDL64 lasers groups
  UPROPERTY(EditAnywhere)
  bool ModelHDL64LasersGroups = false;
};
