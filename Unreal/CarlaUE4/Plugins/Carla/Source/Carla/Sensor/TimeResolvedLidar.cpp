// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/TimeResolvedLidar.h"
#include "Carla/Sensor/LidarTransceptor/src/common/constants.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

FActorDefinition ATimeResolvedLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
}


ATimeResolvedLidar::ATimeResolvedLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);

  parametersLiDAR params = {
			      // Global
			    .LAMBDA0 = 950e-9,
			    .MAX_RANGE = 300,
			    .DEBUG_GLOBAL = false,
			    .LOG_TX = false,
			    .LOG_RX = false,
			    .LOG_CHANNEL = false,

			    // TX
			    .PTX = 50e-3,       // Potencia del Transmisor [Watts]
			    .TAU_SIGNAL = 5e-9, // Duracion del pulso [s] si lo aumento, gano SNR, pierdo resolucion
			    .TX_FS = 2e9,       // Frecuencia de Muestreo [Hz]
			    .TX_NOS = 5,        // Sobremuestreo [Veces]

			    // Channel
  
			    .ARX = 1.592e-3,  // Ganancia de la optica del receptor (pi*(2.54e-2/2)^2) diametro 1 pulgada del receptor
			    .CH_FS = 2e9,     // Frecuencia de Muestreo [Hz]
			    .CH_NOS = 5,      // Sobremuestreo [Veces]

			    // RX
			    .PRX = 1,     // Amplificador del detector
			    .RPD = 0.8,   // Sensibilidad del fotodetector [A/W] 
			    .RX_FS = 2e9, // Frecuencia de Muestreo [Hz]
			    .RX_NOS = 5   // Sobremuestreo [Veces]
  };
  
  TxLidarPulsed * tx_lidar = new TxLidarPulsed();
  tx_lidar->init             (     &params     );
  ChannelLidar * channel_lidar = new ChannelLidar();
  channel_lidar->init        (     &params     );
  RxLidarPulsed * rx_lidar = new RxLidarPulsed();
  rx_lidar->init             (     &params     );
  
}

void ATimeResolvedLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ATimeResolvedLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarData = FLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);

  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

void ATimeResolvedLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ATimeResolvedLidar::PostPhysTick);
  SimulateLidar(DeltaTime);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    DataStream.Send(*this, LidarData, DataStream.PopBufferFromPool());
  }
}

float ATimeResolvedLidar::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

ATimeResolvedLidar::FDetection ATimeResolvedLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  const float Distance = Detection.point.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  Detection.intensity = IntRec;

  return Detection;
}

  void ATimeResolvedLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    Super::PreprocessRays(Channels, MaxPointsPerChannel);

    for (auto ch = 0u; ch < Channels; ch++) {
      for (auto p = 0u; p < MaxPointsPerChannel; p++) {
        RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
      }
    }
  }

  bool ATimeResolvedLidar::PostprocessDetection(FDetection& Detection) const
  {
    if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
      const auto ForwardVector = Detection.point.MakeUnitVector();
      const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
      Detection.point += Noise;
    }

    const float Intensity = Detection.intensity;
    if(Intensity > Description.DropOffIntensityLimit)
      return true;
    else
      return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
  }

  void ATimeResolvedLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
      PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();

    LidarData.ResetMemory(PointsPerChannel);

    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
      for (auto& hit : RecordedHits[idxChannel]) {
        FDetection Detection = ComputeDetection(hit, SensorTransform);
        if (PostprocessDetection(Detection))
          LidarData.WritePointSync(Detection);
        else
          PointsPerChannel[idxChannel]--;
      }
    }

    LidarData.WriteChannelCount(PointsPerChannel);
  }
