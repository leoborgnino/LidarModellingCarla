// Copyright (c) 2024 Fundacion Fulgor
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <vector>

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/RayCastSemanticLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include <carla/sensor/data/LidarData.h>

// Lidar Transceptor Files
// Tx
#include "Carla/Sensor/LidarTransceptor/src/common/constants.h"
#include "Carla/Sensor/LidarTransceptor/src/tx_lidar/TxLidarPulsed.h"
// Channel
#include "Carla/Sensor/LidarTransceptor/src/channel_lidar/ChannelLidar.h"
// Rx
#include "Carla/Sensor/LidarTransceptor/src/rx_lidar/RxLidarPulsed.h"

#include <compiler/disable-ue4-macros.h>
#include <compiler/enable-ue4-macros.h>

#include "TimeResolvedLidar.generated.h"


// A ray-cast based Lidar sensor with multiple returns, intensity calculation and pulsed transceptor.
UCLASS()
class CARLA_API ATimeResolvedLidar : public ASensor
{
  GENERATED_BODY()

  using FLidarData = carla::sensor::data::LidarData; //
  using FDetection = carla::sensor::data::LidarDetection;

public:
  static FActorDefinition GetSensorDefinition();

  ATimeResolvedLidar(const FObjectInitializer &ObjectInitializer);
  virtual void Set(const FActorDescription &Description);
  virtual void Set(const FLidarDescription &LidarDescription);

  // Funcion que llama el simulador a su vez llama a Simulate Lidar y luego manda los datos
  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime);

private:

  // ********************* //
  //   Métodos Privados    //
  // ********************* //
  
  // Función Principal
  void SimulateLidar(const float DeltaTime);

  // DropOff Modelling
  void PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel);

  // Shoot a laser ray-trace, return whether the laser hit something.
  bool ShootLaser(const float VerticalAngle, float HorizontalAngle, TArray<FHitResult>& HitResults, FCollisionQueryParams& TraceParams, int32 idxChannel, const bool MultiShoot );

  /// This method uses all the saved FHitResults, compute the
  /// RawDetections and then send it to the LidarData structure.
  void ComputeAndSaveDetections(const FTransform& SensorTransform);

  // Entrada: HitResults (colisiones de un haz)
  // Salida -> punto x,y,z,i
  //        -> time_resolved_signal
  FDetection ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf);

  // Transceptor Function
  // Entrada -> Atenuacion, Time Delay en Distancia
  // Salida -> Señal en el tiempo a la salida del receptor
  vector<float> ModelTransceptor(float atenuation, float distance);
  
  // Calculo de la intensidad
  float ComputeIntensity(const FHitResult& HitInfo,const FTransform& SensorTransf);

  /// Utils
  /// Creates a Laser for each channel.
  void CreateLasers();
  bool PostprocessDetection(FDetection& Detection) const;
  bool WriteFile(FString Filename, FString String) ;
  //Funcion para leer un archivo json y cargar el reflectivity map
  void LoadReflectivityMapFromJson(); 
  FString GetHitMaterialName(const FHitResult& HitInfo) const;
  float GetHitMaterialSpecular(const FHitResult& HitInfo) const;
  void LoadActorsList();
  //Calcular el loc de disparo segun el canal
  FVector GetShootLoc(FVector LidarBodyLoc, FRotator ResultRot, int32 idxChannel);
  //Determinar si el objetivo, esta dentro del rango segun su reflectividad
  bool CheckDetectableReflectance(const FHitResult& HitInfo,const FTransform& SensorTransf);
  //Eliminar puntos que corresponden al vehiculo donde esta montado el sensor
  bool UnderMinimumReturnDistance(const FHitResult& HitInfo,const FTransform& SensorTransf);
  //Calcular distancia del hit
  float GetHitDistance(const FHitResult& HitInfo,const FTransform& SensorTransf);
  int32 GetGroupOfChannel(int32 idxChannel);
  float GetMaterialReflectanceValue(FString MaterialNameHit) const;


  // Campos Privados
  FLidarData LidarData;
  UPROPERTY(EditAnywhere)
  FLidarDescription Description;

  TArray<float> LaserAngles;

  vector<vector<vector<FHitResult>>> RecordedHits; // 3D for multiple detections
  //std::vector<std::vector<FHitResult>> RecordedHits;
  vector<vector<bool>> RayPreprocessCondition; 
  vector<uint32_t> PointsPerChannel;
  
  /// Enable/Disable general dropoff of lidar points
  bool DropOffGenActive;

  /// Slope for the intensity dropoff of lidar points, it is calculated
  /// throught the dropoff limit and the dropoff at zero intensity
  /// The points is kept with a probality alpha*Intensity + beta where
  /// alpha = (1 - dropoff_zero_intensity) / droppoff_limit
  /// beta = (1 - dropoff_zero_intensity)
  float DropOffAlpha;
  float DropOffBeta;

  //Map de materialName,reflectivity para todos los materiales 
  //Se lo inicializa leyendo desde un archivo json en el constructor de la clase
  TMap<FString, double> ReflectivityMap;
  float ReflectivityValue;
  
  //Lista de los nombres de actores, para los cuales se van a tener en cuenta los materiales
  //Se lo inicializa leyendo desde un archivo json en el constructor de la clase
  TArray<FString> ActorsList;

  // Transceptor LiDAR
  parametersLiDAR params;

  TxLidarPulsed * tx_lidar;
  ChannelLidar *  channel_lidar;
  RxLidarPulsed * rx_lidar;

  vector<float> output_rx;

};
