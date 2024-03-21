// Copyright (c) 2024 Fundacion Fulgor
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include <chrono>

#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/TimeResolvedLidar.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "JsonUtilities.h"

namespace crp = carla::rpc;

// Sensor definition for Carla Top
FActorDefinition ATimeResolvedLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_time_resolved"));
}

// *********************************************************************************** //
//                         Configuraciones Iniciales                                   //
// *********************************************************************************** //

// Constructor
ATimeResolvedLidar::ATimeResolvedLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  // Random Engine
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);
  // Datos de Materiales y Actores
  // Cargar el reflectivitymap desde un archivo json
  LoadReflectivityMapFromJson();
  // Cargar la lista de actores desde un archivo json 
  LoadActorsList();
}

// Configuracion del Lidar segun parametros de la API
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

  // Params Transceptor
  params.LAMBDA0 = Description.LAMBDA0; 
  params.MAX_RANGE = Description.Range/100.0f; // Warning: Centimeters?
  params.DEBUG_GLOBAL = Description.DEBUG_GLOBAL;
  params.LOG_TX  = Description.LOG_TX;
  params.LOG_RX  = Description.LOG_RX;
  params.LOG_CHANNEL  = Description.LOG_CHANNEL;
  params.PTX  = Description.PTX;
  params.TAU_SIGNAL = Description.TAU_SIGNAL ;
  params.TX_FS = Description.TX_FS;
  params.TX_NOS = Description.TX_NOS;
  params.ARX = Description.ARX;
  params.CH_FS = Description.CH_FS;
  params.CH_NOS = Description.CH_NOS;
  params.PRX = Description.PRX;
  params.RPD = Description.RPD;
  params.RX_FS = Description.RX_FS;
  params.RX_NOS = Description.RX_NOS;
  
  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();

  // LiDAR Transceptor
  tx_lidar = new TxLidarPulsed();
  tx_lidar->init(&params);
  
  channel_lidar = new ChannelLidar();
  channel_lidar->init(&params);

  rx_lidar = new RxLidarPulsed();
  rx_lidar->init(&params);
}

// *********************************************************************************** //
//                              Adquisición de Datos                                   //
// *********************************************************************************** //

// Llamado Principal: Ejecutado por el top de carla
// 1. Modela el LiDAR
// 2. Envía los datos serial hacia el cliente
void ATimeResolvedLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ATimeResolvedLidar::PostPhysTick);
  // Adquisición de datos
  SimulateLidar(DeltaTime);
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    // Envío de datos serial
    DataStream.Send(*this, LidarData, DataStream.PopBufferFromPool());
  }
}

// Escritura de los datos del LiDAR en Paralelo y asíncronamente
// 1. Traza los rayos, genera los FHitResult y los escribe
//    en RecordedHits que es un campo de la clase
// 2. Llama a ComputeAndSaveDetections
void ATimeResolvedLidar::SimulateLidar(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ATimeResolvedLidar::SimulateLidar);
  
  const bool ModelMultipleReturn = Description.ModelMultipleReturn;
  const uint32 ChannelCount = Description.Channels;

  // Last Horizontal Angle
  const float CurrentHorizontalAngle = carla::geom::Math::ToDegrees(LidarData.GetHorizontalAngle());
  // Angulo total barrido segun el tiempo delta
  const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov * DeltaTime;
  // Cantidad de puntos por canal
  const uint32_t PointsToScanWithOneLaser = AngleDistanceOfTick / Description.HorizontalFOVRes;

  // Resetea el vector y reserva memoria fija
  // Size: Cantidad de Puntos por Canal * NumReturnsMax
  // Crea un arreglo igual 
  //ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser*Description.NumReturnsMax);
  RecordedHits.resize(ChannelCount);
  for (int i = 0; i < ChannelCount; ++i) {
    RecordedHits[i].resize(PointsToScanWithOneLaser);
    for (int j = 0; j < PointsToScanWithOneLaser; ++j) {
      RecordedHits[i][j].clear();
      RecordedHits[i][j].reserve(Description.NumReturnsMax);
    }
  }
  // Crea un vector (PreprocessRay) todo en true
  // Warning: solo uno por todos los hits de un multiple return
  PreprocessRays(ChannelCount, PointsToScanWithOneLaser*Description.NumReturnsMax);

  // Parámetros de Tracing
  FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;
  
  // RayTracing Engine
  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(ChannelCount, [&](int32 idxChannel)
      {
	TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);
	
	// ShootLaser por cada canal y cada angulo del delta de angulo horizontal
	//  segun el tick del simulador
	for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
	  // Ángulo vertical y horizontal
	  const float VertAngle = LaserAngles[idxChannel];
	  const float HorizAngle = std::fmod(CurrentHorizontalAngle + Description.HorizontalFOVRes
					     * idxPtsOneLaser, Description.HorizontalFov) - Description.HorizontalFov / 2;
	  // DropOff (Descartar aleatoriamente rayos)
	  const bool PreprocessResult = RayPreprocessCondition[idxChannel][idxPtsOneLaser];

	  // Trazado del Rayo
	  TArray<FHitResult> HitsResult;
	  if ( PreprocessResult )
	    if ( ShootLaser(VertAngle, HorizAngle, HitsResult, TraceParams, idxChannel, ModelMultipleReturn) )
	      {
		uint16_t cnt_hit = 0;
		for (auto& hitInfo : HitsResult)
		  if(cnt_hit < Description.NumReturnsMax)
		    RecordedHits[idxChannel][cnt_hit].emplace_back(hitInfo);
		cnt_hit++;
	      }
	};
      });
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  // Procesar los hits de los lasers
  FTransform ActorTransf = GetTransform();
  ComputeAndSaveDetections(ActorTransf);

  // Actualizar nuevo angulo horizontal
  const float HorizontalAngle = carla::geom::Math::ToRadians(
							     std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
  LidarData.SetHorizontalAngle(HorizontalAngle);
}

//
void ATimeResolvedLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel)
{
  // Reset Vector
  RayPreprocessCondition.resize(Channels);
  for (auto& conds : RayPreprocessCondition) {
    conds.clear();
    conds.resize(MaxPointsPerChannel);
    std::fill(conds.begin(), conds.end(), true);
  }

  // Generate random false in the vector to avoid
  // shoot laser 
  for (auto ch = 0u; ch < Channels; ch++) {
    for (auto p = 0u; p < MaxPointsPerChannel; p++) {
      RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
    }
       
  }
}


bool ATimeResolvedLidar::ShootLaser(const float VerticalAngle, const float HorizontalAngle, TArray<FHitResult>& HitResults, FCollisionQueryParams& TraceParams, int32 idxChannel, const bool MultiShoot)
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  FHitResult HitInfo(ForceInit);

  FTransform ActorTransf = GetTransform();
  FVector LidarBodyLoc = ActorTransf.GetLocation();
  FRotator LidarBodyRot = ActorTransf.Rotator();

  FRotator LaserRot (VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
    LaserRot,
    LidarBodyRot
  );

  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  //Calcular la posicion del disparo segun el canal
  FVector ShootLoc = GetShootLoc(LidarBodyLoc, ResultRot, idxChannel);

  //CAMBIOS DE MODELO  
  //El Trace debe ser complejo y retornar el face index para obtener el material
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnFaceIndex = true;

  // Only Debug Time Trace
  double TimeStampStart;
  if(Description.DEBUG_GLOBAL)
    TimeStampStart = FPlatformTime::Seconds() * 1000.0;

  // Shoot Laser
  if (MultiShoot)
    GetWorld()->LineTraceMultiByChannel( // Parallel ?
      HitResults,
      ShootLoc,
      EndTrace,
      ECC_GameTraceChannel2,
      TraceParams,
      FCollisionResponseParams::DefaultResponseParam
    );
  else
  {
    HitResults.Add(HitInfo);
    GetWorld()->ParallelLineTraceSingleByChannel(
      HitResults[0],
      ShootLoc,
      EndTrace,
      ECC_GameTraceChannel2,
      TraceParams,
      FCollisionResponseParams::DefaultResponseParam
    );
  }

  double TimeStampEnd;
  double TimeTrace;
  // Only Debug Time Trace
  if(Description.DEBUG_GLOBAL)
  {
    TimeStampEnd = FPlatformTime::Seconds() * 1000.0;
    TimeTrace = TimeStampEnd - TimeStampStart;
  }

  // Get Distances from Hits
  TArray<float> DistanceTraces;
  uint32_t cnt_hit = 0;
  bool state_hits = true;
  for (const auto& hitInfo : HitResults) 
  { 
    float DistanceTrace = GetHitDistance(hitInfo,ActorTransf);
    DistanceTraces.Add(DistanceTrace);

    if(Description.DEBUG_GLOBAL)
      {
      // Log Time/Distance
      //nombre del archivo para log
      FString NameLogFile = TEXT("Log_channel_") + FString::FromInt(idxChannel) + TEXT(".txt");
      //tiempo del disparo para log
      FString TimeLog = TEXT("Tiempo:") +FString::SanitizeFloat(TimeTrace);
      //ditancia del disparo para log
      FString DistLog = TEXT("Distancia:") +FString::SanitizeFloat(DistanceTrace);
      WriteFile(NameLogFile,DistLog);
      WriteFile(NameLogFile,TimeLog);
    }

    //eliminar puntos que son del vehiculo recolector de datos
    if(UnderMinimumReturnDistance(hitInfo,ActorTransf))
      state_hits = false;

    //determinar si el punto corresponde a una reflectancia detectable
    if(!CheckDetectableReflectance(hitInfo,ActorTransf))
      state_hits = false;

    // Blocking Removed: I think I don't care in multiple returns
    //if (hitInfo.bBlockingHit) {
    //  HitResult = hitInfo;
    //}
  }
  return state_hits;
}


/// This method uses all the saved FHitResults, compute the
/// RawDetections and then send it to the LidarData structure.
/// Usa RecordedHits para pasar ya a la estructura de x,y,z,time_res
void ATimeResolvedLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
  // WARNING: Check for Multiple Returns
  // Variable size of points by channel, horizontal nop
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
    PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();

  // Reset Data Lidar
  LidarData.ResetMemory(PointsPerChannel);

  // 
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
    for (auto& channel_hits : RecordedHits[idxChannel]) {
      for (auto& hit : channel_hits ) {
	// Warning: relacionar los hits (por ahora como si fueran independientes)
	FDetection Detection = ComputeDetection(hit, SensorTransform);
	// Dropoff Intensity-based
	if (PostprocessDetection(Detection))
	  LidarData.WritePointSync(Detection);	    
	else
	  PointsPerChannel[idxChannel]--;
      }
    }
  }
  // Escribir en el header el numero de puntos por canal
  // Warning: phase error ??
  LidarData.WriteChannelCount(PointsPerChannel);
}

ATimeResolvedLidar::FDetection ATimeResolvedLidar::ComputeDetection(const FHitResult& HitResult, const FTransform& SensorTransf)
{
  auto start = std::chrono::high_resolution_clock::now();

  // Info From Hitpoints
  //Atenuacion atmosferica en base a la distancia, por defecto de CARLA
  
  FDetection Detections;

  float Intensity = 1.0;
      
  // Càlculo de la intensidad
  if (Description.INTENSITY_CALC)
    Intensity = ComputeIntensity(HitResult,SensorTransf);

  // Point and intensity
  FDetection Detection;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitResult.ImpactPoint);
  Detection.intensity = Intensity;
  
  // La intensidad del punto tiene en cuenta:
  // Atenuacion atmosferica -> la intensidad sera menor a mayor distancia
  // Cos Ang Incidencia -> la intensidad mientras mas perpendicular a la superficie sea el rayo incidente

  if (Description.TRANS_ON){

    output_rx = ModelTransceptor(Intensity, Detection.point.Length());
    Detection.time_signal = output_rx;

    // Calculo de la distancia
    auto it = max_element(output_rx.begin(),output_rx.end());
    int max_idx = distance(output_rx.begin(),it);
    double max_value = *it;
    double distance = ((max_idx+1-int((2*params.MAX_RANGE/LIGHT_SPEED)*params.TX_FS*params.TX_NOS))/(params.RX_FS*params.RX_NOS))*LIGHT_SPEED/2;      // Calculo de la distancia
    auto vector_proc = (Detection.point*distance);

    // Only Debug
    if(params.DEBUG_GLOBAL){
      if (params.LOG_RX){
	cout << "Punto: " << Detection.point.x << " " << Detection.point.y << " " << Detection.point.z << endl;
	cout << "Punto: " << vector_proc.x << " " << vector_proc.y << " " << vector_proc.z << endl;
	UE_LOG(LogTemp, Log, TEXT("Distancia: %f"), Detection.point.Length());
	cout << "Distancia Receptor: " << distance << endl;
	cout << "******************* Detección ***************" << endl;
	for (auto& i : Detection.time_signal)
	  cout <<  i << " ";
	cout << endl;
      }
    }  
    Detection.point.x = vector_proc.x;
    Detection.point.y = vector_proc.y;
    Detection.point.z = -vector_proc.z;
  }


  if(Description.DEBUG_GLOBAL)
    {
      using nano = std::chrono::nanoseconds;
      auto finish = std::chrono::high_resolution_clock::now();
      std::cout << "RAYCAST ELAPSED: "
		<< std::chrono::duration_cast<nano>(finish - start).count()
		<< " nanoseconds\n";
    }
  return Detection;
}

bool ATimeResolvedLidar::PostprocessDetection(FDetection& Detection) const
{
  const float Intensity = Detection.intensity;
  if(Intensity > Description.DropOffIntensityLimit)
    return true;
  else
    return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
}



float ATimeResolvedLidar::ComputeIntensity(const FHitResult& HitInfo, const FTransform& SensorTransf)
{
  //Posicion del sensor
  FVector SensorLocation = SensorTransf.GetLocation(); 

  // Distancia del hit point
  FVector HitPoint = HitInfo.ImpactPoint;
  const float Distance = (SensorTransf.Inverse().TransformPosition(HitPoint)).Size();

  // Componente Atmosférica
  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);
  
  //Vector incidente, normalizado, entre sensor y punto de hit con el target
  FVector VectorIncidente = - (HitPoint - SensorLocation).GetSafeNormal();
  FVector VectorIncidente_t = VectorIncidente * Distance;
  
  //Vector normal a la superficie de hit, normalizado
  FVector VectorNormal = HitInfo.ImpactNormal;
  //Producto punto entre ambos vector, se obtiene el coseno del ang de incidencia
  const float CosAngle = FVector::DotProduct(VectorIncidente, VectorNormal);
  //CosAngle = sqrtf(CosAngle);
  
  //Efecto de la reflectividad del material
  AActor* ActorHit = HitInfo.GetActor();
  FString ActorHitName = ActorHit->GetName();

  const double* ReflectivityPointer;
  bool MaterialFound = false;
  bool ActorFound = false;

  //Determinar si el actor del hit, esta dentro de los actores a los cuales computar los materiales
  for (int32 i=0; i!=ActorsList.Num();i++){
    if(ActorHitName.Contains(ActorsList[i])){
      ActorFound=true;
      break;
    }
  }

  float MaterialSpecular = GetHitMaterialSpecular(HitInfo);
  if(params.DEBUG_GLOBAL)
    {
      UE_LOG(LogTemp, Log, TEXT("Specular: %s"), MaterialSpecular);
      UE_LOG(LogTemp, Log, TEXT("Vector3: %s"), *(VectorIncidente*Distance).ToString());
      UE_LOG(LogTemp, Log, TEXT("Vector: %s"), *VectorIncidente.ToString());		  
      UE_LOG(LogTemp, Log, TEXT("Vector2: %s"), *VectorIncidente_t.ToString());
    } 

  //Segun si el nombre del actor, corresponde a un actor al cual computar su material
  if( ActorFound ){
    //Se obtiene el nombre del material del hit
    FString MaterialNameHit = GetHitMaterialName(HitInfo);
    //FString MaterialSpecular = GetHitMaterialName(HitInfo);

    if(params.DEBUG_GLOBAL)
      UE_LOG(LogTemp, Log, TEXT("Material: %s"), *MaterialNameHit);

    //Se recorre la lista de materiales con su respectiva reflectividad
    for (auto& Elem : ReflectivityMap){
      FString MaterialKey = Elem.Key;
      //comprueba de si el nombre del material esta incluido en el material del hit
      if(MaterialNameHit.Contains(MaterialKey)){
	//cuando se encuentra, se obtiene el valor de reflectividad asociado a ese material
	ReflectivityValue = (float)Elem.Value;
	if(params.DEBUG_GLOBAL)
	  UE_LOG(LogTemp, Log, TEXT("Reflectividad: %f"), ReflectivityValue);
	MaterialFound=true;
	//WriteFile(MaterialNameHit);
	break;
      }
    }

    if(!MaterialFound){
      //Se le asigna una reflectivdad por defecto a los materiales no criticos
      ReflectivityPointer = ReflectivityMap.Find(TEXT("NoMaterial"));
      ReflectivityValue = (float)*ReflectivityPointer;
    }	
  }

  return CosAngle*ReflectivityValue*AbsAtm;
}


vector<float> ATimeResolvedLidar::ModelTransceptor(float atenuation, float distance)
{
  // LiDAR Transceptor
  vector<float> output_tx;
  output_tx = tx_lidar->run();
  
  vector<float> output_channel;
  output_channel = channel_lidar->run(output_tx,distance,atenuation); // Ojo calcula la intensidad diferente
  
  output_rx = rx_lidar->run(output_tx,output_channel);

  //cout << output_tx.size() << " " << output_channel.size() << " " << endl;

  return output_rx;
    
}



// *********************************************************************************** //
//                                   Utilidades                                        //
// *********************************************************************************** //
void ATimeResolvedLidar::CreateLasers()
{
  const auto NumberOfLasers = Description.Channels;
  check(NumberOfLasers > 0u);
  const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
    (Description.UpperFovLimit - Description.LowerFovLimit) /
    static_cast<float>(NumberOfLasers - 1);
  LaserAngles.Empty(NumberOfLasers);
  for(auto i = 0u; i < NumberOfLasers; ++i)
    {
      const float VerticalAngle =
        Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
      LaserAngles.Emplace(VerticalAngle);
    }
}

//Funcion implementada para leer desde un json, la reflectividad asociada a cada material
//y cargarlo en el ReflectivityMap
void ATimeResolvedLidar::LoadReflectivityMapFromJson(){

  //path del archivo json
  const FString FilePath = FPaths::ProjectDir() + "/LidarModelFiles/materials.json";
  
  //const FString JsonFilePath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";

  //carga el json a un string
  FString JsonString;
  FFileHelper::LoadFileToString(JsonString,*FilePath);

  TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject());
  TSharedRef<TJsonReader<>> JsonReader = TJsonReaderFactory<>::Create(JsonString);

  //parsea el string a un jsonobject
  if (FJsonSerializer::Deserialize(JsonReader, JsonObject) && JsonObject.IsValid())
    { 
      //obtener el array de materials
      TArray<TSharedPtr<FJsonValue>> objArray=JsonObject->GetArrayField("materials");
      
      //iterar sobre todos los elmentos del array
      for(int32 index=0;index<objArray.Num();index++)
	{
	  TSharedPtr<FJsonObject> obj = objArray[index]->AsObject();
	  if(obj.IsValid()){
          
	    //de cada elemento, obtener nombre y reflectivity
	    FString name = obj->GetStringField("name");
	    double reflec = obj->GetNumberField("reflectivity");

	    //cargar en el ReflectivityMap
	    ReflectivityMap.Add(name,reflec);

	    GLog->Log("name:" + name);
	    GLog->Log("reflectivity:" + FString::SanitizeFloat(reflec));
	  }
	}
    }
}

void ATimeResolvedLidar::LoadActorsList(){

  //path del archivo json
  const FString FilePath = FPaths::ProjectDir() + "/LidarModelFiles/vehicles.json";
  
  //const FString JsonFilePath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";

  //carga el json a un string
  FString JsonString;
  FFileHelper::LoadFileToString(JsonString,*FilePath);

  TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject());
  TSharedRef<TJsonReader<>> JsonReader = TJsonReaderFactory<>::Create(JsonString);

  //parsea el string a un jsonobject
  if (FJsonSerializer::Deserialize(JsonReader, JsonObject) && JsonObject.IsValid())
    { 
      //obtener el array de materials
      TArray<TSharedPtr<FJsonValue>> objArray=JsonObject->GetArrayField("vehicles");
      
      //iterar sobre todos los elmentos del array
      for(int32 index=0;index<objArray.Num();index++)
	{
	  TSharedPtr<FJsonObject> obj = objArray[index]->AsObject();
	  if(obj.IsValid()){
          
	    //de cada elemento, obtener el nombre del actor
	    FString name = obj->GetStringField("unreal_actor_name");

	    ActorsList.Add(name);

	    GLog->Log("name:" + name);

	  }
	}
    }

}

bool ATimeResolvedLidar::WriteFile(FString Filename, FString String) {
  const FString FilePath = FPaths::ProjectContentDir() + TEXT("/LogFiles/") + Filename;

  FString new_String = FString::Printf( TEXT( "%s \n" ), *String);
  FFileHelper::SaveStringToFile(new_String, *FilePath,
				FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);

  return true;
}

FString ATimeResolvedLidar::GetHitMaterialName(const FHitResult& HitInfo) const{

  UPrimitiveComponent* ComponentHit = HitInfo.GetComponent();
    
  if(ComponentHit){
    if (HitInfo.FaceIndex != -1) {
      int32 section = 0;
      UMaterialInterface* MaterialIntHit = ComponentHit->GetMaterialFromCollisionFaceIndex(HitInfo.FaceIndex, section);
      return MaterialIntHit->GetName();

    }
  }

  return FString(TEXT("NoMaterial"));
    
}

float ATimeResolvedLidar::GetHitMaterialSpecular(const FHitResult& HitInfo) const{

  UPrimitiveComponent* ComponentHit = HitInfo.GetComponent();
    
  if(ComponentHit){
    if (HitInfo.FaceIndex != -1) {
      int32 section = 0;
      UMaterialInterface* MaterialIntHit = ComponentHit->GetMaterialFromCollisionFaceIndex(HitInfo.FaceIndex, section);
      float ScalarValue;
      UMaterialInstanceDynamic* MaterialInstance = UMaterialInstanceDynamic::Create(MaterialIntHit, nullptr);
      bool bScalarFound = MaterialInstance->GetScalarParameterValue(FName("Specular"), ScalarValue);

      return ScalarValue;

    }
  }

  return -1;
    
}

  float ATimeResolvedLidar::GetHitDistance(const FHitResult& HitInfo,const FTransform& SensorTransf){

    FDetection Detection;
    const FVector HitPoint = HitInfo.ImpactPoint;
    Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

    float Distance = Detection.point.Length();

    return Distance;
  }

  bool ATimeResolvedLidar::CheckDetectableReflectance(const FHitResult& HitInfo,const FTransform& SensorTransf){
    
    const bool ModelReflectanceLimitsFunction = Description.ModelReflectanceLimitsFunction;

    if(ModelReflectanceLimitsFunction){
      float Distance = GetHitDistance(HitInfo,SensorTransf);
      const float Reflectance = GetMaterialReflectanceValue(GetHitMaterialName(HitInfo));

      //Funcion de rango de deteccion segun reflec R(d) = a + b.d^2
      //float a = 0.0005f;
      float a = Description.ReflectanceLimitsFunctionCoeffA;
      //float b = 0.000054f;
      float b = Description.ReflectanceLimitsFunctionCoeffB;

      float ReflectanceLimit = a + b * (Distance*Distance);

      if(Reflectance >= ReflectanceLimit){
        return true;
      }else{
        float dif = ReflectanceLimit - Reflectance;
        float RangeRandom = 0.5 * ReflectanceLimit; //ancho del rango de reflectancia por debajo del umbral, donde el comportamiento es aleatorio
        if(RangeRandom > 0.15){
          RangeRandom = 0.15;}
        return RandomEngine->GetUniformFloat() > (dif/RangeRandom); //si da true, el punto se cuenta, mientras mas grande el dif, menos chances de contar el punto
      }
      
    }else{

      return true;
    }
    
  }
  
  bool ATimeResolvedLidar::UnderMinimumReturnDistance(const FHitResult& HitInfo,const FTransform& SensorTransf){
    //descartar puntos que estan por debajo de la minima
    float Distance = GetHitDistance(HitInfo,SensorTransf);
    float MinimumReturnDistance = 2.5;

    return (Distance <= MinimumReturnDistance);
  }

  FVector ATimeResolvedLidar::GetShootLoc(FVector LidarBodyLoc, FRotator ResultRot, int32 idxChannel){
    //Calcular el punto de disparo de los laser segun el canal
    
    if(Description.ModelHDL64LasersGroups){
      //HDL64 divide los 64 lasers en 2 bloques (upper y lower), con 2 grupos(left y right).

      float VerticalDistance = 2.5;//entre bloques, verticalmente hay 5 cm de distancia, desde el centro seria la mitad
      
      FVector UpTrans= FVector(0.0,0.0,VerticalDistance);
      FVector DownTrans= FVector(0.0,0.0,-1.0*VerticalDistance);

      //Ubicacion del centro de los bloques upper y lower
      FVector UpperBlockLoc = UpTrans + LidarBodyLoc;
      FVector LowerBlockLoc = DownTrans + LidarBodyLoc;

      //Para determinar la posicion de los grupo left y right, se tiene en cuenta la orientacion del sensor
      //y se obtiene el rightVector y leftVector de esa orientacion.

      float HorizontalDistance = 2.5; //entre lentes, horizontalmente hay 5 cm de distancia, desde el centro seria la mitad
      FVector RigthGroupTrans = HorizontalDistance * UKismetMathLibrary::GetRightVector(ResultRot);
      FVector LeftGroupTrans = HorizontalDistance * -1.0 * UKismetMathLibrary::GetRightVector(ResultRot);

      //Ubicacion de cada grupo, desplazando la ubicacion del centro de cada bloque, a la izquierda o derecha
      FVector UpperRigthGroupLoc = UpperBlockLoc + RigthGroupTrans;
      FVector UpperLeftGroupLoc = UpperBlockLoc + LeftGroupTrans;
      FVector LowerRigthGroupLoc = LowerBlockLoc + RigthGroupTrans;
      FVector LowerLeftGroupLoc = LowerBlockLoc + LeftGroupTrans;

      int32 GroupOfLaser = GetGroupOfChannel(idxChannel);
      //0: UpperLeft
      //1: UpperRigth
      //2: LowerLeft
      //3: LowerRigth

      switch(GroupOfLaser){
        case 0:
          return UpperLeftGroupLoc;
        case 1:
          return UpperRigthGroupLoc;
        case 2:
          return LowerLeftGroupLoc;
        case 3:
          return LowerRigthGroupLoc;
      }
    }
      
    return LidarBodyLoc;
  }

  int32 ATimeResolvedLidar::GetGroupOfChannel(int32 idxChannel){
    //Determinar a que grupo corresponde cada canal para el HDL64
    //Segun el manual:
    //0 a 31: upper block, pares left, impares rigth
    //32 a 63: lower block, pares left, impares rigth
    //Se asigna un numero a cada grupo: 
    //0: UpperLeft
    //1: UpperRigth
    //2: LowerLeft
    //3: LowerRigth

    if(idxChannel < 32){
      if(idxChannel%2 == 0){
        return 0;
      }else{
        return 1;
      }
    }else{
      if(idxChannel%2 == 0){
        return 2;
      }else{
        return 3;
      }
    }

  }

float ATimeResolvedLidar::GetMaterialReflectanceValue(FString MaterialNameHit) const
{
    const double* ReflectancePointer;
    bool MaterialFound = false;
    float Reflectance = 1.0;
    //Se recorre la lista de materiales con su respectiva reflectividad
    for (auto& Elem : ReflectivityMap)
    {
      FString MaterialKey = Elem.Key;
      //comprueba de si el nombre del material esta incluido en el material del hit
      if(MaterialNameHit.Contains(MaterialKey)){
        //cuando se encuentra, se obtiene el valor de reflectividad asociado a ese material
        Reflectance = (float)Elem.Value;
        MaterialFound=true;
        //WriteFile(MaterialNameHit);
        break;
      }
    }

    if(!MaterialFound){
      //Se le asigna una reflectivdad por defeto a los materiales no criticos
      ReflectancePointer = ReflectivityMap.Find(TEXT("NoMaterial"));
      Reflectance = (float)*ReflectancePointer;
    }

    return Reflectance;
  }
