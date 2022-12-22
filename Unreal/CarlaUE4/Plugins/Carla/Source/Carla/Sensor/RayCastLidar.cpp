// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/RayCastLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "JsonUtilities.h"

FActorDefinition ARayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
}


ARayCastLidar::ARayCastLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);

  //Cargar el reflectivitymap desde un archivo json
  //const FString JsonMaterialsPath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";
  LoadReflectivityMapFromJson();

}

void ARayCastLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastLidar::Set(const FLidarDescription &LidarDescription)
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

void ARayCastLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidar::PostPhysTick);
  SimulateLidar(DeltaTime);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    DataStream.Send(*this, LidarData, DataStream.PopBufferFromPool());
  }
}

float ARayCastLidar::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

ARayCastLidar::FDetection ARayCastLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  const float Distance = Detection.point.Length();

  //Atenuacion atmosferica en base a la distancia, por defecto de CARLA
  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  //MEJORAS DEL MODELO
  //Efecto del angulo del incidencia
  
  //SACAR EN OTRA FUNCION 
  
  //Posicion del sensor
  FVector SensorLocation = SensorTransf.GetLocation(); 
  //Vector incidente, normalizado, entre sensor y punto de hit con el target
  FVector VectorIncidente = - (HitPoint - SensorLocation).GetSafeNormal(); 
  //Vector normal a la superficie de hit, normalizado
  FVector VectorNormal = HitInfo.ImpactNormal;
  //Producto punto entre ambos vector, se obtiene el coseno del ang de incidencia
  float CosAngle = FVector::DotProduct(VectorIncidente, VectorNormal);
  CosAngle = sqrtf(CosAngle);
  
  //Efecto de la reflectividad del material
  AActor* ActorHit = HitInfo.GetActor();
  FString ActorHitName = ActorHit->GetName();

  

  //WriteFile(ActorHitName);

  const double* ReflectivityPointer;
  float ReflectivityValue;
  bool MaterialFound=false;
  //Obtener el nombre del material del hit
  //GLog->Log("CHECK MATERIAL");
  FString MaterialNameHit = GetHitMaterialName(HitInfo);
  //GLog->Log(MaterialNameHit);
  if(ActorHitName.Contains("Audi")){
    //WriteFile(MaterialNameHit);
    for (auto& Elem : ReflectivityMap)
    {
      FString MaterialKey = Elem.Key;
      //comprueba de si el nombre del material esta incluido en el material del hit
      if(MaterialNameHit.Contains(MaterialKey)){
        //cuando se encuentra, se opbtiene el valor de reflectividad asociado a ese material
        ReflectivityValue = (float)Elem.Value;
        MaterialFound=true;
        //WriteFile(MaterialNameHit);
        break;
      }
    }
  }
  
  // recorre el mapa de materiales
  
  if(!MaterialFound){
    if(MaterialNameHit.Compare("DefaultMaterial") != 0){
      //GLog->Log(MaterialNameHit);
    }
    
    ReflectivityValue = 0.1f;  //CAMBIAR REFLECTIVIDAD POR DEFECTO
  }

  
  //Buseuda del material en el mapa de reflectividades
  //if(ReflectivityMap.Contains(MaterialNameHit)){
    //ReflectivityPointer = ReflectivityMap.Find(MaterialNameHit);
    //ReflectivityValue = (float)*ReflectivityPointer;

    //GLog->Log(MaterialNameHit);
    //GLog->Log("reflec:" + FString::SanitizeFloat(ReflectivityValue));
  //}else{
    //ReflectivityValue = 0.1f;  //CAMBIAR REFLECTIVIDAD POR DEFECTO
  //}
  
  //WriteFile(MaterialNameHit);

  //La intensidad del punto tiene en cuenta:
  //Atenuacion atmosferica: la intensidad sera menor a mayor distancia
  //Cos Ang Incidencia: la intensidad mientras mas perpendicular a la superficie sea el rayo incidente
  //Reflectividad del material
  //const float IntRec = CosAngle * AbsAtm * ReflectivityValue;
  const float IntRec = ReflectivityValue;

  Detection.intensity = IntRec;

  return Detection;
}

  void ARayCastLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    Super::PreprocessRays(Channels, MaxPointsPerChannel);

    for (auto ch = 0u; ch < Channels; ch++) {
      for (auto p = 0u; p < MaxPointsPerChannel; p++) {
        RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
      }
    }
  }

  bool ARayCastLidar::PostprocessDetection(FDetection& Detection) const
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

  void ARayCastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
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

  //Funcion implementada para leer desde un json, la reflectividad asociada a cada material
  //y cargarlo en el ReflectivityMap
  void ARayCastLidar::LoadReflectivityMapFromJson(){

    //path del archivo json
    const FString JsonFilePath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";

    //carga el json a un string
    FString JsonString;
    FFileHelper::LoadFileToString(JsonString,*JsonFilePath);

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

  void ARayCastLidar::WriteFile(FString String) const{
    const FString FilePath = FPaths::ProjectContentDir() + "/JsonFiles/materiales.txt";
    FString new_String = FString::Printf( TEXT( "%s \n" ), *String);
    FFileHelper::SaveStringToFile(new_String, *FilePath,
    FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);

  }

  FString ARayCastLidar::GetHitMaterialName(const FHitResult& HitInfo) const{

    UPrimitiveComponent* ComponentHit = HitInfo.GetComponent();
    
    if(ComponentHit){
      if (HitInfo.FaceIndex != -1) {
        int32 section = 0;
        UMaterialInterface* MaterialIntHit = ComponentHit->GetMaterialFromCollisionFaceIndex(HitInfo.FaceIndex, section);
        
        if(MaterialIntHit){
          UMaterial* MaterialHit = MaterialIntHit->GetMaterial();

          if(MaterialHit){
            return MaterialHit->GetName();
          }
        }
      }
    }

    return FString(TEXT("NoMaterial"));
    
  }
