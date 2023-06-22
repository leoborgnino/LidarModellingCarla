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

  //Cargar el Reflectancemap desde un archivo json
  //const FString JsonMaterialsPath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";
  LoadReflectanceMapFromJson();

  //Cargar la lista de actores desde un archivo json 
  LoadVehiclesList();

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

  const float Distance = GetHitDistanceConst(HitInfo,SensorTransf);

  //Atenuacion atmosferica en base a la distancia, por defecto de CARLA
  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);
  //const float AbsAtm = 1.0;

  //MEJORAS DEL MODELO
  //Efecto del angulo del incidencia

  const bool ModelAngleofIncidence = Description.ModelAngleofIncidence;
  const bool ModelMaterial = Description.ModelMaterial;

  float CosAngle = 1.0;
  if (ModelAngleofIncidence)
  {
    CosAngle = GetHitCosIncAngle(HitInfo, SensorTransf);
    CosAngle = sqrtf(CosAngle);
  }
  
  //Efecto de la reflectividad del material

  float Reflectance = 1.0;
  const double* ReflectancePointer;

  if(ModelMaterial){
    AActor* ActorHit = HitInfo.GetActor();
    FString ActorHitName = ActorHit->GetName();

    //Segun si el nombre del actor, corresponde a un actor al cual computar su material
    bool CriticalVehicle = IsCriticalVehicle(ActorHitName);
    if(CriticalVehicle){
      //Se obtiene el nombre del material del hit
      FString MaterialNameHit = GetHitMaterialName(HitInfo);
      //Si el actor corresponde a un ciclista y no se obtiene material, coresponde a la parte de la persona
      if(IsCyclist(ActorHitName) && (MaterialNameHit.Compare("NoMaterial") == 0)){
        Reflectance = GetMaterialReflectanceValue(TEXT("Pedestrian"));
      }else{
        Reflectance = GetMaterialReflectanceValue(MaterialNameHit);
      }
      
    }else if(IsPedestrian(ActorHitName)){
      Reflectance = GetMaterialReflectanceValue(TEXT("Pedestrian"));
      //Reflectivity = 0.1;
    }
    else{
      //Se le asigna una reflectivdad por defeto a los materiales no criticos
      Reflectance = GetMaterialReflectanceValue(TEXT("NoMaterial"));
      //Reflectivity = 0.1;
    }
  }

  //La intensidad del punto tiene en cuenta:
  //Atenuacion atmosferica -> la intensidad sera menor a mayor distancia
  //Cos Ang Incidencia -> la intensidad mientras mas perpendicular a la superficie sea el rayo incidente
  //Reflectividad del material
  float IntensityNoiseStdDev = Description.NoiseStdDevIntensity;
  //const float IntRec = (CosAngle * AbsAtm * Reflectivity / (Distance*Distance)) + RandomEngine->GetNormalDistribution(0.0f, IntensityNoiseStdDev);
  //const float IntRec = (50.0 * CosAngle * AbsAtm * Reflectivity / (Distance*Distance)) + RandomEngine->GetNormalDistribution(0.0f, IntensityNoiseStdDev);
  const float IntRec = (CosAngle * AbsAtm * Reflectance ) + RandomEngine->GetNormalDistribution(0.0f, IntensityNoiseStdDev);
  //const float IntRec = ReflectivityValue;
  if(IntRec <= 0.99 && IntRec > 0.0){
    Detection.intensity = IntRec;
  }else if(IntRec > 0.99){
    Detection.intensity = 0.99;
  }else{
    Detection.intensity = 0.0;
  }
  
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

    /* 
    const float Intensity = Detection.intensity;
    if(Intensity > Description.DropOffIntensityLimit)
      return true;
    else
      return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
    */
    return true;
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
  //y cargarlo en el ReflectanceMap
  void ARayCastLidar::LoadReflectanceMapFromJson(){

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
          ReflectanceMap.Add(name,reflec);

          GLog->Log("name:" + name);
          GLog->Log("reflectivity:" + FString::SanitizeFloat(reflec));
        }
      }
    }
  }

  void ARayCastLidar::LoadVehiclesList(){

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

          VehiclesList.Add(name);

          GLog->Log("name:" + name);

        }
      }
    }

  }

  bool ARayCastLidar::WriteFile(FString Filename, FString String) {
    const FString FilePath = FPaths::ProjectContentDir() + TEXT("/LogFiles/") + Filename;

    FString new_String = FString::Printf( TEXT( "%s \n" ), *String);
    FFileHelper::SaveStringToFile(new_String, *FilePath,
    FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);

    return true;
  }

  FString ARayCastLidar::GetHitMaterialName(const FHitResult& HitInfo) const{

    UPrimitiveComponent* ComponentHit = HitInfo.GetComponent();
    
    if(ComponentHit){
      if (HitInfo.FaceIndex != -1) {
        int32 section = 0;
        UMaterialInterface* MaterialIntHit = ComponentHit->GetMaterialFromCollisionFaceIndex(HitInfo.FaceIndex, section);

        if(MaterialIntHit){
          return MaterialIntHit->GetName();
        }
      }
    }

    return FString(TEXT("NoMaterial"));
    
  }

  float ARayCastLidar::GetHitCosIncAngle(const FHitResult& HitInfo, const FTransform& SensorTransf) const{

    const FVector HitPoint = HitInfo.ImpactPoint;
    //Posicion del sensor
    FVector SensorLocation = SensorTransf.GetLocation(); 
    //Vector incidente, normalizado, entre sensor y punto de hit con el target
    FVector VectorIncidente = - (HitPoint - SensorLocation).GetSafeNormal(); 
    //Vector normal a la superficie de hit, normalizado
    FVector VectorNormal = HitInfo.ImpactNormal;
    //Producto punto entre ambos vector, se obtiene el coseno del ang de incidencia
    float CosAngle = FVector::DotProduct(VectorIncidente, VectorNormal);
    //CosAngle = sqrtf(CosAngle);
    return CosAngle;
  }

  bool ARayCastLidar::IsCriticalVehicle(FString ActorHitName) const{

    bool ActorFound = false;
    //Determinar si el actor del hit, esta dentro de los vevhiculos a los cuales computar los materiales
    for (int32 i=0; i!=VehiclesList.Num();i++){
      if(ActorHitName.Contains(VehiclesList[i])){
        ActorFound=true;
        break;
      }
    }

    return ActorFound;
  }

  bool ARayCastLidar::IsPedestrian(FString ActorHitName) const{

    return ActorHitName.Contains(TEXT("Walker"));
  }

  bool ARayCastLidar::IsCyclist(FString ActorHitName) const{
    return ActorHitName.Contains(TEXT("Bike"));
  }
  float ARayCastLidar::GetMaterialReflectanceValue(FString MaterialNameHit)const {

    const double* ReflectancePointer;
    bool MaterialFound = false;
    float Reflectance = 1.0;
    //Se recorre la lista de materiales con su respectiva reflectividad
    for (auto& Elem : ReflectanceMap)
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
      ReflectancePointer = ReflectanceMap.Find(TEXT("NoMaterial"));
      Reflectance = (float)*ReflectancePointer;
    }

    return Reflectance;
  }

  float ARayCastLidar::GetHitDistance(const FHitResult& HitInfo,const FTransform& SensorTransf){

    FDetection Detection;
    const FVector HitPoint = HitInfo.ImpactPoint;
    Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

    float Distance = Detection.point.Length();

    return Distance;
  }

  float ARayCastLidar::GetHitDistanceConst(const FHitResult& HitInfo,const FTransform& SensorTransf) const{

    FDetection Detection;
    const FVector HitPoint = HitInfo.ImpactPoint;
    Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

    const float Distance = Detection.point.Length();

    return Distance;
  }

  bool ARayCastLidar::CheckDetectableReflectance(const FHitResult& HitInfo,const FTransform& SensorTransf){
    
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
  
  bool ARayCastLidar::UnderMinimumReturnDistance(const FHitResult& HitInfo,const FTransform& SensorTransf){
    //descartar puntos que estan por debajo de la minima
    float Distance = GetHitDistance(HitInfo,SensorTransf);
    float MinimumReturnDistance = 2.5;

    return (Distance <= MinimumReturnDistance);
  }

  FVector ARayCastLidar::GetShootLoc(FVector LidarBodyLoc, FRotator ResultRot, int32 idxChannel){
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

  int32 ARayCastLidar::GetGroupOfChannel(int32 idxChannel){
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