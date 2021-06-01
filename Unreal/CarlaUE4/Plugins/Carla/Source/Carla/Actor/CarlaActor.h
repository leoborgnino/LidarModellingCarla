// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/ActorInfo.h"
#include "Carla/Actor/ActorData.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Walker/WalkerController.h"
#include "Carla/Traffic/TrafficLightState.h"

#include "carla/rpc/ActorState.h"
#include "carla/rpc/AttachmentType.h"

#include "Carla/Server/CarlaServerResponse.h"

class AActor;

/// A view over an actor and its properties.
class FCarlaActor
{
public:

  using IdType = uint32;

  enum class ActorType : uint8
  {
    Other,
    Vehicle,
    Walker,
    TrafficLight,
    TrafficSign,
    Sensor,
    INVALID
  };

  FCarlaActor() = default;
  // FCarlaActor(const FCarlaActor &) = default;
  // FCarlaActor(FCarlaActor &&) = default;

  FCarlaActor(
      IdType ActorId,
      AActor* Actor,
      TSharedPtr<const FActorInfo> Info,
      carla::rpc::ActorState InState);

  virtual ~FCarlaActor() {};


  bool IsInValid() const
  {
    return (carla::rpc::ActorState::Invalid == State);
  }

  bool IsAlive() const
  {
    return (carla::rpc::ActorState::Alive == State);
  }

  bool IsDormant() const
  {
    return (carla::rpc::ActorState::Dormant == State);
  }

  bool IsPendingKill() const
  {
    return (carla::rpc::ActorState::PendingKill == State);
  }

  IdType GetActorId() const
  {
    return Id;
  }

  ActorType GetActorType() const
  {
    return Type;
  }

  AActor *GetActor()
  {
    return TheActor;
  }

  const AActor *GetActor() const
  {
    return TheActor;
  }

  const FActorInfo *GetActorInfo() const
  {
    return Info.Get();
  }

  carla::rpc::ActorState GetActorState() const
  {
    return State;
  }

  void SetActorState(carla::rpc::ActorState InState)
  {
    State = InState;
  }

  void SetParent(IdType InParentId)
  {
    ParentId = InParentId;
  }

  IdType GetParent() const
  {
    return ParentId;
  }

  void AddChildren(IdType ChildId)
  {
    Children.Add(ChildId);
  }

  void RemoveChildren(IdType ChildId)
  {
    Children.Remove(ChildId);
  }

  const TArray<IdType>& GetChildren() const
  {
    return Children;
  }

  void SetAttachmentType(carla::rpc::AttachmentType InAttachmentType)
  {
    Attachment = InAttachmentType;
  }

  carla::rpc::AttachmentType GetAttachmentType() const
  {
    return Attachment;
  }

  void BuildActorData();

  void PutActorToSleep(UCarlaEpisode* CarlaEpisode);

  void WakeActorUp(UCarlaEpisode* CarlaEpisode);

  FActorData* GetActorData()
  {
    return ActorData.Get();
  }

  const FActorData* GetActorData() const
  {
    return ActorData.Get();
  }

  template<typename T>
  T* GetActorData()
  {
    return dynamic_cast<T*>(ActorData.Get());
  }

  template<typename T>
  const T* GetActorData() const
  {
    return dynamic_cast<T*>(ActorData.Get());
  }

  // Actor function interface ----------------------

  // General functions
  ECarlaServerResponse SetActorLocation(const FVector& Location, ETeleportType Teleport);

  ECarlaServerResponse SetActorTransform(const FTransform& Transform, ETeleportType Teleport);

  ECarlaServerResponse SetActorTargetVelocity(const FVector& Velocity);

  ECarlaServerResponse SetActorTargetAngularVelocity(const FVector& AngularVelocity);

  ECarlaServerResponse AddActorImpulse(const FVector& Impulse);

  ECarlaServerResponse AddActorImpulseAtLocation(const FVector& Impulse, const FVector& Location);

  ECarlaServerResponse AddActorForce(const FVector& Force);

  ECarlaServerResponse AddActorForceAtLocation(const FVector& Force, const FVector& Location);

  ECarlaServerResponse AddActorAngularImpulse(const FVector& AngularInpulse);

  ECarlaServerResponse AddActorTorque(const FVector& Torque);

  virtual ECarlaServerResponse SetActorSimulatePhysics(bool bEnabled);

  virtual ECarlaServerResponse SetActorEnableGravity(bool bEnabled);

  // Vehicle functions
  virtual ECarlaServerResponse EnableActorConstantVelocity(const FVector&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse DisableActorConstantVelocity()
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse GetPhysicsControl(FVehiclePhysicsControl&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse GetVehicleLightState(FVehicleLightState&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse ApplyPhysicsControl(const FVehiclePhysicsControl&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse SetVehicleLightState(const FVehicleLightState&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse SetWheelSteerDirection(const EVehicleWheelLocation&, float)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse GetWheelSteerAngle(const EVehicleWheelLocation&, float&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse ApplyControlToVehicle(
      const FVehicleControl&, const EVehicleInputPriority&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse SetActorAutopilot(bool)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse EnableCarSim(const FString&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse UseCarSimRoad(bool)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse EnableChronoPhysics(uint64_t, float,
      const FString&, const FString&, const FString&, const FString&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  // Traffic light functions

  virtual ECarlaServerResponse SetTrafficLightState(const ETrafficLightState&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse SetLightGreenTime(float)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse SetLightYellowTime(float)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse SetLightRedTime(float)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  // Traffic sign functions

  // Walker functions
  virtual ECarlaServerResponse SetWalkerState(
      const FTransform& Transform,
      carla::rpc::WalkerControl WalkerControl,
      float Speed)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse ApplyControlToWalker(const FWalkerControl&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse ApplyBoneControlToWalker(const FWalkerBoneControl&)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse FreezeTrafficLight(bool)
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  virtual ECarlaServerResponse ResetTrafficLightGroup()
  {
    return ECarlaServerResponse::ActorTypeMismatch;
  }

  // Sensor functions

  static TSharedPtr<FCarlaActor> ConstructCarlaActor(
      IdType ActorId,
      AActor* Actor,
      TSharedPtr<const FActorInfo> Info,
      ActorType Type,
      carla::rpc::ActorState InState);

private:

  friend class FActorRegistry;

  AActor *TheActor = nullptr;

  TSharedPtr<const FActorInfo> Info = nullptr;

  IdType Id = 0u;

  IdType ParentId = 0u;

  carla::rpc::ActorState State = carla::rpc::ActorState::Invalid;

  carla::rpc::AttachmentType Attachment = carla::rpc::AttachmentType::INVALID;

  TArray<IdType> Children;

protected:

  ActorType Type = ActorType::INVALID;

  TSharedPtr<FActorData> ActorData = nullptr;

};

class FVehicleActor : public FCarlaActor
{
public:
  FVehicleActor(
      IdType ActorId,
      AActor* Actor,
      TSharedPtr<const FActorInfo> Info,
      carla::rpc::ActorState InState);

  virtual ECarlaServerResponse EnableActorConstantVelocity(const FVector& Velocity) final;

  virtual ECarlaServerResponse DisableActorConstantVelocity() final;

  virtual ECarlaServerResponse GetPhysicsControl(FVehiclePhysicsControl& PhysicsControl) final;

  virtual ECarlaServerResponse GetVehicleLightState(FVehicleLightState& LightState) final;

  virtual ECarlaServerResponse ApplyPhysicsControl(
      const FVehiclePhysicsControl& PhysicsControl) final;

  virtual ECarlaServerResponse SetVehicleLightState(
      const FVehicleLightState& LightState) final;

  virtual ECarlaServerResponse SetWheelSteerDirection(
      const EVehicleWheelLocation& WheelLocation, float AngleInDeg) final;

  virtual ECarlaServerResponse GetWheelSteerAngle(
      const EVehicleWheelLocation& WheelLocation, float& Angle);

  virtual ECarlaServerResponse SetActorSimulatePhysics(bool bSimulatePhysics) final;

  virtual ECarlaServerResponse ApplyControlToVehicle(
      const FVehicleControl&, const EVehicleInputPriority&) final;

  virtual ECarlaServerResponse SetActorAutopilot(bool bEnabled) final;

  virtual ECarlaServerResponse EnableCarSim(const FString& SimfilePath) final;

  virtual ECarlaServerResponse UseCarSimRoad(bool bEnabled) final;

  virtual ECarlaServerResponse EnableChronoPhysics(
      uint64_t MaxSubsteps, float MaxSubstepDeltaTime,
      const FString& VehicleJSON, const FString& PowertrainJSON,
      const FString& TireJSON, const FString& BaseJSONPath) final;
};

class FSensorActor : public FCarlaActor
{
public:
  FSensorActor(
      IdType ActorId,
      AActor* Actor,
      TSharedPtr<const FActorInfo> Info,
      carla::rpc::ActorState InState);

};

class FTrafficSignActor : public FCarlaActor
{
public:
  FTrafficSignActor(
      IdType ActorId,
      AActor* Actor,
      TSharedPtr<const FActorInfo> Info,
      carla::rpc::ActorState InState);
};

class FTrafficLightActor : public FCarlaActor
{
public:
  FTrafficLightActor(
      IdType ActorId,
      AActor* Actor,
      TSharedPtr<const FActorInfo> Info,
      carla::rpc::ActorState InState);

  virtual ECarlaServerResponse SetTrafficLightState(const ETrafficLightState& State) final;

  virtual ECarlaServerResponse SetLightGreenTime(float time) final;

  virtual ECarlaServerResponse SetLightYellowTime(float time) final;

  virtual ECarlaServerResponse SetLightRedTime(float time) final;

  virtual ECarlaServerResponse FreezeTrafficLight(bool bFreeze) final;

  virtual ECarlaServerResponse ResetTrafficLightGroup() final;

};

class FWalkerActor : public FCarlaActor
{
public:
  FWalkerActor(
      IdType ActorId,
      AActor* Actor,
      TSharedPtr<const FActorInfo> Info,
      carla::rpc::ActorState InState);

  virtual ECarlaServerResponse SetWalkerState(
      const FTransform& Transform,
      carla::rpc::WalkerControl WalkerControl,
      float Speed) final;

  virtual ECarlaServerResponse SetActorSimulatePhysics(bool bSimulatePhysics) final;

  virtual ECarlaServerResponse SetActorEnableGravity(bool bEnabled) final;

  virtual ECarlaServerResponse ApplyControlToWalker(const FWalkerControl&) final;

  virtual ECarlaServerResponse ApplyBoneControlToWalker(const FWalkerBoneControl&) final;
};

class FOtherActor : public FCarlaActor
{
public:
  FOtherActor(
      IdType ActorId,
      AActor* Actor,
      TSharedPtr<const FActorInfo> Info,
      carla::rpc::ActorState InState);

};
