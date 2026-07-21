// LocalMotionDevice.cpp — backend for this board's own drivers. Implementation.
#include "LocalMotionDevice.h"
#include <Movement/Move.h>
#include <Movement/DriveMovement.h>

MotionDeviceType LocalMotionDevice::GetType() const noexcept {
    return MotionDeviceType::local;
}

const char *_ecv_array LocalMotionDevice::GetName() const noexcept {
    return "local";
}

void LocalMotionDevice::AddAxisMovement(Move& move, const PrepParams& params, uint32_t startTime, size_t logicalDrive, DriverId drive, int32_t delta, MovementFlags moveFlags) noexcept {
    move.AddLinearSegments(logicalDrive, startTime, params, (motioncalc_t) delta, moveFlags);
}

void LocalMotionDevice::AddExtruderMovement(Move& move, const PrepParams& params, uint32_t startTime, size_t logicalDrive, DriverId drive, motioncalc_t delta, bool usePressureAdvance, MovementFlags segFlags) noexcept {
    move.AddLinearSegments(logicalDrive, startTime, params, delta, segFlags.AddIsExtruder());
}

void LocalMotionDevice::EnableDriver (Move& move, DriverId drive, float requiredCurrent) noexcept {
    move.EnableOneLocalDriver(drive.localDriver, requiredCurrent);
}

void LocalMotionDevice::DisableDriver (Move& move, DriverId drive) noexcept {
    move.DisableOneLocalDriver(drive.localDriver);
}

GCodeResult LocalMotionDevice::ConfigureDriver(Move& move, DriverId drive, GCodeBuffer& gb, const StringRef& reply) THROWS (GCodeException) {
    return move.ConfigureLocalDriver(gb, reply, drive.localDriver);
}

void LocalMotionDevice::StopDriver (DriverId, int32_t) noexcept {
    // Local drives are stopped inside the step ISR (DriveMovement::StopLogicalDrive via
    // Move::CheckEndstops), not by an external call, so there is nothing to delegate to here.
    // TODO: route local stop through this method when the endstop/homing path is refactored.
}



