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

void LocalMotionDevice::AddAxisMovement(const PrepParams&, DriverId, int32_t) noexcept {
    // No-op: local axis motion is produced by the shared Move::AddLinearSegments call in DDA::Prepare,
    // which both drives the local motors and tracks position. Nothing backend-specific to do per driver.
    // This lets the interface separate batching from local calls
}

void LocalMotionDevice::AddExtruderMovement(const PrepParams&, DriverId, motioncalc_t, bool) noexcept {
    // No-op: local axis motion is produced by the shared Move::AddLinearSegments call in DDA::Prepare,
    // which both drives the local motors and tracks position. Nothing backend-specific to do per driver.
    // This lets the interface separate batching from local calls
}

void LocalMotionDevice::EnableDriver (Move& move, const CanDriversList& drivers, float requiredCurrent) noexcept {
    for (size_t i = 0; i < drivers.GetNumEntries(); i++) {
        move.EnableOneLocalDriver(drivers.GetEntry(i).localDriver, requiredCurrent);
    }
}

void LocalMotionDevice::DisableDriver (Move& move, const CanDriversList& drivers) noexcept {
    for (size_t i = 0; i < drivers.GetNumEntries(); i++) {
        move.DisableOneLocalDriver(drivers.GetEntry(i).localDriver);
    }
}

GCodeResult LocalMotionDevice::ConfigureDriver(Move& move, DriverId drive, GCodeBuffer& gb, const StringRef& reply) THROWS (GCodeException) {
    return move.ConfigureLocalDriver(gb, reply, drive.localDriver);
}

void LocalMotionDevice::StopDriver (DriverId, int32_t) noexcept {
    // Local drives are stopped inside the step ISR (DriveMovement::StopLogicalDrive via
    // Move::CheckEndstops), not by an external call, so there is nothing to delegate to here.
    // TODO: route local stop through this method when the endstop/homing path is refactored.
}



