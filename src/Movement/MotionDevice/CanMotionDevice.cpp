// CanMotionDevice.cpp — backend for Duet 3 expansion boards on the main CAN bus. Implementation.
#include "CanMotionDevice.h"
#if SUPPORT_CAN_EXPANSION

#include <Movement/Move.h>
#include <CAN/CanMotion.h>
#include <CAN/CanInterface.h>
#include <CAN/CanDriversData.h>

MotionDeviceType CanMotionDevice::GetType() const noexcept {
    return MotionDeviceType::can;
}

const char *_ecv_array CanMotionDevice::GetName() const noexcept {
    return "can";
}

void CanMotionDevice::StartMovement() noexcept {
    CanMotion::StartMovement();
}

void CanMotionDevice::AddAxisMovement (Move& move, const PrepParams& params, uint32_t startTime, size_t logicalDrive, DriverId driver, int32_t delta, MovementFlags moveFlags) noexcept {
    move.AddLinearSegments(logicalDrive, startTime, params, (motioncalc_t) delta, moveFlags); //shadow tracking
    CanMotion::AddAxisMovement(params, driver, delta);
}

void CanMotionDevice::AddExtruderMovement(Move& move, const PrepParams& params, uint32_t startTime, size_t logicalDrive, DriverId driver, motioncalc_t delta, bool usePressureAdvance, MovementFlags segFlags) noexcept {
    move.AddLinearSegments(logicalDrive, startTime, params, delta, segFlags.AddIsExtruder()); //shadow tracking
    CanMotion::AddExtruderMovement(params, driver, (float) delta, usePressureAdvance);
}

uint32_t CanMotionDevice::FinishMovement (const DDA& dda, uint32_t moveStartTime, bool simulating) noexcept {
    return CanMotion::FinishMovement(dda, moveStartTime, simulating);
}

void CanMotionDevice::EnableDriver (Move&, DriverId driver, float) noexcept {
    // requiredCurrent is unused here: motor current is set separately over CAN, not by the enable message.
    // TODO: this sends one message per driver, losing the per-board batching Move::EnableDrivers does currently.
    CanDriversList list;
    list.AddEntry(driver);
    CanInterface::EnableRemoteDrivers(list);
}

void CanMotionDevice::DisableDriver (Move&, DriverId driver) noexcept {
    CanDriversList list;
    list.AddEntry(driver);
    CanInterface::DisableRemoteDrivers(list);
}

GCodeResult CanMotionDevice::ConfigureDriver(Move&, DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) {
    return CanInterface::ConfigureRemoteDriver(driver, gb, reply);
}

void CanMotionDevice::StopDriver (DriverId driver, int32_t netStepsTaken) noexcept {
    (void)CanMotion::StopDriverWhenExecuting(driver, netStepsTaken);		// return value (whether the driver was stopped) is not needed here
}

#endif /* SUPPORT_CAN_EXPANSION */
