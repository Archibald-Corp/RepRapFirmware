// CanMotionDevice.cpp — backend for Duet 3 expansion boards on the main CAN bus. Implementation.
#include "CanMotionDevice.h"
#if SUPPORT_CAN_EXPANSION

#include <Movement/Move.h>
#include <CAN/CanMotion.h>
#include <CAN/CanInterface.h>
#include <CAN/CanDriversData.h>
#include <Platform/RepRap.h>					// for reprap.GetExpansion() and reprap.BoardsUpdated()
#include <CAN/ExpansionManager.h>			// for StoreDriverDirection / StoreDriverMode
#include <GCodes/GCodeBuffer/GCodeBuffer.h>	// for gb.TryGetBValue / TryGetUIValue / GetCommandFraction

MotionDeviceType CanMotionDevice::GetType() const noexcept {
    return MotionDeviceType::can;
}

const char *_ecv_array CanMotionDevice::GetName() const noexcept {
    return "can";
}

void CanMotionDevice::StartMovement() noexcept {
    CanMotion::StartMovement();
}

void CanMotionDevice::AddAxisMovement (const PrepParams& params, DriverId driver, int32_t delta) noexcept {
    CanMotion::AddAxisMovement(params, driver, delta); //shadow tracking is in DDA:Prepare
}

void CanMotionDevice::AddExtruderMovement(const PrepParams& params, DriverId driver, motioncalc_t delta, bool usePressureAdvance) noexcept {
    CanMotion::AddExtruderMovement(params, driver, (float) delta, usePressureAdvance); //shadow tracking is in DDA:Prepare
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
    const GCodeResult res = CanInterface::ConfigureRemoteDriver(driver, gb, reply);

    // If it's M569 with an S parameter then store the direction setting. This backend only handles remote drivers,
    // so the id.IsRemote() check from the original GCodes code is implied.
    if (res <= GCodeResult::warning && gb.GetCommandFraction() <= 0) {
        bool direction;
        bool seen = false;
        if (gb.TryGetBValue('S', direction, seen)) {
            reprap.GetExpansion().StoreDriverDirection(driver, direction);
        }
        uint32_t mode;
        if (gb.TryGetUIValue('D', mode, seen)) {
            reprap.GetExpansion().StoreDriverMode(driver, mode);
        }
        if (seen) {
            reprap.BoardsUpdated();
        }
    }
    return res;
}

void CanMotionDevice::StopDriver (DriverId driver, int32_t netStepsTaken) noexcept {
    (void)CanMotion::StopDriverWhenExecuting(driver, netStepsTaken);		// return value (whether the driver was stopped) is not needed here
}

bool CanMotionDevice::RevertStoppedDrivers() noexcept {
    return CanMotion::RevertStoppedDrivers();
}

#endif /* SUPPORT_CAN_EXPANSION */
