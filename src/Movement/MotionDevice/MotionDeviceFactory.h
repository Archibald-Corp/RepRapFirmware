// MotionDeviceFactory.h — resolves a DriverId to the motion backend that owns it. Declaration.
#ifndef SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEFACTORY_H_
#define SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEFACTORY_H_

#include "MotionDeviceInterface.h"
#include <General/function_ref.h>

namespace MotionDeviceFactory {
    void Init() noexcept;
    MotionDeviceInterface& ResolveDriver(DriverId) noexcept;
    MotionDeviceInterface& GetDevice(MotionDeviceType) noexcept;
    bool IsHandledBy(DriverId driver, MotionDeviceType type) noexcept;										// true if driver's backend (ResolveDriver) is of this type
    void ForEachDevice(function_ref_noexcept<void(MotionDeviceInterface&) noexcept> func) noexcept;		// call func on every backend built into this firmware
    GCodeResult ConfigureBinding(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// M569.10: bind a driver number to a backend by name
}

#endif /* SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEFACTORY_H_ */ 