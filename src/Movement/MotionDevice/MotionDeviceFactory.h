// MotionDeviceFactory.h — resolves a DriverId to the motion backend that owns it. Declaration.
#ifndef SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEFACTORY_H_
#define SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEFACTORY_H_

#include "MotionDeviceInterface.h"

namespace MotionDeviceFactory {
    void Init() noexcept;
    MotionDeviceInterface& ResolveDriver(DriverId) noexcept;
    MotionDeviceInterface& GetDevice(MotionDeviceType) noexcept;
}

#endif /* SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEFACTORY_H_ */ 