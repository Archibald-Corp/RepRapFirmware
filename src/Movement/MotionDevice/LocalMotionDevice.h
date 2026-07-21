// LocalMotionDevice.h — backend for this board's own drivers (step/dir + phase stepping). Declaration.
#ifndef SRC_MOVEMENT_MOTIONDEVICE_LOCALMOTIONDEVICE_H_
#define SRC_MOVEMENT_MOTIONDEVICE_LOCALMOTIONDEVICE_H_
#include "MotionDeviceInterface.h"

class LocalMotionDevice : public MotionDeviceInterface {
public: 
    MotionDeviceType GetType() const noexcept override;
    const char *_ecv_array GetName() const noexcept override;
	void AddAxisMovement (Move&, const PrepParams&, uint32_t startTime, size_t logicalDrive, DriverId, int32_t delta, MovementFlags) noexcept override;
	void AddExtruderMovement (Move&, const PrepParams&, uint32_t startTime, size_t logicalDrive, DriverId, motioncalc_t delta, bool usePressureAdvance, MovementFlags) noexcept override;
	void EnableDriver (Move&, DriverId, float requiredCurrent) noexcept override;
	void DisableDriver (Move&, DriverId) noexcept override;
	GCodeResult ConfigureDriver(Move&, DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException) override;
	void StopDriver (DriverId, int32_t netStepsTaken) noexcept override;
};

#endif /* SRC_MOVEMENT_MOTIONDEVICE_LOCALMOTIONDEVICE_H_ */