// CanMotionDevice.h — backend for Duet 3 expansion boards on the main CAN bus. Declaration.
#ifndef SRC_MOVEMENT_MOTIONDEVICE_CANMOTIONDEVICE_H_
#define SRC_MOVEMENT_MOTIONDEVICE_CANMOTIONDEVICE_H_
#include "MotionDeviceInterface.h"

#if SUPPORT_CAN_EXPANSION

class CanMotionDevice : public MotionDeviceInterface {
public:
	MotionDeviceType GetType() const noexcept override;
	const char *_ecv_array GetName() const noexcept override;
	void StartMovement() noexcept override;
	void AddAxisMovement (Move&, const PrepParams&, uint32_t startTime, size_t logicalDrive, DriverId, int32_t delta, MovementFlags) noexcept override;
	void AddExtruderMovement (Move&, const PrepParams&, uint32_t startTime, size_t logicalDrive, DriverId, motioncalc_t delta, bool usePressureAdvance, MovementFlags) noexcept override;
	uint32_t FinishMovement (const DDA&, uint32_t moveStartTime, bool simulating) noexcept override;
	void EnableDriver (Move&, DriverId, float requiredCurrent) noexcept override;
	void DisableDriver (Move&, DriverId) noexcept override;
	GCodeResult ConfigureDriver(Move&, DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException) override;
	void StopDriver (DriverId, int32_t netStepsTaken) noexcept override;
	bool RevertStoppedDrivers() noexcept override;
};

#endif /* SUPPORT_CAN_EXPANSION */

#endif /* SRC_MOVEMENT_MOTIONDEVICE_CANMOTIONDEVICE_H_ */