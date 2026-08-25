// LocalMotionDevice.h — backend for this board's own drivers (step/dir + phase stepping). Declaration.
#ifndef SRC_MOVEMENT_MOTIONDEVICE_LOCALMOTIONDEVICE_H_
#define SRC_MOVEMENT_MOTIONDEVICE_LOCALMOTIONDEVICE_H_
#include "MotionDeviceInterface.h"

class LocalMotionDevice : public MotionDeviceInterface {
public: 
    MotionDeviceType GetType() const noexcept override;
    const char *_ecv_array GetName() const noexcept override;
	void AddAxisMovement (const PrepParams& params, DriverId driver, int32_t delta) noexcept override;
	void AddExtruderMovement (const PrepParams& params, DriverId driver, motioncalc_t delta, bool usePressureAdvance) noexcept override;
	void EnableDriver (Move&, const CanDriversList& drivers, float requiredCurrent) noexcept override;
	void DisableDriver (Move&, const CanDriversList& drivers) noexcept override;
	GCodeResult ConfigureDriver(Move&, DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException) override;
	void StopDriver (DriverId, int32_t netStepsTaken) noexcept override;
};

#endif /* SRC_MOVEMENT_MOTIONDEVICE_LOCALMOTIONDEVICE_H_ */