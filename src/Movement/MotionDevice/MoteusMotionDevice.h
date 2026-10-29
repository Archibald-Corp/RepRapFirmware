// MoteusMotionDevice.h — backend for mjbots moteus servos on the secondary CAN bus. Declaration.
#ifndef SRC_MOVEMENT_MOTIONDEVICE_MOTEUSMOTIONDEVICE_H_
#define SRC_MOVEMENT_MOTIONDEVICE_MOTEUSMOTIONDEVICE_H_

#include "MotionDeviceInterface.h"
#include "MoteusSupport.h"

#if SUPPORT_MOTEUS

class MoteusMotionDevice final : public MotionDeviceInterface {
public:
	static constexpr size_t MaxServos = 8;

	MotionDeviceType GetType() const noexcept override;
	const char *_ecv_array GetName() const noexcept override;
	void AddAxisMovement(const PrepParams& params, DriverId driver, int32_t delta) noexcept override;
	void AddExtruderMovement(const PrepParams& params, DriverId driver, motioncalc_t delta, bool usePressureAdvance) noexcept override;
	void EnableDriver(Move&, const CanDriversList& drivers, float requiredCurrent) noexcept override;
	void DisableDriver(Move&, const CanDriversList& drivers) noexcept override;
	GCodeResult ConfigureDriver(Move&, DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException) override;
	bool StopDriver(DriverId driver, int32_t netStepsTaken, bool executingMove) noexcept override;
	GCodeResult AttachDriver(DriverId driver, const StringRef& reply) noexcept override;
	void DetachDriver(DriverId driver) noexcept override;
	bool GetDirection(const Move& move, DriverId driver) const noexcept override;

private:
	// One bound servo. Settings come from M569 once M569.10 has bound the driver to this backend.
	struct Servo {
		DriverId driver;					// the RRF driver number, e.g. 100.1
		uint8_t servoId = 0;				// M569 I: moteus CAN ID, 1-127 (0 = not set yet)
		bool reverse = false;				// M569 S0 = reverse, S1 = forwards
		float stepsPerRev = 0.0;			// M569 U: RRF steps per servo revolution (0 = not set yet)
		float maxTorque = 0.0;				// M569 T: maximum torque in N.m, sent with every position command
	};

	Servo *_ecv_null Find(DriverId driver) noexcept;

	Servo servos[MaxServos];
	size_t numServos = 0;
};

#endif	// SUPPORT_MOTEUS

#endif /* SRC_MOVEMENT_MOTIONDEVICE_MOTEUSMOTIONDEVICE_H_ */
