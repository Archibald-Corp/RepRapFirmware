// MoteusMotionDevice.h — backend for mjbots moteus servos on the secondary CAN bus. Declaration.
#ifndef SRC_MOVEMENT_MOTIONDEVICE_MOTEUSMOTIONDEVICE_H_
#define SRC_MOVEMENT_MOTIONDEVICE_MOTEUSMOTIONDEVICE_H_

#include "MotionDeviceInterface.h"
#include "MoteusSupport.h"
#include <CAN/Bus/Protocols/MoteusProtocol.h>

#if SUPPORT_MOTEUS

class MoteusMotionDevice final : public MotionDeviceInterface {
public:
	static constexpr size_t MaxServos = 8;
	static constexpr uint32_t MinUpdateIntervalMillis = 2;	// the shortest time between setpoints for a servo (FreeRTOS ticks are 1 ms)
	static constexpr uint32_t LeadClocks = StepClockRate * 2/1000;	// sample the plan this far ahead (2 ms), to cover the time to reach the servo
	static constexpr uint32_t ReplyMarginClocks = StepClockRate/5000;	// stop collecting replies this long before the next cycle is due (0.2 ms)
	static constexpr uint8_t MaxMissedReplies = 5;			// frames in a row an enabled servo may miss before it counts as lost (about 10 ms at the shortest cycle)

	void Spin() noexcept;									// one streaming cycle, called every GetUpdateIntervalMillis() by the moteus task
	uint32_t GetUpdateIntervalMillis() const noexcept { return updateIntervalMillis; }

	MotionDeviceType GetType() const noexcept override;
	const char *_ecv_array GetName() const noexcept override;
	void StartMovement() noexcept override;
	void AddAxisMovement(const PrepParams& params, DriverId driver, int32_t delta) noexcept override;
	void AddExtruderMovement(const PrepParams& params, DriverId driver, motioncalc_t delta, bool usePressureAdvance) noexcept override;
	void EnableDriver(Move&, const CanDriversList& drivers, float requiredCurrent) noexcept override;
	void DisableDriver(Move&, const CanDriversList& drivers) noexcept override;
	GCodeResult ConfigureDriver(Move&, DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException) override;
	bool StopDriver(DriverId driver, int32_t netStepsTaken, bool executingMove) noexcept override;
	GCodeResult AttachDriver(DriverId driver, const StringRef& reply) noexcept override;
	void DetachDriver(DriverId driver) noexcept override;
	bool GetDirection(const Move& move, DriverId driver) const noexcept override;
	void EmergencyStop() noexcept override;

private:
	// One bound servo. Settings come from M569 once M569.10 has bound the driver to this backend.
	struct Servo {
		DriverId driver;					// the RRF driver number, e.g. 100.1
		uint8_t servoId = 0;				// M569 I: moteus CAN ID, 1-127 (0 = not set yet)
		bool reverse = false;				// M569 S0 = reverse, S1 = forwards
		float stepsPerRev = 0.0;			// M569 U: RRF steps per servo revolution = M92 steps/mm × mm of travel per servo revolution (0 = not set yet)
		float maxTorque = 0.0;				// M569 T: maximum torque in N.m, sent with every position command
		float targetRev = 0.0;				// latest planned position in revolutions (before the offset)
		float targetRevPerSec = 0.0;		// latest planned speed in revolutions per second
		bool hasTarget = false;				// false until U is set and M584 has mapped the driver
		float offsetRev = 0.0;				// added to targetRev to get the position to send to the servo
		bool needsAlign = true;				// take a new offset from the next reply
		uint32_t positionSets = 0;			// the drive's position-set count at the last sample, to spot G92 and homing
		size_t drive = 0;					// the logical drive this servo followed at the last sample
		Moteus::ServoState state;			// latest reply: mode, fault, position (rev), velocity, torque, voltage, temperature
		bool awaitingReply = false;			// a frame went out this cycle and its reply hasn't arrived yet
		bool replied = false;				// true if the servo answered its latest finished frame
		bool enabled = false;				// RRF wants this servo holding position (M17 or a move)
		bool stopPending = false;			// send a stop command in the next cycle
		volatile bool frozen = false;		// stopped by its own endstop; holds its command until the next move
		uint8_t missedReplies = 0;			// frames in a row with no answer
	};

	// A servo that CheckHealth turned off. Reported after the table lock is released, because reporting takes other locks.
	struct Failure {
		DriverId driver;
		size_t drive;						// the logical drive it followed, or MaxAxesPlusExtruders if none
		uint8_t servoId;
		uint8_t fault;						// moteus fault code from its latest reply
		bool notAnswering;					// true if it stopped answering, false if it reported a fault
	};

	Servo *_ecv_null Find(DriverId driver) noexcept;
	bool MakeFrame(CanMessageBuffer& buf, Servo& s) noexcept;
	void SendStopNow(CanTransport& bus, const Servo& s) noexcept;
	bool CheckHealth(Servo& s, Failure& failure) noexcept;
	static void ReportFailure(const Failure& failure) noexcept;

	Servo servos[MaxServos];
	size_t numServos = 0;
	uint32_t updateIntervalMillis = MinUpdateIntervalMillis;	// set by Spin from the number of servos it talks to
};

#endif	// SUPPORT_MOTEUS

#endif /* SRC_MOVEMENT_MOTIONDEVICE_MOTEUSMOTIONDEVICE_H_ */
