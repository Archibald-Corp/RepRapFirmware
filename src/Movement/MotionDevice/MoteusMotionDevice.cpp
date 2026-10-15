// MoteusMotionDevice.cpp — backend for mjbots moteus servos on the secondary CAN bus. Implementation.
#include "MoteusMotionDevice.h"

#if SUPPORT_MOTEUS

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <CAN/Bus/Protocols/MoteusProtocol.h>		// for Moteus::MaxServoId

// Limits for the M569 settings
constexpr float MinStepsPerRev = 1.0;
constexpr float MaxStepsPerRev = 1000000.0;			// beyond this, float positions lose whole-step resolution over long moves
constexpr float MaxTorqueLimit = 100.0;				// N.m; generous ceiling, the real limit is set per motor with T

MotionDeviceType MoteusMotionDevice::GetType() const noexcept {
	return MotionDeviceType::moteus;
}

const char *_ecv_array MoteusMotionDevice::GetName() const noexcept {
	return "moteus";
}

void MoteusMotionDevice::AddAxisMovement(const PrepParams&, DriverId, int32_t) noexcept {
	// No-op: moteus motion is streamed from RRF's planned position (step 5), not sent per move.
}

void MoteusMotionDevice::AddExtruderMovement(const PrepParams&, DriverId, motioncalc_t, bool) noexcept {
	// No-op, as AddAxisMovement.
}

void MoteusMotionDevice::EnableDriver(Move&, const CanDriversList&, float) noexcept {
	// TODO step 7: mark the servos enabled; the streaming task then holds them in position
}

void MoteusMotionDevice::DisableDriver(Move&, const CanDriversList&) noexcept {
	// TODO step 7: queue a stop command for each servo
}

bool MoteusMotionDevice::StopDriver(DriverId, int32_t, bool) noexcept {
	// TODO step 7: freeze a servo that was stopped by its own endstop
	return false;										// nothing for the Duet CAN sender to do
}

// Called by M569.10 P<driver> B"moteus". Creates an entry with nothing set; M569 then fills it in.
GCodeResult MoteusMotionDevice::AttachDriver(DriverId driver, const StringRef& reply) noexcept {
	if (!driver.IsRemote()) {
		reply.copy("a moteus servo needs a driver number on an unused board address, e.g. P100.1");
		return GCodeResult::error;
	}
	if (Find(driver) != nullptr) {
		return GCodeResult::ok;							// already ours: keep its settings
	}
	if (numServos == MaxServos) {
		reply.printf("too many moteus servos (maximum %u)", (unsigned int)MaxServos);
		return GCodeResult::error;
	}
	servos[numServos] = Servo();						// fresh entry
	servos[numServos].driver = driver;
	++numServos;
	return GCodeResult::ok;
}

// Called when a driver we had is bound to a different backend
void MoteusMotionDevice::DetachDriver(DriverId driver) noexcept {
	Servo *_ecv_null const s = Find(driver);
	if (s != nullptr) {
		*s = servos[numServos - 1];						// order doesn't matter: move the last entry into the gap
		--numServos;
	}
	// TODO step 5: guard this against the streaming task reading the table at the same time
}

// M569 P<driver> [S0/S1] [I<servo ID>] [U<steps per rev>] [T<max torque N.m>]. With no parameters, report the settings.
GCodeResult MoteusMotionDevice::ConfigureDriver(Move&, DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) {
	if (gb.GetCommandFraction() > 0) {
		reply.printf("M569.%d is not supported for moteus servos", gb.GetCommandFraction());
		return GCodeResult::errorNotSupported;
	}

	Servo *_ecv_null const s = Find(driver);
	if (s == nullptr) {
		// Shouldn't happen: ResolveDriver only routes drivers bound to this backend here
		reply.printf("Driver %u.%u is not bound to the moteus backend", driver.boardAddress, driver.localDriver);
		return GCodeResult::error;
	}

	bool seen = false;
	bool forwards;
	if (gb.TryGetBValue('S', forwards, seen)) {
		s->reverse = !forwards;
	}

	uint32_t id;
	if (gb.TryGetLimitedUIValue('I', id, seen, 1, Moteus::MaxServoId + 1)) {
		for (size_t i = 0; i < numServos; ++i) {
			if (&servos[i] != s && servos[i].servoId == id) {
				reply.printf("moteus servo ID %u is already used by driver %u.%u",
							 (unsigned int)id, servos[i].driver.boardAddress, servos[i].driver.localDriver);
				return GCodeResult::error;
			}
		}
		s->servoId = (uint8_t)id;
	}

	float f;
	if (gb.TryGetLimitedFValue('U', f, seen, MinStepsPerRev, MaxStepsPerRev)) {
		s->stepsPerRev = f;
	}
	if (gb.TryGetLimitedFValue('T', f, seen, 0.0, MaxTorqueLimit)) {
		s->maxTorque = f;
	}

	if (!seen) {
		reply.printf("Driver %u.%u: moteus servo ", driver.boardAddress, driver.localDriver);
		if (s->servoId == 0) {
			reply.cat("ID not set");
		} else {
			reply.catf("%u", (unsigned int)s->servoId);
		}
		reply.catf(", %s, ", (s->reverse) ? "reverse" : "forwards");
		if (s->stepsPerRev <= 0.0) {
			reply.cat("steps/rev not set");
		} else {
			reply.catf("%.1f steps/rev", (double)s->stepsPerRev);
		}
		reply.catf(", max torque %.2f N.m", (double)s->maxTorque);
	}
	return GCodeResult::ok;
}

MoteusMotionDevice::Servo *_ecv_null MoteusMotionDevice::Find(DriverId driver) noexcept {
	for (size_t i = 0; i < numServos; ++i) {
		if (servos[i].driver == driver) {
			return &servos[i];
		}
	}
	return nullptr;
}

#endif	// SUPPORT_MOTEUS
