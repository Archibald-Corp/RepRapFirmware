// MoteusMotionDevice.cpp — backend for mjbots moteus servos on the secondary CAN bus. Implementation.
#include "MoteusMotionDevice.h"

#if SUPPORT_MOTEUS

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <CAN/Bus/Protocols/MoteusProtocol.h>		// for Moteus::MaxServoId
#include <Platform/TaskPriorities.h>
#include <Platform/RepRap.h>
#include <Movement/Move.h>
#include <Movement/StepTimer.h>
#include <CAN/Bus/SecondaryCanBus.h>
#include <CanMessageBuffer.h>

// Limits for the M569 settings
constexpr float MinStepsPerRev = 1.0;
constexpr float MaxStepsPerRev = 1000000.0;			// beyond this, float positions lose whole-step resolution over long moves
constexpr float MaxTorqueLimit = 100.0;				// N.m; generous ceiling, the real limit is set per motor with T

// The streaming task. It is created when the first servo is bound, so firmware with no moteus servos configured doesn't run it.
constexpr size_t MoteusTaskStackWords = 400;			// same as CanSender; check the high-water mark in M122 once it sends messages
static Task<MoteusTaskStackWords> moteusTask;
static Mutex servoTableMutex;							// held by Spin for a whole cycle, and by anything that changes the servo table

extern "C" [[noreturn]] void MoteusTaskLoop(void *param) noexcept
{
	MoteusMotionDevice& device = *static_cast<MoteusMotionDevice*>(param);
	uint32_t lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		device.Spin();
		vTaskDelayUntil(&lastWakeTime, MoteusMotionDevice::UpdateIntervalMillis);
	}
}

void MoteusMotionDevice::Spin() noexcept {
	MutexLocker lock(servoTableMutex);
	const Move& move = reprap.GetMove();
	const uint32_t when = StepTimer::GetMovementTimerTicks() + LeadClocks;		// segments use movement time, which stops while paused
	for (size_t i = 0; i < numServos; ++i) {
		Servo& s = servos[i];
		size_t drive;
		if (s.stepsPerRev <= 0.0 || !move.GetLogicalDriveOfDriver(s.driver, drive)) {
			s.hasTarget = false;							// not configured yet: nothing to follow
			continue;
		}
		float steps, stepsPerClock;
		move.GetPlannedMotion(drive, when, steps, stepsPerClock);
		const float sign = (s.reverse) ? -1.0 : 1.0;
		s.targetRev = sign * steps/s.stepsPerRev;							// rev = steps / U
		s.targetRevPerSec = sign * stepsPerClock * (float)StepClockRate/s.stepsPerRev;	// rev/s = steps per clock × clocks per second / U
		s.hasTarget = true;
	}

	// Send every servo with an ID its frame, then collect the replies. For now the frame is a query only; step 7 sends position commands.
	CanTransport *_ecv_null const bus = SecondaryCanBus::GetTransport();
	if (bus == nullptr) {
		return;												// the secondary bus isn't running
	}
	CanMessageBuffer buf;
	while (bus->Receive(CanRxQueue::fifo0, 0, nullptr)) { }		// discard late replies from the last cycle so we don't match them to this one

	size_t expected = 0;
	for (size_t i = 0; i < numServos; ++i) {
		Servo& s = servos[i];
		s.replied = false;
		if (s.servoId != 0 && bus->IsSpaceAvailable(CanTxSlot::fifo, 0) && Moteus::MakeQuery(buf, s.servoId)) {
			(void)bus->Send(CanTxSlot::fifo, 0, &buf);
			++expected;
		}
	}

	const uint32_t collectStart = StepTimer::GetTimerTicks();
	while (expected != 0 && StepTimer::GetTimerTicks() - collectStart < ReplyWindowClocks && bus->Receive(CanRxQueue::fifo0, 1, &buf)) {
		for (size_t i = 0; i < numServos; ++i) {
			Servo& s = servos[i];
			if (!s.replied && s.servoId != 0 && Moteus::ParseReply(buf, s.servoId, s.state)) {
				s.replied = true;
				--expected;
				break;
			}
		}
	}
}

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
	if (!moteusTask.IsRunning()) {
		servoTableMutex.Create("MoteusServos");
		moteusTask.Create(MoteusTaskLoop, "Moteus", this, TaskPriority::MoteusPriority);	// first servo: start streaming
	}

	MutexLocker lock(servoTableMutex);
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
	if (!moteusTask.IsRunning()) {
		return;											// nothing has ever been attached, so the mutex doesn't exist yet
	}
	MutexLocker lock(servoTableMutex);
	Servo *_ecv_null const s = Find(driver);
	if (s != nullptr) {
		*s = servos[numServos - 1];						// order doesn't matter: move the last entry into the gap
		--numServos;
	}
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
	MutexLocker lock(servoTableMutex);					// we change the settings Spin reads, and report its latest sample

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
		if (s->hasTarget) {
			reply.catf(", planned %.3f rev at %.3f rev/s", (double)s->targetRev, (double)s->targetRevPerSec);
		}
		if (s->replied) {
			reply.catf(", servo at %.3f rev, mode %u, fault %u", (double)s->state.position, s->state.mode, s->state.fault);
		} else if (s->servoId != 0) {
			reply.cat(", no reply");
		}
	}
	return GCodeResult::ok;
}

bool MoteusMotionDevice::GetDirection(const Move&, DriverId driver) const noexcept {
	for (size_t i = 0; i < numServos; ++i) {
		if (servos[i].driver == driver) {
			return !servos[i].reverse;
		}
	}
	return true;										// not one of ours (can't happen through ResolveDriver): report forwards
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
