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
static Mutex servoTableMutex;							// held by Spin while it uses the servo table, and by anything that changes the table

extern "C" [[noreturn]] void MoteusTaskLoop(void *param) noexcept
{
	MoteusMotionDevice& device = *static_cast<MoteusMotionDevice*>(param);
	uint32_t lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		device.Spin();
		vTaskDelayUntil(&lastWakeTime, device.GetUpdateIntervalMillis());
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
			s.needsAlign = true;							// when it comes back, the old offset won't apply
			continue;
		}
		float steps, stepsPerClock;
		uint32_t positionSets;
		move.GetPlannedMotion(drive, when, steps, stepsPerClock, positionSets);
		if (s.hasTarget && drive != s.drive) {
			s.needsAlign = true;							// M584 moved it to another drive, so the old offset doesn't apply
		}
		s.drive = drive;
		const float sign = (s.reverse) ? -1.0 : 1.0;
		const float newTargetRev = sign * steps/s.stepsPerRev;				// rev = steps / U
		if (s.hasTarget && positionSets != s.positionSets) {
			s.offsetRev -= newTargetRev - s.targetRev;		// only RRF's idea of the position changed (G92, homing), so keep the command the same
		}
		s.positionSets = positionSets;
		s.targetRev = newTargetRev;
		s.targetRevPerSec = sign * stepsPerClock * (float)StepClockRate/s.stepsPerRev;	// rev/s = steps per clock × clocks per second / U
		s.hasTarget = true;
	}

	// Send every servo with an ID its frame, then collect the replies. For now the frame is a query or a stop; stage 7.3 adds position commands.
	CanTransport *_ecv_null const bus = SecondaryCanBus::GetTransport();
	if (bus == nullptr) {
		return;												// the secondary bus isn't running
	}
	CanMessageBuffer buf;
	while (bus->Receive(CanRxQueue::fifo0, 0, nullptr)) { }		// discard late replies from the last cycle so we don't match them to this one

	size_t expected = 0;
	for (size_t i = 0; i < numServos; ++i) {
		Servo& s = servos[i];
		if (s.awaitingReply) {
			s.replied = false;								// it didn't answer the last frame in time
		}
		s.awaitingReply = false;
		if (s.servoId != 0 && bus->IsSpaceAvailable(CanTxSlot::fifo, 0) && MakeFrame(buf, s)) {
			(void)bus->Send(CanTxSlot::fifo, 0, &buf);
			s.awaitingReply = true;
			++expected;
		} else {
			s.replied = false;								// nothing went out, so no answer is coming
		}
	}
	updateIntervalMillis = max<uint32_t>(MinUpdateIntervalMillis, expected);	// each servo uses about 0.75 ms of bus time per cycle, so allow 1 ms
	lock.Release();											// don't hold the table while we sleep, or the main task could wait forever for it

	const uint32_t window = updateIntervalMillis * (StepClockRate/1000) - ReplyMarginClocks;
	const uint32_t collectStart = StepTimer::GetTimerTicks();
	while (expected != 0 && StepTimer::GetTimerTicks() - collectStart < window) {
		if (!bus->Receive(CanRxQueue::fifo0, 1, &buf)) {
			continue;										// a 1-tick wait can end early, so keep going until the window closes
		}
		lock.ReAcquire();
		for (size_t i = 0; i < numServos; ++i) {
			Servo& s = servos[i];
			if (s.awaitingReply && Moteus::ParseReply(buf, s.servoId, s.state)) {
				s.awaitingReply = false;
				s.replied = true;
				--expected;
				if (s.needsAlign && s.hasTarget && !std::isnan(s.state.position)) {
					s.offsetRev = s.state.position - s.targetRev;	// from here on, targetRev + offsetRev starts from where the servo is now
					s.needsAlign = false;
				}
				break;
			}
		}
		lock.Release();
	}
}

// Build this cycle's frame for a servo. Each frame also asks for the servo's state.
bool MoteusMotionDevice::MakeFrame(CanMessageBuffer& buf, Servo& s) noexcept {
	if (s.stopPending) {
		if (!Moteus::MakeStop(buf, s.servoId, true)) {
			return false;
		}
		s.stopPending = false;
		return true;
	}
	return Moteus::MakeQuery(buf, s.servoId);
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

// Called by M17 and whenever a move needs these drivers, so it often arrives for servos that are already enabled. requiredCurrent doesn't apply: T sets the torque.
void MoteusMotionDevice::EnableDriver(Move&, const CanDriversList& drivers, float) noexcept {
	if (!moteusTask.IsRunning()) {
		return;											// no servo has ever been bound, so the mutex doesn't exist yet
	}
	MutexLocker lock(servoTableMutex);
	for (size_t i = 0; i < drivers.GetNumEntries(); ++i) {
		Servo *_ecv_null const s = Find(drivers.GetEntry(i));
		if (s != nullptr && !s->enabled) {
			s->enabled = true;
			s->stopPending = false;
			s->needsAlign = true;						// it may have been turned by hand while it was off
		}
	}
}

// Called by M18, M84 and the idle timeout when the idle current is 0. Spin sends the stop, so this doesn't wait for the bus.
void MoteusMotionDevice::DisableDriver(Move&, const CanDriversList& drivers) noexcept {
	if (!moteusTask.IsRunning()) {
		return;											// no servo has ever been bound, so the mutex doesn't exist yet
	}
	MutexLocker lock(servoTableMutex);
	for (size_t i = 0; i < drivers.GetNumEntries(); ++i) {
		Servo *_ecv_null const s = Find(drivers.GetEntry(i));
		if (s != nullptr && s->enabled) {
			s->enabled = false;
			s->stopPending = true;
		}
	}
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
	if (SecondaryCanBus::GetTransport() == nullptr) {
		reply.copy("the second CAN bus isn't running, so this servo can't be reached");
		return GCodeResult::warning;
	}
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
		s->needsAlign = true;
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
		s->needsAlign = true;
	}

	float f;
	if (gb.TryGetLimitedFValue('U', f, seen, MinStepsPerRev, MaxStepsPerRev)) {
		s->stepsPerRev = f;
		s->needsAlign = true;
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
		reply.catf(", max torque %.2f N.m, %s", (double)s->maxTorque, (s->enabled) ? "enabled" : "disabled");
		if (s->hasTarget) {
			reply.catf(", planned %.3f rev at %.3f rev/s", (double)s->targetRev, (double)s->targetRevPerSec);
			if (!s->needsAlign) {
				reply.catf(", offset %.3f rev, command %.3f rev", (double)s->offsetRev, (double)(s->targetRev + s->offsetRev));
			}
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
