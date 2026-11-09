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
#include <Platform/Event.h>
#include <GCodes/GCodes.h>

// Limits for the M569 settings
constexpr float MinStepsPerRev = 1.0;
constexpr float MaxStepsPerRev = 1000000.0;			// beyond this, float positions lose whole-step resolution over long moves
constexpr float MaxTorqueLimit = 100.0;				// N.m; generous ceiling, the real limit is set per motor with T

// Limits on how long a stop can wait, so a dead bus or a stuck task can't hold it up
constexpr uint32_t EmergencyLockTimeoutMillis = 10;
constexpr uint32_t StopSendTimeoutMillis = 2;			// for each servo, while waiting for room in the transmit FIFO

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
		const bool frozen = s.frozen;						// read once: the step interrupt can set it at any time
		if (s.hasTarget && (frozen || positionSets != s.positionSets)) {
			s.offsetRev -= newTargetRev - s.targetRev;		// RRF's position changed (G92, homing) or this servo's endstop stopped it, so keep the command the same
		}
		s.positionSets = positionSets;
		s.targetRev = newTargetRev;
		s.targetRevPerSec = (frozen) ? 0.0 : sign * stepsPerClock * (float)StepClockRate/s.stepsPerRev;	// rev/s = steps per clock × clocks per second / U
		s.hasTarget = true;
	}

	// Send every servo with an ID its frame, then collect the replies. MakeFrame chooses a stop, a position command or a query.
	CanTransport *_ecv_null const bus = SecondaryCanBus::GetTransport();
	if (bus == nullptr) {
		return;												// the secondary bus isn't running
	}
	CanMessageBuffer buf;
	while (bus->Receive(CanRxQueue::fifo0, 0, nullptr)) { }		// discard late replies from the last cycle so we don't match them to this one

	Failure failures[MaxServos];
	size_t numFailures = 0;
	size_t expected = 0;
	for (size_t i = 0; i < numServos; ++i) {
		Servo& s = servos[i];
		if (s.awaitingReply) {
			s.replied = false;								// it didn't answer the last frame in time
			++s.missedReplies;
		}
		s.awaitingReply = false;
		if (CheckHealth(s, failures[numFailures])) {
			++numFailures;
		}
		if (s.servoId != 0 && bus->IsSpaceAvailable(CanTxSlot::fifo, 0) && MakeFrame(buf, s)) {
			(void)bus->Send(CanTxSlot::fifo, 0, &buf);
			s.awaitingReply = true;
			++expected;
		} else {
			s.replied = false;								// nothing went out, so no answer is coming
			++s.missedReplies;								// a full FIFO means frames aren't getting out, which counts as missing
		}
	}
	updateIntervalMillis = max<uint32_t>(MinUpdateIntervalMillis, expected);	// each servo uses about 0.75 ms of bus time per cycle, so allow 1 ms
	lock.Release();											// don't hold the table while we sleep, or the main task could wait forever for it
	for (size_t i = 0; i < numFailures; ++i) {
		ReportFailure(failures[i]);
	}

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
				s.missedReplies = 0;
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
	if (s.enabled && s.hasTarget && !s.needsAlign && s.maxTorque > 0.0) {
		return Moteus::MakePosition(buf, s.servoId, s.targetRev + s.offsetRev, s.targetRevPerSec, s.maxTorque, true);
	}
	return Moteus::MakeQuery(buf, s.servoId);
}

// Called once per cycle for each servo, before its next frame, with the table locked. If an enabled servo has faulted or stopped
// answering, turn it off, fill in failure and return true. It stays off until M17, so it can't jump back into position on its own.
bool MoteusMotionDevice::CheckHealth(Servo& s, Failure& failure) noexcept {
	if (!s.enabled || s.servoId == 0) {
		s.missedReplies = 0;							// only an enabled servo with an ID is expected to answer
		return false;
	}
	const bool faulted = s.replied && (s.state.mode == (uint8_t)Moteus::Mode::fault || s.state.mode == (uint8_t)Moteus::Mode::timeout);
	if (!faulted && s.missedReplies < MaxMissedReplies) {
		return false;
	}
	s.enabled = false;
	s.stopPending = true;
	failure.driver = s.driver;
	failure.drive = (s.hasTarget) ? s.drive : MaxAxesPlusExtruders;
	failure.servoId = s.servoId;
	failure.fault = s.state.fault;
	failure.notAnswering = !faulted;
	return true;
}

// Report a servo that CheckHealth turned off. The event pauses a print and shows the message. Called without the table lock, because this takes other locks.
void MoteusMotionDevice::ReportFailure(const Failure& failure) noexcept {
	GCodes& gc = reprap.GetGCodes();
	if (failure.drive < gc.GetTotalAxes()) {
		gc.SetAxisNotHomed(failure.drive);				// the plan may have moved on without it, so its position can't be trusted
	}
	const uint16_t status = (uint16_t)(1u << StandardDriverStatus::ExternDriverErrorBitPos);	// RRF reads the event parameter as driver status bits
	if (failure.notAnswering) {
		(void)Event::AddEvent(EventType::driver_error, status, failure.driver.boardAddress, failure.driver.localDriver,
								", moteus servo %u not answering", (unsigned int)failure.servoId);
	} else {
		(void)Event::AddEvent(EventType::driver_error, status, failure.driver.boardAddress, failure.driver.localDriver,
								", moteus servo %u fault %u", (unsigned int)failure.servoId, (unsigned int)failure.fault);
	}
}

// Send a stop straight away, without going through Spin. The caller holds the servo table lock.
void MoteusMotionDevice::SendStopNow(CanTransport& bus, const Servo& s) noexcept {
	CanMessageBuffer buf;
	if (s.servoId != 0 && bus.IsSpaceAvailable(CanTxSlot::fifo, StopSendTimeoutMillis) && Moteus::MakeStop(buf, s.servoId, false)) {
		(void)bus.Send(CanTxSlot::fifo, 0, &buf);
	}
}

MotionDeviceType MoteusMotionDevice::GetType() const noexcept {
	return MotionDeviceType::moteus;
}

const char *_ecv_array MoteusMotionDevice::GetName() const noexcept {
	return "moteus";
}

// Called when a move is being prepared. Endstop moves are isolated, so any move that froze a servo has ended by now, even if it was aborted.
// CheckEndstops runs after this during preparation, so an endstop that is already triggered freezes its servo again.
void MoteusMotionDevice::StartMovement() noexcept {
	for (size_t i = 0; i < numServos; ++i) {
		servos[i].frozen = false;
	}
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
			s->stopPending = true;						// clear any fault or timeout left from before (e.g. a Duet reset), so CheckHealth sees a fresh state
			s->needsAlign = true;						// it may have been turned by hand while it was off
			s->replied = false;							// CheckHealth must only judge replies to frames sent from now on...
			s->awaitingReply = false;					// ...so a late reply to an older frame isn't taken as one
			s->missedReplies = 0;
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

// Called when this servo's own endstop triggers but others on the axis haven't. This can be in the step interrupt, so no mutex.
// A single endstop on an axis doesn't come here: RRF stops the plan, and the servo follows it.
bool MoteusMotionDevice::StopDriver(DriverId driver, int32_t, bool) noexcept {
	Servo *_ecv_null const s = Find(driver);
	if (s != nullptr) {
		s->frozen = true;
	}
	return false;										// nothing for the Duet CAN sender to do
}

// Called by RepRap::EmergencyStop. Sends the stops from here rather than through Spin, so they don't depend on the task being healthy.
void MoteusMotionDevice::EmergencyStop() noexcept {
	if (!moteusTask.IsRunning()) {
		return;											// no servo was bound, or this is a repeat of M112
	}
	MutexLocker lock(servoTableMutex, EmergencyLockTimeoutMillis);	// so the task can't end while holding the table; if it's stuck holding it, carry on anyway
	moteusTask.TerminateAndUnlink();					// nothing can send a command after the stops
	CanTransport *_ecv_null const bus = SecondaryCanBus::GetTransport();
	if (bus == nullptr) {
		return;											// the secondary bus isn't running
	}
	for (size_t i = 0; i < numServos; ++i) {
		Servo& s = servos[i];
		s.enabled = false;
		s.stopPending = false;
		SendStopNow(*bus, s);
	}
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
		CanTransport *_ecv_null const bus = SecondaryCanBus::GetTransport();
		if (s->enabled && bus != nullptr) {
			SendStopNow(*bus, *s);						// once it's off the table, Spin can't stop it
		}
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
		if (s->enabled && s->maxTorque <= 0.0) {
			reply.cat(" (T not set, so no position commands)");
		}
		if (s->frozen) {
			reply.cat(", stopped by endstop");
		}
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
