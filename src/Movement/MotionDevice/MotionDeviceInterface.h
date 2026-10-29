// MotionDeviceInterface.h — abstract interface common to all motion backends (the refactor seam).
#ifndef SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEINTERFACE_H_
#define SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEINTERFACE_H_
#include <RepRapFirmware.h>
#include <Movement/DDA.h>
#include <CAN/CanDriversData.h>

//forward declarations
class Move;
class DDA;
class GCodeBuffer;

enum class MotionDeviceType : uint8_t {
	none = 0,
	local = 1,
	can = 2,
	moteus = 3 // mjbots moteus servos on the secondary CAN bus (MoteusMotionDevice)
};

class MotionDeviceInterface {
public:
	//default constructor
	MotionDeviceInterface() = default;

	//disable copying since only one backend and we only want pass by reference for dynamic allocation
	MotionDeviceInterface(const MotionDeviceInterface&) = delete;

	//destructor
	virtual ~MotionDeviceInterface() = default;

	//methods
	virtual MotionDeviceType GetType() const noexcept = 0;																						// which backend kind
	virtual const char *_ecv_array GetName() const noexcept = 0;																				// name for diagnostics
	virtual void StartMovement() noexcept {};																									// begin a move (default: nothing)
	virtual void AddAxisMovement(const PrepParams& params, DriverId driver, int32_t delta) noexcept = 0;										// execute one axis's motion | execution only tracking is shared 
	virtual void AddExtruderMovement(const PrepParams& params, DriverId driver, motioncalc_t delta, bool usePressureAdvance) noexcept = 0;		// execute one extruder's motion | execution only tracking is shared
	virtual uint32_t FinishMovement (const DDA&, uint32_t moveStartTime, bool simulating) noexcept { return 0; };								// commit move, return duration in clocks
	virtual void EnableDriver (Move&, const CanDriversList& drivers, float requiredCurrent) noexcept = 0;																// energize a motor
	virtual void DisableDriver (Move&, const CanDriversList& drivers) noexcept = 0;																					// de-energize a motor
	virtual GCodeResult ConfigureDriver(Move&, DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException) = 0;						// handle M569 for this driver
	virtual bool StopDriver (DriverId driver, int32_t netStepsTaken, bool executingMove) noexcept = 0;											// stop a motor mid-move (homing) we only deal with remote drives through the interface
	virtual bool GetDirection(const Move& move, DriverId driver) const noexcept = 0;								// true = forwards (M569 S1)
	virtual bool SupportsIndependentMotorMoves() const noexcept { return false; }									// can move one motor of an axis on its own (leadscrew adjustment)
	virtual bool RevertStoppedDrivers() noexcept { return true; }																				//default is nothing to revert
	virtual GCodeResult AttachDriver(DriverId, const StringRef& reply) noexcept													// M569.10: take over a driver number (default: refuse)
	{
		reply.printf("the %s backend can't take over drivers", GetName());
		return GCodeResult::error;
	}
	virtual void DetachDriver(DriverId) noexcept { }																			// a driver this backend had was bound elsewhere (default: nothing)
};

#endif /*SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEINTERFACE_H_*/