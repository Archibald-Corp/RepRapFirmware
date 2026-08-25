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
	moteus = 3 //will be implemented in a later sprint, used to test rejection
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
	virtual void StopDriver (DriverId, int32_t netStepsTaken) noexcept = 0;																		// stop a motor mid-move (homing)
	virtual bool RevertStoppedDrivers() noexcept { return true; }																				//default is nothing to revert
};

#endif /*SRC_MOVEMENT_MOTIONDEVICE_MOTIONDEVICEINTERFACE_H_*/