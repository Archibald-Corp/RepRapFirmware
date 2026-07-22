// MotionDeviceFactory.cpp — owns the backend singletons and the resolution logic. Implementation.
#include "MotionDeviceFactory.h"
#include "LocalMotionDevice.h"
#include "CanMotionDevice.h"

// used to describe a backend error; anonymous to only be used in this file.
namespace
{
    class NullMotionDevice final : public MotionDeviceInterface
    {
    public:
        // StartMovement and FinishMovement are inherited from the interface defaults (no-op / return 0).
        MotionDeviceType GetType() const noexcept override {
            return MotionDeviceType::none;
        }
        const char *_ecv_array GetName() const noexcept override {
            return "none";
        }
        void AddAxisMovement(Move&, const PrepParams&, uint32_t, size_t, DriverId, int32_t, MovementFlags) noexcept override {}
        void AddExtruderMovement(Move&, const PrepParams&, uint32_t, size_t, DriverId, motioncalc_t, bool, MovementFlags) noexcept override {}
        void EnableDriver(Move&, DriverId, float) noexcept override {}
        void DisableDriver(Move&, DriverId) noexcept override {}
        GCodeResult ConfigureDriver(Move&, DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException) override {
            reply.copy("driver is not assigned to a supported motion backend");
            return GCodeResult::error;
        }
        void StopDriver(DriverId, int32_t) noexcept override {}
    };
}

static LocalMotionDevice localDevice;
static NullMotionDevice nullDevice;
#if SUPPORT_CAN_EXPANSION
static CanMotionDevice canDevice;
#endif

namespace MotionDeviceFactory
{

void Init() noexcept {
    //filled in later for backend binding for each drive
}

MotionDeviceInterface& GetDevice(MotionDeviceType device) noexcept {
    switch (device) {
        case MotionDeviceType::local:
            return localDevice;
#if SUPPORT_CAN_EXPANSION
        case MotionDeviceType::can:
            return canDevice;
#endif
        case MotionDeviceType::moteus:
            //TODO Moteus — declared but not functional, so it fails safe
            return nullDevice;
        case MotionDeviceType::none:
        default:
            return nullDevice;
    }
}

MotionDeviceInterface& ResolveDriver(DriverId driver) noexcept {
    //will replace with a binding table per drive in later sprint.
#if SUPPORT_CAN_EXPANSION
    if (driver.IsRemote()) {
        return GetDevice(MotionDeviceType::can);
    }
#else
    (void)driver;
#endif
    return GetDevice(MotionDeviceType::local);
}

}	// namespace MotionDeviceFactory
