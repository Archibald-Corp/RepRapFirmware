// MotionDeviceFactory.cpp — owns the backend singletons and the resolution logic. Implementation.
#include "MotionDeviceFactory.h"
#include "LocalMotionDevice.h"
#include "CanMotionDevice.h"
#include "MoteusMotionDevice.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

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
        void AddAxisMovement(const PrepParams& params, DriverId driver, int32_t delta) noexcept override {}
        void AddExtruderMovement(const PrepParams& params, DriverId driver, motioncalc_t delta, bool usePressureAdvance) noexcept override {}
        void EnableDriver(Move&, const CanDriversList& drivers, float) noexcept override {}
        void DisableDriver(Move&, const CanDriversList& drivers) noexcept override {}
        GCodeResult ConfigureDriver(Move&, DriverId, GCodeBuffer&, const StringRef& reply) THROWS(GCodeException) override {
            reply.copy("driver is not assigned to a supported motion backend");
            return GCodeResult::error;
        }
        bool StopDriver (DriverId, int32_t, bool) noexcept override { return false; }
    };
}

static LocalMotionDevice localDevice;
static NullMotionDevice nullDevice;
#if SUPPORT_CAN_EXPANSION
static CanMotionDevice canDevice;
#endif
#if SUPPORT_MOTEUS
static MoteusMotionDevice moteusDevice;
#endif

// Drivers handed to a backend by M569.10. Any driver not listed uses its default backend (DefaultDevice).
struct DriverBinding {
    DriverId driver;
    MotionDeviceType type;
};

constexpr size_t MaxDriverBindings = 16;
static DriverBinding bindings[MaxDriverBindings];
static size_t numBindings = 0;

static DriverBinding *_ecv_null FindBinding(DriverId driver) noexcept {
    for (size_t i = 0; i < numBindings; ++i) {
        if (bindings[i].driver == driver) {
            return &bindings[i];
        }
    }
    return nullptr;
}

// The backend a driver uses when it isn't bound: this board's own drivers are local, any other is a Duet CAN expansion board
static MotionDeviceInterface& DefaultDevice(DriverId driver) noexcept {
#if SUPPORT_CAN_EXPANSION
    if (driver.IsRemote()) {
        return canDevice;
    }
#else
    (void)driver;
#endif
    return localDevice;
}

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
#if SUPPORT_MOTEUS
            return moteusDevice;
#else
            return nullDevice;          // moteus isn't built into this firmware: fail safe
#endif
        case MotionDeviceType::none:
        default:
            return nullDevice;
    }
}

MotionDeviceInterface& ResolveDriver(DriverId driver) noexcept {
    const DriverBinding *_ecv_null const binding = FindBinding(driver);
    return (binding != nullptr) ? GetDevice(binding->type) : DefaultDevice(driver);
}

void ForEachDevice(function_ref_noexcept<void(MotionDeviceInterface&) noexcept> func) noexcept {
    func(localDevice);
#if SUPPORT_CAN_EXPANSION
    func(canDevice);
#endif
#if SUPPORT_MOTEUS
    func(moteusDevice);
#endif
}

// M569.10 P<driver> B"<backend>": hand a driver number to a backend, e.g. B"moteus".
// Naming the driver's default backend (B"local" or B"can") removes the binding. Without B, report the driver's backend.
GCodeResult ConfigureBinding(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) {
    gb.MustSee('P');
    const DriverId driver = gb.GetDriverId();

    String<StringLength20> name;
    bool seen = false;
    if (!gb.TryGetQuotedString('B', name.GetRef(), seen)) {
#if SUPPORT_CAN_EXPANSION
        reply.printf("Driver %u.%u uses the %s backend", driver.boardAddress, driver.localDriver, ResolveDriver(driver).GetName());
#else
        reply.printf("Driver %u uses the %s backend", driver.localDriver, ResolveDriver(driver).GetName());
#endif
        return GCodeResult::ok;
    }

    MotionDeviceInterface *_ecv_null target = nullptr;
    ForEachDevice([&target, &name](MotionDeviceInterface& dev) noexcept {
        if (StringEqualsIgnoreCase(dev.GetName(), name.c_str())) {
            target = &dev;
        }
    });
    if (target == nullptr) {
        reply.printf("unknown motion backend \"%s\"", name.c_str());
        return GCodeResult::error;
    }

    DriverBinding *_ecv_null const binding = FindBinding(driver);
    MotionDeviceInterface& current = (binding != nullptr) ? GetDevice(binding->type) : DefaultDevice(driver);
    if (target == &current) {
        return GCodeResult::ok;                                     // already uses that backend
    }

    if (target == &DefaultDevice(driver)) {
        // Back to the default backend: remove the binding. (binding isn't null here: an unbound driver's current backend is its default.)
        current.DetachDriver(driver);
        *binding = bindings[numBindings - 1];
        --numBindings;
        return GCodeResult::ok;
    }

    if (binding == nullptr && numBindings == MaxDriverBindings) {
        reply.copy("too many driver bindings");
        return GCodeResult::error;
    }

    const GCodeResult rslt = target->AttachDriver(driver, reply);
    if (rslt > GCodeResult::warning) {
        return rslt;                                                // the backend refused; nothing changes
    }

    if (binding == nullptr) {
        bindings[numBindings].driver = driver;
        bindings[numBindings].type = target->GetType();
        ++numBindings;
    } else {
        current.DetachDriver(driver);                               // moving from one non-default backend to another
        binding->type = target->GetType();
    }
    return rslt;
}

}	// namespace MotionDeviceFactory
