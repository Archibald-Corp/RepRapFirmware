/*
 * SecondaryCanBus.cpp
 *
 * Owner of the second CAN bus on boards that have one.
 */

#include "SecondaryCanBus.h"

#if SUPPORT_CAN_EXPANSION && DUAL_CAN

#include "CanDeviceTransport.h"
#include "SecondaryCanProtocol.h"

// The memory layout of the CAN controller is fixed at compile time, so dataSize is taken from the protocol selected for this bus.
// CanDevice runs the controller in CAN-FD mode when dataSize is greater than 8.
constexpr CanDevice::Config Can1Config =
{
	.dataSize = SecondaryCanProtocol::MaxDataLength,
	.numTxBuffers = 2,
	.txFifoSize = 4,
	.numRxBuffers =  0,
	.rxFifo0Size = 16,
	.rxFifo1Size = 16,
	.numShortFilterElements = 1,
	.numExtendedFilterElements = 1,
	.txEventFifoSize = 16
};

static_assert(Can1Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can1Memory[Can1Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

static CanDeviceTransport can1Transport(1, SecondaryCanDeviceNumber, Can1Config, can1Memory);
static const CanBusProtocol *_ecv_null currentProtocol = nullptr;

bool SecondaryCanBus::Init(const CanBusProtocol& protocol) noexcept
{
	if (protocol.GetMaxDataLength() > Can1Config.dataSize)
	{
		return false;							// the controller memory was not laid out for frames this large
	}

	CanTiming timing;
	protocol.GetTiming(timing);
	if (!can1Transport.Init(timing))
	{
		return false;
	}

	protocol.SetupFilters(can1Transport);
	can1Transport.Enable();
	currentProtocol = &protocol;
	return true;
}

CanTransport *_ecv_null SecondaryCanBus::GetTransport() noexcept
{
	return (can1Transport.IsActive()) ? &can1Transport : nullptr;
}

const CanBusProtocol *_ecv_null SecondaryCanBus::GetProtocol() noexcept
{
	return (can1Transport.IsActive()) ? currentProtocol : nullptr;
}

#endif	// SUPPORT_CAN_EXPANSION && DUAL_CAN
