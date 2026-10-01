/*
 * CanDeviceTransport.h
 *
 * CanTransport implementation backed by a CoreN2G CanDevice (the MCU's own CAN controller).
 */

#ifndef SRC_CAN_CANDEVICETRANSPORT_H_
#define SRC_CAN_CANDEVICETRANSPORT_H_

#include "CanTransport.h"

#if SUPPORT_CAN_EXPANSION

#ifndef SUPPORT_CAN
# define SUPPORT_CAN		1				// needed by CanDevice.h
#endif
#include <CanDevice.h>

class CanDeviceTransport final : public CanTransport
{
public:
	// The config and memory must outlive this object, because CanDevice keeps a reference to the config.
	// No hardware is touched until Init is called, so instances may be file-static.
	constexpr CanDeviceTransport(unsigned int p_whichCan, unsigned int p_whichPort, const CanDevice::Config& p_config, uint32_t *_ecv_array p_memory) noexcept
		: whichCan(p_whichCan), whichPort(p_whichPort), config(p_config), memory(p_memory), dev(nullptr)
	{
	}

	// Lifecycle
	bool Init(const CanTiming& timing) noexcept override;
	void DeInit() noexcept override;
	bool IsActive() const noexcept override { return dev != nullptr; }
	void Enable() noexcept override;
	void Disable() noexcept override;

	// Sending and receiving
	uint32_t Send(CanTxSlot slot, uint32_t timeout, CanMessageBuffer *buf) noexcept override;
	bool Receive(CanRxQueue queue, uint32_t timeout, CanMessageBuffer *_ecv_null buf) noexcept override;
	bool IsSpaceAvailable(CanTxSlot slot, uint32_t timeout) noexcept override;
	void PollTxEvents(CanTxEventCallback callback) noexcept override;

	// Receive filters
	void SetShortFilter(unsigned int index, CanRxQueue queue, uint32_t id, uint32_t mask) noexcept override;
	void SetExtendedFilter(unsigned int index, CanRxQueue queue, uint32_t id, uint32_t mask) noexcept override;

	// Bit timing
	void GetTiming(CanTiming& timing) const noexcept override;
	void ChangeTiming(const CanTiming& timing) noexcept override;

	// Diagnostics and time stamps
	void GetAndClearStats(CanTransportStats& stats) noexcept override;
	uint16_t ReadTimeStampCounter() const noexcept override;
#if !SAME70
	uint16_t GetTimeStampPeriod() const noexcept override;
#endif

private:
	const unsigned int whichCan;
	const unsigned int whichPort;
	const CanDevice::Config& config;
	uint32_t *_ecv_array const memory;
	CanDevice *_ecv_null dev;
};

#endif	// SUPPORT_CAN_EXPANSION

#endif /* SRC_CAN_CANDEVICETRANSPORT_H_ */
