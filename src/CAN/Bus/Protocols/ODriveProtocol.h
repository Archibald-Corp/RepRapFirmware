/*
 * ODriveProtocol.h
 *
 * Bus settings for ODrive motor controllers using the CANSimple protocol: classic CAN with 11-bit IDs.
 */

#ifndef SRC_CAN_BUS_PROTOCOLS_ODRIVEPROTOCOL_H_
#define SRC_CAN_BUS_PROTOCOLS_ODRIVEPROTOCOL_H_

#include "CanBusProtocol.h"

#if SUPPORT_CAN_EXPANSION && DUAL_CAN

class ODriveProtocol final : public CanBusProtocol
{
public:
	static constexpr uint32_t BitRate = 250'000;
	static constexpr size_t MaxDataLength = 8;									// classic CAN

	const char *_ecv_array GetName() const noexcept override { return "ODrive"; }
	void GetTiming(CanTiming& timing) const noexcept override;
	size_t GetMaxDataLength() const noexcept override { return MaxDataLength; }
	void SetupFilters(CanTransport& bus) const noexcept override;
};

#endif	// SUPPORT_CAN_EXPANSION && DUAL_CAN

#endif /* SRC_CAN_BUS_PROTOCOLS_ODRIVEPROTOCOL_H_ */
