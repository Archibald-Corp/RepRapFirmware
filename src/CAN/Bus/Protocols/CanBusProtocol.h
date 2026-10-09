/*
 * CanBusProtocol.h
 *
 * What a protocol needs from the CAN bus it runs on. The bus owner asks the protocol for these settings when it starts the bus,
 * so the bus itself has no knowledge of which protocol is using it.
 */

#ifndef SRC_CAN_BUS_PROTOCOLS_CANBUSPROTOCOL_H_
#define SRC_CAN_BUS_PROTOCOLS_CANBUSPROTOCOL_H_

#include "../CanTransport.h"

#if SUPPORT_CAN_EXPANSION

class CanBusProtocol
{
public:
	CanBusProtocol() noexcept = default;
	CanBusProtocol(const CanBusProtocol&) = delete;
	virtual ~CanBusProtocol() = default;

	virtual const char *_ecv_array GetName() const noexcept = 0;									// name for diagnostics
	virtual void GetTiming(CanTiming& timing) const noexcept = 0;								// bit rate, sample point and whether to use bit rate switching
	virtual size_t GetMaxDataLength() const noexcept = 0;										// largest frame payload the protocol sends or receives: 8 for classic CAN, up to 64 for CAN-FD
	virtual void SetupFilters(CanTransport& bus) const noexcept = 0;							// set up the receive filters; called while the bus is disabled
};

#endif	// SUPPORT_CAN_EXPANSION

#endif /* SRC_CAN_BUS_PROTOCOLS_CANBUSPROTOCOL_H_ */
