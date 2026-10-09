/*
 * SecondaryCanBus.h
 *
 * Owner of the second CAN bus on boards that have one. The bus is not tied to any protocol: it is started with
 * whichever CanBusProtocol is given to Init, using that protocol's timing and filters.
 */

#ifndef SRC_CAN_BUS_SECONDARYCANBUS_H_
#define SRC_CAN_BUS_SECONDARYCANBUS_H_

#include "Protocols/CanBusProtocol.h"

#if SUPPORT_CAN_EXPANSION && DUAL_CAN

namespace SecondaryCanBus
{
	bool Init(const CanBusProtocol& protocol) noexcept;										// start the bus for this protocol, returning true if successful
	CanTransport *_ecv_null GetTransport() noexcept;											// the bus, or null if it is not running
	const CanBusProtocol *_ecv_null GetProtocol() noexcept;									// the protocol the bus was started for, or null if it is not running
}

#endif	// SUPPORT_CAN_EXPANSION && DUAL_CAN

#endif /* SRC_CAN_BUS_SECONDARYCANBUS_H_ */
