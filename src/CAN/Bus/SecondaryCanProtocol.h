/*
 * SecondaryCanProtocol.h
 *
 * Selects the protocol that runs on the secondary CAN bus. This is a build-time choice because the CAN controller's
 * memory layout (and whether it runs CAN-FD) depends on the largest frame the protocol uses.
 *
 * Set SECONDARY_CAN_PROTOCOL to one of the values below to choose the protocol; the default is moteus.
 */

#ifndef SRC_CAN_BUS_SECONDARYCANPROTOCOL_H_
#define SRC_CAN_BUS_SECONDARYCANPROTOCOL_H_

#include "Protocols/ODriveProtocol.h"
#include "Protocols/MoteusProtocol.h"

#if SUPPORT_CAN_EXPANSION && DUAL_CAN

#define SECONDARY_CAN_PROTOCOL_ODRIVE	1
#define SECONDARY_CAN_PROTOCOL_MOTEUS	2

#ifndef SECONDARY_CAN_PROTOCOL
# define SECONDARY_CAN_PROTOCOL			SECONDARY_CAN_PROTOCOL_MOTEUS
#endif

#if SECONDARY_CAN_PROTOCOL == SECONDARY_CAN_PROTOCOL_MOTEUS
typedef MoteusProtocol SecondaryCanProtocol;
#elif SECONDARY_CAN_PROTOCOL == SECONDARY_CAN_PROTOCOL_ODRIVE
typedef ODriveProtocol SecondaryCanProtocol;
#else
# error "Unknown SECONDARY_CAN_PROTOCOL"
#endif

#endif	// SUPPORT_CAN_EXPANSION && DUAL_CAN

#endif /* SRC_CAN_BUS_SECONDARYCANPROTOCOL_H_ */
