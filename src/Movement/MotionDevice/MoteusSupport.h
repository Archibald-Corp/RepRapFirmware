// MoteusSupport.h — decides whether the moteus motion backend is built into this firmware.
// moteus servos run on the secondary CAN bus, so this needs a board with two CAN buses and a build that
// selects the moteus protocol for that bus (see CAN/Bus/SecondaryCanProtocol.h).
#ifndef SRC_MOVEMENT_MOTIONDEVICE_MOTEUSSUPPORT_H_
#define SRC_MOVEMENT_MOTIONDEVICE_MOTEUSSUPPORT_H_

#include <RepRapFirmware.h>

// SECONDARY_CAN_PROTOCOL only exists on boards with two CAN buses, and testing an undefined macro
// fails this build (-Wundef with warnings as errors), so it is only tested inside the DUAL_CAN branch.
#if SUPPORT_CAN_EXPANSION && DUAL_CAN
# include <CAN/Bus/SecondaryCanProtocol.h>
# define SUPPORT_MOTEUS		(SECONDARY_CAN_PROTOCOL == SECONDARY_CAN_PROTOCOL_MOTEUS)
#else
# define SUPPORT_MOTEUS		0
#endif

#endif /* SRC_MOVEMENT_MOTIONDEVICE_MOTEUSSUPPORT_H_ */
