/*
 * ODriveProtocol.cpp
 *
 * Bus settings for ODrive motor controllers.
 */

#include "ODriveProtocol.h"

#if SUPPORT_CAN_EXPANSION && DUAL_CAN

void ODriveProtocol::GetTiming(CanTiming& timing) const noexcept
{
	timing.SetDefaults(BitRate);
}

void ODriveProtocol::SetupFilters(CanTransport& bus) const noexcept
{
	// Receive all messages in FIFO 0. Replies are matched against the expected ID by the code that sent the request.
	bus.SetShortFilter(0, CanRxQueue::fifo0, 0, 0);
	bus.SetExtendedFilter(0, CanRxQueue::fifo0, 0, 0);
}

#endif	// SUPPORT_CAN_EXPANSION && DUAL_CAN
