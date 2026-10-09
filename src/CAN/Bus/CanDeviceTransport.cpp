/*
 * CanDeviceTransport.cpp
 *
 * CanTransport implementation backed by a CoreN2G CanDevice.
 */

#include "CanDeviceTransport.h"

#if SUPPORT_CAN_EXPANSION

// Map the transport's slots and queues to CanDevice buffers by name, so we don't depend on the numbering of the CanDevice enums.
// There is deliberately no default case, so the compiler warns if a slot or queue is added without a mapping.
static CanDevice::TxBufferNumber ToDevice(CanTxSlot slot) noexcept
{
	switch (slot)
	{
	case CanTxSlot::fifo:	return CanDevice::TxBufferNumber::fifo;
	case CanTxSlot::slot0:	return CanDevice::TxBufferNumber::buffer0;
	case CanTxSlot::slot1:	return CanDevice::TxBufferNumber::buffer1;
	case CanTxSlot::slot2:	return CanDevice::TxBufferNumber::buffer2;
	case CanTxSlot::slot3:	return CanDevice::TxBufferNumber::buffer3;
	case CanTxSlot::slot4:	return CanDevice::TxBufferNumber::buffer4;
	case CanTxSlot::slot5:	return CanDevice::TxBufferNumber::buffer5;
	}
	return CanDevice::TxBufferNumber::fifo;
}

static CanDevice::RxBufferNumber ToDevice(CanRxQueue queue) noexcept
{
	switch (queue)
	{
	case CanRxQueue::fifo0:	return CanDevice::RxBufferNumber::fifo0;
	case CanRxQueue::fifo1:	return CanDevice::RxBufferNumber::fifo1;
	}
	return CanDevice::RxBufferNumber::fifo0;
}

bool CanDeviceTransport::Init(const CanTiming& timing) noexcept
{
	dev = CanDevice::Init(whichCan, whichPort, config, memory, timing, nullptr);
	return dev != nullptr;
}

void CanDeviceTransport::DeInit() noexcept
{
	if (dev != nullptr)
	{
		dev->DeInit();
		dev = nullptr;
	}
}

void CanDeviceTransport::Enable() noexcept
{
	dev->Enable();
}

void CanDeviceTransport::Disable() noexcept
{
	dev->Disable();
}

uint32_t CanDeviceTransport::Send(CanTxSlot slot, uint32_t timeout, CanMessageBuffer *buf) noexcept
{
	return dev->SendMessage(ToDevice(slot), timeout, buf);
}

bool CanDeviceTransport::Receive(CanRxQueue queue, uint32_t timeout, CanMessageBuffer *_ecv_null buf) noexcept
{
	return dev->ReceiveMessage(ToDevice(queue), timeout, buf);
}

bool CanDeviceTransport::IsSpaceAvailable(CanTxSlot slot, uint32_t timeout) noexcept
{
	return dev->IsSpaceAvailable(ToDevice(slot), timeout);
}

void CanDeviceTransport::PollTxEvents(CanTxEventCallback callback) noexcept
{
	dev->PollTxEventFifo(callback);
}

void CanDeviceTransport::SetShortFilter(unsigned int index, CanRxQueue queue, uint32_t id, uint32_t mask) noexcept
{
	dev->SetShortFilterElement(index, ToDevice(queue), id, mask);
}

void CanDeviceTransport::SetExtendedFilter(unsigned int index, CanRxQueue queue, uint32_t id, uint32_t mask) noexcept
{
	dev->SetExtendedFilterElement(index, ToDevice(queue), id, mask);
}

void CanDeviceTransport::GetTiming(CanTiming& timing) const noexcept
{
	dev->GetLocalCanTiming(timing);
}

void CanDeviceTransport::ChangeTiming(const CanTiming& timing) noexcept
{
	dev->ChangeLocalCanTiming(timing);
}

void CanDeviceTransport::GetAndClearStats(CanTransportStats& stats) noexcept
{
	CanDevice::CanStats local;
	dev->GetAndClearStats(local);
	stats.messagesQueuedForSending = local.messagesQueuedForSending;
	stats.messagesReceived = local.messagesReceived;
	stats.messagesLost = local.messagesLost;
	stats.protocolErrors = local.protocolErrors;
	stats.busOffCount = local.busOffCount;
}

uint16_t CanDeviceTransport::ReadTimeStampCounter() const noexcept
{
	return dev->ReadTimeStampCounter();
}

#if !SAME70

uint16_t CanDeviceTransport::GetTimeStampPeriod() const noexcept
{
	return dev->GetTimeStampPeriod();
}

#endif

#endif	// SUPPORT_CAN_EXPANSION
