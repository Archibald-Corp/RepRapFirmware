/*
 * CanTransport.h
 */

#ifndef SRC_CAN_CANTRANSPORT_H_
#define SRC_CAN_CANTRANSPORT_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

#include <CanId.h>
#include <CanSettings.h>					// for CanTiming

class CanMessageBuffer;

// Transmit slots and receive queues. Each transport implementation maps these to its own hardware buffers.
enum class CanTxSlot : uint8_t { fifo, slot0, slot1, slot2, slot3, slot4, slot5 };
enum class CanRxQueue : uint8_t { fifo0, fifo1 };

struct CanTransportStats
{
	unsigned int messagesQueuedForSending;
	unsigned int messagesReceived;
	unsigned int messagesLost;					// count of received messages lost because the receive FIFO was full
	unsigned int protocolErrors;
	unsigned int busOffCount;					// count of the number of times we have reset due to bus off
};

// Called for each transmit event whose message marker is nonzero
typedef void (*CanTxEventCallback)(uint8_t marker, CanId id, uint16_t timeStamp) noexcept;

class CanTransport
{
public:
	CanTransport() noexcept = default;
	CanTransport(const CanTransport&) = delete;
	virtual ~CanTransport() = default;

	// Lifecycle
	virtual bool Init(const CanTiming& timing) noexcept = 0;													// initialise the hardware, returning true if successful
	virtual void DeInit() noexcept = 0;																			// release the hardware; safe to call when not initialised
	virtual bool IsActive() const noexcept = 0;																	// true between a successful Init and DeInit
	virtual void Enable() noexcept = 0;
	virtual void Disable() noexcept = 0;

	// Sending and receiving
	virtual uint32_t Send(CanTxSlot slot, uint32_t timeout, CanMessageBuffer *buf) noexcept = 0;				// returns the ID of any message cancelled to make room, else 0
	virtual bool Receive(CanRxQueue queue, uint32_t timeout, CanMessageBuffer *_ecv_null buf) noexcept = 0;	// returns true if a message was received; null buf discards it
	virtual bool IsSpaceAvailable(CanTxSlot slot, uint32_t timeout) noexcept = 0;
	virtual void PollTxEvents(CanTxEventCallback callback) noexcept = 0;

	// Receive filters. Only valid while the transport is disabled.
	virtual void SetShortFilter(unsigned int index, CanRxQueue queue, uint32_t id, uint32_t mask) noexcept = 0;
	virtual void SetExtendedFilter(unsigned int index, CanRxQueue queue, uint32_t id, uint32_t mask) noexcept = 0;

	// Bit timing
	virtual void GetTiming(CanTiming& timing) const noexcept = 0;
	virtual void ChangeTiming(const CanTiming& timing) noexcept = 0;

	// Diagnostics and time stamps
	virtual void GetAndClearStats(CanTransportStats& stats) noexcept = 0;
	virtual uint16_t ReadTimeStampCounter() const noexcept = 0;
#if !SAME70
	virtual uint16_t GetTimeStampPeriod() const noexcept = 0;													// period of the time stamp counter in units of 48MHz CAN clocks
#endif
};

#endif	// SUPPORT_CAN_EXPANSION

#endif /* SRC_CAN_CANTRANSPORT_H_ */
