/*
 * MoteusProtocol.cpp
 *
 * mjbots moteus motor controllers over CAN-FD.
 */

#include "MoteusProtocol.h"

#if SUPPORT_CAN_EXPANSION && DUAL_CAN

#include <CanMessageBuffer.h>
#include <General/Portability.h>
#include <limits>

// Subframe types. Read, write and reply types have the register count in their 2 LSBs, or 0 there if the count follows as a varuint.
namespace SubframeType
{
	constexpr uint8_t WriteInt8 = 0x00;
	constexpr uint8_t WriteFloat = 0x0c;
	constexpr uint8_t ReadInt8 = 0x10;
	constexpr uint8_t ReadFloat = 0x1c;
	constexpr uint8_t ReplyBase = 0x20;				// 0x20 int8, 0x24 int16, 0x28 int32, 0x2c float
	constexpr uint8_t ReplyInt8 = 0x20;
	constexpr uint8_t ReplyInt16 = 0x24;
	constexpr uint8_t ReplyInt32 = 0x28;
	constexpr uint8_t ReplyFloat = 0x2c;
	constexpr uint8_t WriteError = 0x30;
	constexpr uint8_t ReadError = 0x31;
	constexpr uint8_t Nop = 0x50;					// also used to pad frames to a valid CAN-FD length
}

constexpr uint32_t ReplyRequiredBit = 0x8000;
constexpr uint32_t MaxStandardId = 0x7FF;

void MoteusProtocol::GetTiming(CanTiming& timing) const noexcept
{
	timing.SetDefaults(BitRate);					// this also disables bit rate switching
	timing.SetNormalSamplePoint(SamplePoint);
}

void MoteusProtocol::SetupFilters(CanTransport& bus) const noexcept
{
	// Receive all messages in FIFO 0. Replies are matched against the servo and host IDs when they are parsed.
	bus.SetShortFilter(0, CanRxQueue::fifo0, 0, 0);
	bus.SetExtendedFilter(0, CanRxQueue::fifo0, 0, 0);
}

// Frame building

Moteus::FrameBuilder::FrameBuilder(CanMessageBuffer& p_buf, uint8_t p_servoId, uint8_t p_hostId) noexcept
	: buf(p_buf), length(0), servoId(p_servoId), hostId(p_hostId), replyRequired(false), overflowed(false)
{
}

bool Moteus::FrameBuilder::AddVaruint(uint32_t value) noexcept
{
	do
	{
		if (!HaveSpace(1))
		{
			overflowed = true;
			return false;
		}
		uint8_t b = value & 0x7F;
		value >>= 7;
		if (value != 0)
		{
			b |= 0x80;
		}
		buf.msg.raw[length++] = b;
	} while (value != 0);
	return true;
}

bool Moteus::FrameBuilder::AddHeader(uint8_t baseType, uint16_t startRegister, size_t count) noexcept
{
	if (count == 0 || !HaveSpace(1))
	{
		overflowed = true;
		return false;
	}
	if (count <= 3)
	{
		buf.msg.raw[length++] = baseType | (uint8_t)count;
	}
	else
	{
		buf.msg.raw[length++] = baseType;
		if (!AddVaruint(count))
		{
			return false;
		}
	}
	return AddVaruint(startRegister);
}

bool Moteus::FrameBuilder::WriteInt8(uint16_t startRegister, const int8_t *_ecv_array values, size_t count) noexcept
{
	if (!AddHeader(SubframeType::WriteInt8, startRegister, count) || !HaveSpace(count))
	{
		overflowed = true;
		return false;
	}
	for (size_t i = 0; i < count; ++i)
	{
		buf.msg.raw[length++] = (uint8_t)values[i];
	}
	return true;
}

bool Moteus::FrameBuilder::WriteFloat(uint16_t startRegister, const float *_ecv_array values, size_t count) noexcept
{
	if (!AddHeader(SubframeType::WriteFloat, startRegister, count) || !HaveSpace(count * sizeof(float)))
	{
		overflowed = true;
		return false;
	}
	for (size_t i = 0; i < count; ++i)
	{
		StoreLEF32(&buf.msg.raw[length], values[i]);
		length += sizeof(float);
	}
	return true;
}

bool Moteus::FrameBuilder::ReadInt8(uint16_t startRegister, size_t count) noexcept
{
	replyRequired = true;
	return AddHeader(SubframeType::ReadInt8, startRegister, count);
}

bool Moteus::FrameBuilder::ReadFloat(uint16_t startRegister, size_t count) noexcept
{
	replyRequired = true;
	return AddHeader(SubframeType::ReadFloat, startRegister, count);
}

// Return the smallest valid CAN-FD data length that holds 'length' bytes
static size_t PaddedLength(size_t length) noexcept
{
	if (length <= 8) { return length; }
	if (length <= 24) { return (length + 3) & ~(size_t)3; }			// 12, 16, 20, 24
	if (length <= 32) { return 32; }
	if (length <= 48) { return 48; }
	return 64;
}

void Moteus::FrameBuilder::Finish() noexcept
{
	const size_t paddedLength = PaddedLength(length);
	while (length < paddedLength)
	{
		buf.msg.raw[length++] = SubframeType::Nop;
	}

	const uint32_t id = (replyRequired ? ReplyRequiredBit : 0) | ((uint32_t)hostId << 8) | servoId;
	buf.id.SetReceivedId(id);
	buf.extId = (id > MaxStandardId);
	buf.dataLength = length;
	buf.fdMode = 1;
	buf.useBrs = 0;
	buf.remote = 0;
	buf.marker = 0;
	buf.reportInFifo = 0;
}

// Common frames

// Add reads of the registers that ServoState holds
static bool AddStateQuery(Moteus::FrameBuilder& frame) noexcept
{
	return frame.ReadInt8(Moteus::Register::Mode, 1)
		&& frame.ReadFloat(Moteus::Register::Position, 3)				// position, velocity, torque
		&& frame.ReadFloat(Moteus::Register::Voltage, 2)				// voltage, temperature
		&& frame.ReadInt8(Moteus::Register::Fault, 1);
}

bool Moteus::MakeStop(CanMessageBuffer& buf, uint8_t servoId, bool query) noexcept
{
	FrameBuilder frame(buf, servoId);
	const int8_t mode = (int8_t)Mode::stopped;
	if (!frame.WriteInt8(Register::Mode, &mode, 1) || (query && !AddStateQuery(frame)))
	{
		return false;
	}
	frame.Finish();
	return true;
}

bool Moteus::MakePosition(CanMessageBuffer& buf, uint8_t servoId, float position, float velocity, float maxTorque, bool query) noexcept
{
	FrameBuilder frame(buf, servoId);
	const int8_t mode = (int8_t)Mode::position;
	const float positionAndVelocity[2] = { position, velocity };
	if (   !frame.WriteInt8(Register::Mode, &mode, 1)
		|| !frame.WriteFloat(Register::CommandPosition, positionAndVelocity, 2)
		|| !frame.WriteFloat(Register::CommandMaxTorque, &maxTorque, 1)
		|| (query && !AddStateQuery(frame))
	   )
	{
		return false;
	}
	frame.Finish();
	return true;
}

bool Moteus::MakeQuery(CanMessageBuffer& buf, uint8_t servoId) noexcept
{
	FrameBuilder frame(buf, servoId);
	if (!AddStateQuery(frame))
	{
		return false;
	}
	frame.Finish();
	return true;
}

// Reply parsing

enum class ParseResult : uint8_t { notForUs, ok, error };

static bool FetchVaruint(const uint8_t *_ecv_array data, size_t length, size_t& pos, uint32_t& value) noexcept
{
	value = 0;
	for (unsigned int shift = 0; shift < 35; shift += 7)
	{
		if (pos >= length)
		{
			return false;
		}
		const uint8_t b = data[pos++];
		value |= (uint32_t)(b & 0x7F) << shift;
		if ((b & 0x80) == 0)
		{
			return true;
		}
	}
	return false;
}

// Return the value of one LSB of an integer reply for this register, or 0 if the register has no scaling (e.g. mode, fault)
static float ScaleFor(uint16_t reg, uint8_t replyType) noexcept
{
	const unsigned int sizeIndex = (replyType == SubframeType::ReplyInt8) ? 0 : (replyType == SubframeType::ReplyInt16) ? 1 : 2;
	switch (reg)
	{
	case Moteus::Register::Position:		{ constexpr float s[] = { 0.01, 0.0001, 0.00001 };	return s[sizeIndex]; }
	case Moteus::Register::Velocity:		{ constexpr float s[] = { 0.1, 0.00025, 0.00001 };	return s[sizeIndex]; }
	case Moteus::Register::Torque:			{ constexpr float s[] = { 0.5, 0.01, 0.001 };		return s[sizeIndex]; }
	case Moteus::Register::Voltage:			{ constexpr float s[] = { 0.5, 0.1, 0.001 };		return s[sizeIndex]; }
	case Moteus::Register::Temperature:		{ constexpr float s[] = { 1.0, 0.1, 0.001 };		return s[sizeIndex]; }
	default:								return 0.0;
	}
}

static void StoreValue(Moteus::ServoState& state, uint16_t reg, float value) noexcept
{
	switch (reg)
	{
	case Moteus::Register::Mode:			state.mode = (uint8_t)value; break;
	case Moteus::Register::Fault:			state.fault = (uint8_t)value; break;
	case Moteus::Register::Position:		state.position = value; break;
	case Moteus::Register::Velocity:		state.velocity = value; break;
	case Moteus::Register::Torque:			state.torque = value; break;
	case Moteus::Register::Voltage:			state.voltage = value; break;
	case Moteus::Register::Temperature:		state.temperature = value; break;
	default:								break;		// a register we don't track
	}
}

static ParseResult Parse(const CanMessageBuffer& buf, uint8_t servoId, Moteus::ServoState& state, uint8_t hostId) noexcept
{
	const uint32_t expectedId = ((uint32_t)servoId << 8) | hostId;
	if (buf.id.GetWholeId() != expectedId)
	{
		return ParseResult::notForUs;
	}

	const uint8_t *_ecv_array const data = buf.msg.raw;
	const size_t length = buf.dataLength;
	size_t pos = 0;
	while (pos < length)
	{
		const uint8_t type = data[pos++];
		if (type == SubframeType::Nop)
		{
			continue;
		}
		if (type == SubframeType::WriteError || type == SubframeType::ReadError)
		{
			return ParseResult::error;
		}
		if ((type & 0xF0) != SubframeType::ReplyBase)
		{
			return ParseResult::error;					// a subframe type we don't expect in a reply
		}

		const uint8_t replyType = type & 0xFC;
		uint32_t count = type & 0x03;
		uint32_t reg;
		if ((count == 0 && !FetchVaruint(data, length, pos, count)) || !FetchVaruint(data, length, pos, reg))
		{
			return ParseResult::error;
		}

		const size_t valueSize = (replyType == SubframeType::ReplyInt8) ? 1 : (replyType == SubframeType::ReplyInt16) ? 2 : 4;
		if (pos + count * valueSize > length)
		{
			return ParseResult::error;
		}

		for (uint32_t i = 0; i < count; ++i, ++reg, pos += valueSize)
		{
			float value;
			if (replyType == SubframeType::ReplyFloat)
			{
				value = LoadLEF32(&data[pos]);
			}
			else
			{
				const int32_t raw = (replyType == SubframeType::ReplyInt8) ? (int32_t)(int8_t)data[pos]
									: (replyType == SubframeType::ReplyInt16) ? (int32_t)LoadLEI16(&data[pos])
										: LoadLEI32(&data[pos]);
				const bool isUnset = (replyType == SubframeType::ReplyInt8) ? raw == std::numeric_limits<int8_t>::min()
										: (replyType == SubframeType::ReplyInt16) ? raw == std::numeric_limits<int16_t>::min()
											: raw == std::numeric_limits<int32_t>::min();
				const float scale = ScaleFor((uint16_t)reg, replyType);
				value = (scale == 0.0) ? (float)raw
						: (isUnset) ? std::numeric_limits<float>::quiet_NaN()
							: (float)raw * scale;
			}
			StoreValue(state, (uint16_t)reg, value);
		}
	}
	return ParseResult::ok;
}

bool Moteus::ParseReply(const CanMessageBuffer& buf, uint8_t servoId, ServoState& state, uint8_t hostId) noexcept
{
	return Parse(buf, servoId, state, hostId) == ParseResult::ok;
}

// Request and reply

bool Moteus::Transact(CanTransport& bus, CanMessageBuffer& buf, uint8_t servoId, ServoState& state, uint32_t timeoutMillis) noexcept
{
	while (bus.Receive(CanRxQueue::fifo0, 0, nullptr)) { }		// discard any stale frames so we don't match an old reply

	(void)bus.Send(CanTxSlot::fifo, timeoutMillis, &buf);

	const uint32_t startTime = millis();
	for (;;)
	{
		const uint32_t elapsed = millis() - startTime;
		if (elapsed >= timeoutMillis || !bus.Receive(CanRxQueue::fifo0, timeoutMillis - elapsed, &buf))
		{
			return false;
		}
		switch (Parse(buf, servoId, state, DefaultHostId))
		{
		case ParseResult::ok:		return true;
		case ParseResult::error:	return false;
		case ParseResult::notForUs:	break;							// another device's frame, keep waiting
		}
	}
}

#endif	// SUPPORT_CAN_EXPANSION && DUAL_CAN
