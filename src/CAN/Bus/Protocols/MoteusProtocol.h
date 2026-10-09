/*
 * MoteusProtocol.h
 *
 * mjbots moteus motor controllers over CAN-FD. See https://mjbots.github.io/moteus/protocol/can/
 *
 * Each frame carries a sequence of register subframes (write, read, reply). Frames are addressed with the 7-bit host (source)
 * and servo (destination) IDs; setting the reply-required bit makes the servo answer with the registers that were read.
 *
 * moteus normally runs at 1Mbit/s with a 5Mbit/s data phase, which the 48MHz CAN clock cannot generate. We therefore send
 * CAN-FD frames without bit rate switching, so the whole frame runs at 1Mbit/s. moteus copies the BRS and FD flags of a
 * request into its reply, so replies also arrive without BRS.
 */

#ifndef SRC_CAN_BUS_PROTOCOLS_MOTEUSPROTOCOL_H_
#define SRC_CAN_BUS_PROTOCOLS_MOTEUSPROTOCOL_H_

#include "CanBusProtocol.h"

#if SUPPORT_CAN_EXPANSION && DUAL_CAN

class CanMessageBuffer;

class MoteusProtocol final : public CanBusProtocol
{
public:
	static constexpr uint32_t BitRate = 1'000'000;
	static constexpr float SamplePoint = 0.666;											// recommended by the moteus documentation
	static constexpr size_t MaxDataLength = 64;											// CAN-FD

	const char *_ecv_array GetName() const noexcept override { return "moteus"; }
	void GetTiming(CanTiming& timing) const noexcept override;
	size_t GetMaxDataLength() const noexcept override { return MaxDataLength; }
	void SetupFilters(CanTransport& bus) const noexcept override;
};

namespace Moteus
{
	constexpr uint8_t DefaultHostId = 0;												// the source ID we send from; replies are addressed to it
	constexpr uint8_t MaxServoId = 127;

	// Register numbers
	namespace Register
	{
		constexpr uint16_t Mode = 0x000;
		constexpr uint16_t Position = 0x001;
		constexpr uint16_t Velocity = 0x002;
		constexpr uint16_t Torque = 0x003;
		constexpr uint16_t QCurrent = 0x004;
		constexpr uint16_t DCurrent = 0x005;
		constexpr uint16_t Voltage = 0x00d;
		constexpr uint16_t Temperature = 0x00e;
		constexpr uint16_t Fault = 0x00f;
		constexpr uint16_t CommandPosition = 0x020;
		constexpr uint16_t CommandVelocity = 0x021;
		constexpr uint16_t CommandFeedforwardTorque = 0x022;
		constexpr uint16_t CommandKpScale = 0x023;
		constexpr uint16_t CommandKdScale = 0x024;
		constexpr uint16_t CommandMaxTorque = 0x025;
		constexpr uint16_t CommandStopPosition = 0x026;
		constexpr uint16_t CommandWatchdogTimeout = 0x027;
		constexpr uint16_t CommandVelocityLimit = 0x028;
		constexpr uint16_t CommandAccelLimit = 0x029;
	}

	// Values of the mode register. Only the documented values are named; others are reported as their raw number.
	enum class Mode : uint8_t
	{
		stopped = 0,
		fault = 1,
		pwm = 5,
		position = 10,
		timeout = 11,
	};

	// Builds one moteus frame in a CanMessageBuffer. Call the Write/Read functions, then Finish before sending.
	// Each function returns false if the frame is full, in which case the frame must not be sent.
	class FrameBuilder
	{
	public:
		FrameBuilder(CanMessageBuffer& p_buf, uint8_t servoId, uint8_t hostId = DefaultHostId) noexcept;

		bool WriteInt8(uint16_t startRegister, const int8_t *_ecv_array values, size_t count) noexcept;
		bool WriteFloat(uint16_t startRegister, const float *_ecv_array values, size_t count) noexcept;
		bool ReadInt8(uint16_t startRegister, size_t count) noexcept;							// also sets the reply-required bit
		bool ReadFloat(uint16_t startRegister, size_t count) noexcept;							// also sets the reply-required bit

		void Finish() noexcept;																	// pad to a valid CAN-FD length and set the ID and flags

	private:
		bool AddHeader(uint8_t baseType, uint16_t startRegister, size_t count) noexcept;
		bool AddVaruint(uint32_t value) noexcept;
		bool HaveSpace(size_t bytes) const noexcept { return length + bytes <= MoteusProtocol::MaxDataLength; }

		CanMessageBuffer& buf;
		size_t length;
		uint8_t servoId;
		uint8_t hostId;
		bool replyRequired;
		bool overflowed;
	};

	// The servo state decoded from a reply. Fields that were not in the reply are left unchanged.
	struct ServoState
	{
		uint8_t mode = 0;
		uint8_t fault = 0;
		float position = 0.0;							// revolutions
		float velocity = 0.0;							// revolutions/second
		float torque = 0.0;								// N.m
		float voltage = 0.0;							// volts
		float temperature = 0.0;						// degrees C
	};

	// Frames for common operations. Each returns false if the frame could not be built.
	bool MakeStop(CanMessageBuffer& buf, uint8_t servoId, bool query) noexcept;
	bool MakePosition(CanMessageBuffer& buf, uint8_t servoId, float position, float velocity, float maxTorque, bool query) noexcept;
	bool MakeQuery(CanMessageBuffer& buf, uint8_t servoId) noexcept;

	// Decode a received frame. Returns false if it is not a reply from servoId to hostId, or if it contains an error subframe.
	bool ParseReply(const CanMessageBuffer& buf, uint8_t servoId, ServoState& state, uint8_t hostId = DefaultHostId) noexcept;

	// Send a frame built with the reply-required bit set and wait for the matching reply, decoding it into state.
	// On return buf holds the last frame received. Returns false if no valid reply arrived within the timeout.
	bool Transact(CanTransport& bus, CanMessageBuffer& buf, uint8_t servoId, ServoState& state, uint32_t timeoutMillis) noexcept;
}

#endif	// SUPPORT_CAN_EXPANSION && DUAL_CAN

#endif /* SRC_CAN_BUS_PROTOCOLS_MOTEUSPROTOCOL_H_ */
