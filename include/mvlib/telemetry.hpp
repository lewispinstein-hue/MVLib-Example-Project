#pragma once

#include "core.hpp"
#include "pros/rtos.hpp"
#include <cstdint>
#include <string>

namespace mvlib {

/**
 * @enum MsgType
 * @brief Identifies the binary packet type for the MotionView decoder.
 */
enum class MsgType : uint8_t {
    POSE   = 0x01, // High-speed drivetrain/position
    WPOINT = 0x02, // Waypoint state changes/offsets
    WATCH  = 0x03, // Variable watches (numeric)
    ROSTER = 0x04, // ID-to-Name mapping (The "Late Joiner" fix)
    LOG    = 0x05  // Standard text-based logs
};

// --- Binary Packet Structures (Strictly Packed) ---

struct __attribute__((packed)) PosePacket {
    uint32_t timestamp;
    float x;
    float y;
    float theta;
    float leftVel;
    float rightVel;
};

struct __attribute__((packed)) WaypointPacket {
    uint32_t timestamp;
    uint64_t id;
    uint8_t  state; // 0=OFFSET, 1=REACHED, 2=TIMEDOUT
    float    offX;
    float    offY;
    float    offT;
    float    totalOff;
    int32_t  remainingTimeout; // -1 if NA
};

struct __attribute__((packed)) RosterPacket {
    uint64_t id;
    char     name[24]; // Fixed size for binary stability
};

struct __attribute__((packed)) WatchPacket {
    uint32_t timestamp;
    uint64_t id;
    uint8_t  level;
    float    value;
    // Note: 'label' and 'fmt' are handled via ROSTER or skipped for bandwidth
};

/**
 * @brief Header for variable-length text logs.
 * The actual message string follows immediately after this header in the buffer.
 */
struct __attribute__((packed)) LogPacketHeader {
    uint32_t timestamp;
    uint8_t  level;
};

// --- The Unified Telemetry Class ---

class Telemetry {
public:
    /**
     * @brief Singleton access to the telemetry engine.
     */
    static Telemetry& getInstance();

    /**
     * @brief Set the global minimum log level. 
     * If a log's level is lower than this, it is discarded before processing.
     */
    void setMinLevel(LogLevel level);

    /**
     * @brief Core logic check: Should this level be processed?
     */
    bool shouldLog(LogLevel level) const;

    // --- High-Speed Binary Methods ---

    void sendPose(const PosePacket& pkt);
    void sendWaypoint(const WaypointPacket& pkt);
    void sendWatch(const WatchPacket& pkt);
    void sendRoster(uint64_t id, const std::string& name);

    /**
     * @brief Sends a standard text log wrapped in a binary frame.
     * This replaces the old printf-to-terminal logic.
     */
    void sendText(LogLevel level, const char* fmt, ...);

private:
    Telemetry() : m_minLevel(LogLevel::INFO) {}
    
    LogLevel m_minLevel;
    pros::Mutex m_terminalMutex;

    /**
     * @brief Internal COBS encoder and UART writer.
     * Frames data with 0x00 delimiters to prevent stream desync.
     */
    void transmit(MsgType type, const uint8_t* data, size_t len);

    // Prevent copying
    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;
};

} // namespace mvlib