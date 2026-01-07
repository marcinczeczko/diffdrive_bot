#ifndef TELE_RTP_TELEMETRY_H_
#define TELE_RTP_TELEMETRY_H_

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

class VelocityController;

namespace Rtp
{
constexpr configSTACK_DEPTH_TYPE kStackDepth = 256;
constexpr uint16_t kMagic = 0xAA55;
constexpr uint8_t kMagic_0 = 0xAA;
constexpr uint8_t kMagic_1 = 0x55;
constexpr uint8_t kVersion = 0x01;
constexpr std::size_t kMaxPayload = 160; // TBD

using RtpType = uint8_t;

constexpr RtpType RTP_PID = 0x01;
constexpr RtpType RTP_ODOM = 0x02;
constexpr RtpType RTP_IMU = 0x03;

struct RtpHeader
{
    uint8_t magic[2];
    uint8_t type;
    uint8_t len;
    uint8_t crc;
} __attribute__((packed));

static_assert(sizeof(RtpType) == 1, "RtpType must be 1 byte");
static_assert(sizeof(RtpHeader) == 5, "RtpHeader layout broken");

constexpr RtpType RTP_REQ_PID_SIDE = 0x10; // Ustawienie PID
constexpr RtpType RTP_REQ_PID_ALL = 0x11;  // Ustawienie PID
constexpr RtpType RTP_REQ_CMD = 0x20;      // Komenda tekstowa (CLI)

class RtpTelemetry
{
  public:
    RtpTelemetry() = default;

    void begin();

    // This will assure that the only way to use it is
    // PidPayload payload{ ... }; telemetry.publish(RTP_PID, payload);
    // So no references or pointers to payload to avoid any data consistency issues
    template <typename T> void publish(uint8_t type, const T& payload)
    {
        static_assert(std::is_trivially_copyable<T>::value,
                      "Telemetry payload must be trivially copyable");

        static_assert(sizeof(T) <= kMaxPayload, "Telemetry payload too large");

        publishRaw(type, &payload, sizeof(T));
    }

    // // Dodaj w public:
    void setController(VelocityController* ctrl);
    // Dodaj w private:

  private:
    struct Item
    {
        RtpType type;
        uint8_t len;
        uint8_t payload[kMaxPayload];
    };

    QueueHandle_t m_queue{nullptr};
    VelocityController* m_controller;

    void publishRaw(RtpType type, const void* payload, size_t len);

    static auto calculateCrc8(const uint8_t* data, size_t len) -> uint8_t;
    static void telemetryTask(void* pvParameters);
    static void receiverTask(void* pvParameters);
};

} // namespace Rtp

#endif
