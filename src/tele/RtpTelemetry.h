#ifndef TELE_RTP_TELEMETRY_H_
#define TELE_RTP_TELEMETRY_H_

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

namespace Rtp
{
constexpr configSTACK_DEPTH_TYPE kStackDepth = 512;
constexpr uint8_t kStartOfFrame = 0xAA;
constexpr uint8_t kVersion = 0x01;
constexpr std::size_t kMaxPayload = 64; // TBD

enum class RtpType : uint8_t
{
    PID = 0x01
};

#pragma pack(push, 1)
struct RtpHeader
{
    uint8_t startOfFrame;
    uint8_t version;
    RtpType type;
    uint8_t flags;
    uint8_t length;
};
#pragma pack(pop)

class RtpTelemetry
{
  public:
    RtpTelemetry() = default;

    void begin();
    void publish(RtpType type, const void* payload, uint8_t len);

  private:
    struct Item
    {
        RtpType type;
        uint8_t len;
        uint8_t payload[kMaxPayload];
    };

    QueueHandle_t m_queue{nullptr};

    static void telemetryTask(void* arg);
    static auto calculateCrc8(const uint8_t* data, size_t len) -> uint8_t;
};

} // namespace Rtp

#endif
