#include "RtpTelemetry.h"

#include "config/Config.h"

namespace Rtp
{
// ---- CRC8 ----
auto RtpTelemetry::calculateCrc8(const uint8_t* data, size_t len) -> uint8_t
{
    static constexpr uint8_t CRC8_INIT = 0x00;
    static constexpr uint8_t CRC8_MSB_MASK = 0x80;
    static constexpr uint8_t CRC8_POLYNOMIAL = 0x07;
    static constexpr uint8_t BITS_PER_BYTE = 8;

    uint8_t crc = CRC8_INIT;
    for (size_t i = 0; i < len; ++i)
    {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        crc ^= data[i];
        for (uint8_t j = 0; j < BITS_PER_BYTE; j++)
        {
            crc = (crc & CRC8_MSB_MASK) ? (crc << 1) ^ CRC8_POLYNOMIAL : (crc << 1);
        }
    }
}

// ---- Init ----
void RtpTelemetry::begin()
{
    // Jeśli chcesz używać xQueueOverwrite, kolejka MUSI mieć długość 1.
    // Jeśli chcesz buforować 8 wiadomości, użyj xQueueSend.
    m_queue = xQueueCreate(8, sizeof(Item));

    if (m_queue == nullptr)
    {
        LOG_ERR("Not enough memory for Queue");
        while (true)
        {
            delay(100);
        }
    }

    BaseType_t result = xTaskCreate(telemetryTask, "rtpTele", kStackDepth, this, 1, nullptr);
    if (result != pdPASS)
    {
        LOG_ERR("Not enough memory for telemetry task");
        while (true)
        {
            delay(100);
        }
    }
}

// ---- Publish (callable from task willing to publish telemetry data) ----
void RtpTelemetry::publish(RtpType type, const void* payload, uint8_t len)
{
    if (len > kMaxPayload || m_queue == nullptr)
    {
        return;
    }

    Item item = {};
    item.type = type;
    item.len = len;
    memcpy(item.payload, payload, len);

    // overwrite → always most fresh data
    xQueueOverwrite(m_queue, &item);
}

// ---- Telemetry Task ----
void RtpTelemetry::telemetryTask(void* arg)
{
    auto* self = static_cast<RtpTelemetry*>(arg);

    Item item{};

    for (;;)
    {
        if (xQueueReceive(self->m_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            RtpHeader hdr{kStartOfFrame, kVersion, item.type,
                          0x01, // flags
                          item.len};

            uint8_t crc = 0;
            crc ^= calculateCrc8(&hdr.version, 4);
            crc ^= calculateCrc8(item.payload, item.len);

            Serial.write((uint8_t*)&hdr, sizeof(hdr));
            Serial.write(item.payload, item.len);
            Serial.write(crc);
        }
    }
}
} // namespace Rtp
