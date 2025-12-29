#include "RtpTelemetry.h"

#include "config/Config.h"

namespace Rtp
{
// ---- Init ----
void RtpTelemetry::begin()
{
    // Jeśli chcesz używać xQueueOverwrite, kolejka MUSI mieć długość 1.
    // Jeśli chcesz buforować 8 wiadomości, użyj xQueueSend.
    m_queue = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(Item));

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

// ---- Publish (callable from task willing to publish telemetry data) ---
void RtpTelemetry::publishRaw(RtpType type, const void* payload, size_t len)
{
    if (payload == nullptr || len == 0 || len > kMaxPayload || m_queue == nullptr)
    {
        return;
    }

    Item item{};
    item.type = type;
    item.len = len;
    memcpy(item.payload, payload, len);

    // telemetria → zawsze ostatnia próbka
    xQueueSend(m_queue, &item, 0);
}

// ---- Telemetry Task ----
void RtpTelemetry::telemetryTask(void* pvParameters)
{
    auto* self = (RtpTelemetry*)pvParameters;

    Item item{};

    for (;;)
    {
        if (xQueueReceive(self->m_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            // [HEADER(5) + PAYLOAD + CRC]
            uint8_t frame[sizeof(RtpHeader) + kMaxPayload + 1];
            size_t pos = 0;

            // --- Header ---
            frame[pos++] = kMagic_0;
            frame[pos++] = kMagic_1;
            frame[pos++] = static_cast<uint8_t>(item.type);
            frame[pos++] = item.len;
            frame[pos++] = calculateCrc8(reinterpret_cast<const uint8_t*>(frame), 4);

            // --- Payload ---
            memcpy(&frame[pos], item.payload, item.len);
            pos += item.len;

            // --- Payload CRC ---
            frame[pos++] = calculateCrc8(item.payload, item.len);

            // --- Send atomically ---
            Serial.write(frame, pos);
            Serial.flush(); // IMPORTANT for USB CDC
        }
    }
}

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
    return crc;
}

} // namespace Rtp
