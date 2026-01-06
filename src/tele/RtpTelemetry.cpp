#include "RtpTelemetry.h"

#include "config/Config.h"
#include "controller/PidTypes.h"
#include "controller/VelocityController.h"
#include "task/PidTestTask.h"

extern PidTestTask* g_pidTestTask;

namespace Rtp
{
void RtpTelemetry::setController(VelocityController* controller)
{
    m_controller = controller;
}

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

    if (m_controller != nullptr)
    {
        BaseType_t result = xTaskCreate(receiverTask, "rtpRecv", kStackDepth, this, 2, nullptr);
        if (result != pdPASS)
        {
            LOG_ERR("Not enough memory for telemetry receiver task");
            while (true)
            {
                delay(100);
            }
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
            Serial.write(frame, pos + 1); // R4 CDC Hack for ZLP flag
            // Serial.flush(); // IMPORTANT for USB CDC
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

void RtpTelemetry::receiverTask(void* pvParameters)
{
    auto* self = (RtpTelemetry*)pvParameters;

    enum State : uint8_t
    {
        WAIT_MAGIC_1,
        WAIT_MAGIC_2,
        READ_HEADER,
        READ_PAYLOAD,
        READ_CRC
    };
    State state = WAIT_MAGIC_1;

    RtpHeader header{};
    uint8_t payload[kMaxPayload];
    uint8_t headerIdx = 0;
    uint8_t payloadIdx = 0;

    for (;;)
    {
        if (Serial.available())
        {
            uint8_t b = Serial.read();

            switch (state)
            {
            case WAIT_MAGIC_1:
                if (b == kMagic_0)
                    state = WAIT_MAGIC_2;
                break;

            case WAIT_MAGIC_2:
                if (b == kMagic_1)
                {
                    state = READ_HEADER;
                    headerIdx = 2; // magic już mamy
                    ((uint8_t*)&header)[0] = kMagic_0;
                    ((uint8_t*)&header)[1] = kMagic_1;
                }
                else
                    state = WAIT_MAGIC_1;
                break;

            case READ_HEADER:
                ((uint8_t*)&header)[headerIdx++] = b;
                if (headerIdx == sizeof(RtpHeader))
                {
                    // Sprawdź CRC nagłówka (4 pierwsze bajty)
                    if (calculateCrc8((uint8_t*)&header, 4) == header.crc)
                    {
                        payloadIdx = 0;
                        if (header.len > 0)
                            state = READ_PAYLOAD;
                        else
                            state = READ_CRC;
                    }
                    else
                        state = WAIT_MAGIC_1;
                }
                break;

            case READ_PAYLOAD:
                payload[payloadIdx++] = b;
                if (payloadIdx == header.len)
                    state = READ_CRC;
                break;

            case READ_CRC:
                if (calculateCrc8(payload, header.len) == b)
                {
                    // --- RAMKA POPRAWNA - PROCESUJ ---
                    if (header.type == RTP_REQ_PID_SIDE && self->m_controller != nullptr)
                    {
                        auto* cfg = (PidSideTestCommand*)payload;

                        PidSideTestCommand cmd{};
                        cmd.motor = cfg->motor;
                        cmd.kp = cfg->kp;
                        cmd.ki = cfg->ki;
                        cmd.kff = cfg->kff;
                        cmd.alpha = cfg->alpha;
                        cmd.testRps = cfg->testRps;
                        cmd.rampType = cfg->rampType;

                        g_pidTestTask->enqueue(cmd);
                    }
                    else if (header.type == RTP_REQ_PID_ALL && self->m_controller != nullptr)
                    {
                        auto* cfg = (PidTestAllCommand*)payload;

                        PidTestAllCommand cmd{};
                        cmd.l_kp = cfg->l_kp;
                        cmd.l_ki = cfg->l_ki;
                        cmd.l_kff = cfg->l_kff;
                        cmd.l_alpha = cfg->l_alpha;
                        cmd.l_testRps = cfg->l_testRps;
                        cmd.l_rampType = cfg->l_rampType;
                        cmd.r_kp = cfg->r_kp;
                        cmd.r_ki = cfg->r_ki;
                        cmd.r_kff = cfg->r_kff;
                        cmd.r_alpha = cfg->r_alpha;
                        cmd.r_testRps = cfg->r_testRps;
                        cmd.r_rampType = cfg->r_rampType;

                        g_pidTestTask->enqueue(cmd);
                    }
                    else if (header.type == RTP_REQ_CMD)
                    {
                        // CLI: payload jako string
                        payload[header.len] = '\0'; // Null terminator
                        LOG_INFO((char*)payload);
                        // Tutaj możesz dodać prosty parser komend tekstowych
                    }
                }
                state = WAIT_MAGIC_1;
                break;
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // Czekaj na dane
        }
    }
}

} // namespace Rtp
