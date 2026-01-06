
#include "config/Config.h"
#include "controller/VelocityController.h"
#include "task/PidTestTask.h"
#include "tele/RtpTelemetry.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#ifdef RUN_MODE_CALIB
#include "calibrator/CalibModule.h"
#endif

static Rtp::RtpTelemetry telemetry;

#ifdef RUN_MODE_NORMAL
static VelocityController controller(&telemetry, ControllerMode::AUTO);
static PidTestTask pidTest(&controller);
PidTestTask* g_pidTestTask = &pidTest;

#else
static VelocityController controller(&telemetry, ControllerMode::MANUAL);
static CalibModule calib(&controller);
PidTestTask* g_pidTestTask = nullptr;
#endif

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Wait for R4 Serial
    }
    telemetry.setController(&controller);
    telemetry.begin();
    // =============================
    // 3. NORMAL OPERATION
    // =============================
#ifdef RUN_MODE_NORMAL

    pidTest.begin();
    controller.begin();
    vTaskStartScheduler();

#endif
#ifdef RUN_MODE_CALIB
    calib.begin();
#endif
}

void loop()
{
}
