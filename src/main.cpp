
#include "config/Config.h"
#include "controller/VelocityController.h"
#include "task/TestDriveTask.h"
#include "tele/RtpTelemetry.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#ifdef RUN_MODE_CALIB
#include "calibrator/CalibModule.h"
#endif

Rtp::RtpTelemetry telemetry;

#ifdef RUN_MODE_NORMAL
VelocityController controller(&telemetry, ControllerMode::AUTO);
#else
VelocityController controller(&telemetry, ControllerMode::MANUAL);
CalibModule calib(&controller);
#endif

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Wait for R4 Serial
    }
    telemetry.begin();

    // =============================
    // 3. NORMAL OPERATION
    // =============================
#ifdef RUN_MODE_NORMAL

    controller.begin();
    xTaskCreate(TestDriveTask::startTask, "Move", 512, &controller, 1, nullptr);
    vTaskStartScheduler();

#endif
#ifdef RUN_MODE_CALIB
    calib.begin();
#endif
}

void loop()
{
}
