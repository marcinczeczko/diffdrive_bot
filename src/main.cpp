
#include "config/Config.h"
#include "controller/VelocityController.h"
#include "task/TaskVirtualAnchor.h"
#include "tele/RtpTelemetry.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#ifdef RUN_MODE_CALIB
#include "calibrator/CalibModule.h"
#endif

Rtp::RtpTelemetry telemetry;

#ifdef RUN_MODE_NORMAL
VelocityController controller(ControllerMode::AUTO);
#else
VelocityController controller(ControllerMode::MANUAL);
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

    robot.begin();
    xTaskCreate(VirtualAnchorTask::startTask, "Move", 256, &robot, 1, nullptr);
    vTaskStartScheduler();

#endif
#ifdef RUN_MODE_CALIB
    calib.begin();
#endif
}

void loop()
{
}
