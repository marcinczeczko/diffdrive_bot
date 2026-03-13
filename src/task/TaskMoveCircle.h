#ifndef MOVE_CIRCE_TASK_H_
#define MOVE_CIRCE_TASK_H_

#include "controller/VelocityController.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

class MoveCircleTask
{
  public:
    static void startTask(void* pvParameters)
    {
        auto* vc = (VelocityController*)pvParameters;
        auto& odo = vc->getOdometry();

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for start after power-up ;)

        // --- PARAMETRY OKRĘGU ---
        const float targetRadiusCm = 10.0f;
        const float linearVel = 15.0f;                       // 15 cm/s
        const float angularVel = linearVel / targetRadiusCm; // 0.3 rad/s

        LOG_INFO("Starting circle maneuver (D=1m)...");
        odo.reset();

        // Set arc motion
        vc->setTwist(linearVel, angularVel);

        float accumulatedAngle = 0.0f;
        float lastTheta = odo.getPose().theta;

        while (true)
        {
            Pose p = odo.getPose();

            // Compute angle change, accounting for wrap-around at -PI / PI
            float deltaTheta = p.theta - lastTheta;
            if (deltaTheta > PI)
                deltaTheta -= 2.0f * PI;
            if (deltaTheta < -PI)
                deltaTheta += 2.0f * PI;

            accumulatedAngle += deltaTheta;
            lastTheta = p.theta;

            // Log progress periodically
            if (vc->getLoopCounter() % 20 == 0)
            {
                LOG_DATA("Progress (deg)", abs(accumulatedAngle) * 180.0f / PI);
            }

            // End condition: full circle (2 * PI = 6.28 rad)
            if (abs(accumulatedAngle) >= 2.0f * PI)
            {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vc->stopEmergency();
        LOG_INFO("Circle complete!");

        vTaskDelete(NULL);
    }
};

#endif
