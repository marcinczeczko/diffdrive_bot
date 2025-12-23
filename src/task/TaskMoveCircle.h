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

        vTaskDelay(pdMS_TO_TICKS(2000)); // Czekaj na start po zasilaniu ;)

        // --- PARAMETRY OKRĘGU ---
        const float targetRadiusCm = 10.0f;
        const float linearVel = 15.0f;                       // 15 cm/s
        const float angularVel = linearVel / targetRadiusCm; // 0.3 rad/s

        LOG_INFO("Starting circle maneuver (D=1m)...");
        odo.reset();

        // Ustawiamy ruch po łuku
        vc->setTwist(linearVel, angularVel);

        float accumulatedAngle = 0.0f;
        float lastTheta = odo.getPose().theta;

        while (true)
        {
            Pose p = odo.getPose();

            // Obliczamy zmianę kąta, uwzględniając przeskoki -PI / PI
            float deltaTheta = p.theta - lastTheta;
            if (deltaTheta > PI)
                deltaTheta -= 2.0f * PI;
            if (deltaTheta < -PI)
                deltaTheta += 2.0f * PI;

            accumulatedAngle += deltaTheta;
            lastTheta = p.theta;

            // Logujemy postęp co jakiś czas
            if (vc->getLoopCounter() % 20 == 0)
            {
                LOG_DATA("Progress (deg)", abs(accumulatedAngle) * 180.0f / PI);
            }

            // Warunek końca: pełne okrążenie (2 * PI = 6.28 rad)
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