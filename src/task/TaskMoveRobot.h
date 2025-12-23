#ifndef MOVE_ROBOT_TASK_H_
#define MOVE_ROBOT_TASK_H_

#include "controller/VelocityController.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

class MoveRobotTask
{
  public:
    static void startTask(void* pvParameters)
    {
        auto* vc = (VelocityController*)pvParameters;
        auto& odo = vc->getOdometry();

        vTaskDelay(pdMS_TO_TICKS(2000)); // Czekaj na start

        // --- MANEWR 1: OBRÓT O 90 STOPNI ---
        LOG_INFO("Rotating 90 deg...");
        odo.reset();
        vc->setTwist(0, 5.0f); // Obrót 1 rad/s
        while (true)
        {
            Pose p = odo.getPose();
            if (abs(p.theta) >= 1.57f)
                break; // 90 deg = 1.57 rad
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        vc->stopEmergency();
        vTaskDelay(pdMS_TO_TICKS(500));

        // --- MANEWR 2: JAZDA 30 CM ---
        LOG_INFO("Driving 30 cm...");
        odo.reset();
        vc->setTwist(10.0f, 0); // 10 cm/s prosto

        while (true)
        {
            Pose p = odo.getPose();
            // Liczymy dystans euklidesowy od startu (0,0)
            float dist = sqrt(p.x * p.x + p.y * p.y);
            if (dist >= 30.0f)
                break;
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        vc->stopEmergency();
        LOG_INFO("Mission Complete!");
        vTaskDelete(NULL);
    }
};

#endif