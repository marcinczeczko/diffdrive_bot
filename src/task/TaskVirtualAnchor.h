#ifndef VIRTUAL_ANCHOR_TASK_H_
#define VIRTUAL_ANCHOR_TASK_H_

#include "controller/VelocityController.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

class VirtualAnchorTask
{
  public:
    static void startTask(void* pvParameters)
    {
        auto* vc = (VelocityController*)pvParameters;
        auto& odo = vc->getOdometry();

        // 1. "Drop the anchor" at the current position
        Pose anchor = odo.getPose();
        LOG_INFO("Anchor set! Try to move me.");

        // "Spring" parameters (P-Controller)
        const float Kp_lin = 1.5f;   // Linear return force
        const float Kp_ang = 2.0f;   // Angular return force
        const float MAX_V = 15.0f;   // Max return speed [cm/s]
        const float MAX_W = 2.0f;    // Max angular speed [rad/s]
        const float DEADZONE = 0.5f; // 0.5 cm tolerance

        for (;;)
        {
            Pose current = odo.getPose();

            // 2. Compute position errors
            float errorX = anchor.x - current.x;
            float errorY = anchor.y - current.y;
            float errorTheta = anchor.theta - current.theta;

            // Normalize angle error to -PI..PI
            while (errorTheta > PI)
                errorTheta -= 2.0f * PI;
            while (errorTheta < -PI)
                errorTheta += 2.0f * PI;

            // 3. Compute distance to the anchor point
            float distance = sqrt(errorX * errorX + errorY * errorY);

            float v = 0;
            float w = 0;

            // 4. Return logic (simplified - robot drives toward X, Y, and Theta independently)
            if (distance > DEADZONE || abs(errorTheta) > 0.05f)
            {

                // Linear velocity (proportional to error)
                // Simplification: robot moves forward/back based on errorX in its own frame
                // For full 2D, rotate the error vector into the robot frame
                float localErrorX = errorX * cos(current.theta) + errorY * sin(current.theta);
                v = localErrorX * Kp_lin;

                // Angular velocity
                w = errorTheta * Kp_ang;

                // 5. Speed limits (saturation)
                if (v > MAX_V)
                    v = MAX_V;
                if (v < -MAX_V)
                    v = -MAX_V;
                if (w > MAX_W)
                    w = MAX_W;
                if (w < -MAX_W)
                    w = -MAX_W;

                vc->setTwist(v, w);
            }
            else
            {
                // We are home - release the motors
                vc->setTwist(0, 0);
            }

            vTaskDelay(pdMS_TO_TICKS(50)); // Check 20 times per second
        }
    }
};

#endif
