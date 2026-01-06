#ifndef PID_TEST_TASK_H_
#define PID_TEST_TASK_H_

#include "controller/PidTypes.h"
#include "controller/VelocityController.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#pragma once

class VelocityController;

class PidTestTask
{
  public:
    PidTestTask(VelocityController* vc) : controller(vc)
    {
    }

    void begin()
    {
        queue = xQueueCreate(1, sizeof(PidTestCommand));
        configASSERT(queue);

        xTaskCreate(PidTestTask::taskFn, "PidTest", 128, this,
                    4, // higher than velocity loop
                    nullptr);
    }
    void enqueue(const PidTestCommand& cmd)
    {
        xQueueOverwrite(queue, &cmd);
    }

  private:
    VelocityController* controller;
    QueueHandle_t queue = nullptr;

    static void taskFn(void* arg)
    {
        static_cast<PidTestTask*>(arg)->run();
    }
    void run()
    {
        PidTestCommand cmd{};
        for (;;)
        {
            if (xQueueReceive(queue, &cmd, portMAX_DELAY) == pdTRUE)
            {
                // Serial.println("RECEIVED MESSAGE");
                //  1. STOP EVERYTHING
                controller->hardStop();
                vTaskDelay(pdMS_TO_TICKS(1000));

                controller->resetEncoders();
                controller->resetMotorState(cmd.motor);

                // 2. APPLY NEW PID
                controller->resetForPidTest();
                controller->setRampType(cmd.rampType);

                if (cmd.motor != MOTOR_SIDE_BOTH)
                {
                    if (cmd.motor == MOTOR_SIDE_LEFT)
                        controller->updatePids(MOTOR_SIDE_LEFT, cmd.kp, cmd.ki, cmd.kff, cmd.alpha);
                    else
                        controller->updatePids(MOTOR_SIDE_RIGHT, cmd.kp, cmd.ki, cmd.kff,
                                               cmd.alpha);
                }

                vTaskDelay(pdMS_TO_TICKS(150));

                // 3. START TEST
                if (cmd.motor != MOTOR_SIDE_BOTH)
                {
                    if (cmd.motor == MOTOR_SIDE_LEFT)
                        controller->setRps(cmd.testRps, 0.0f);
                    else
                        controller->setRps(0.0f, cmd.testRps);
                }
                else
                {
                    controller->setRps(cmd.testRps, cmd.testRps);
                }

                vTaskDelay(pdMS_TO_TICKS(10000));

                // 4. STOP AGAIN
                controller->hardStop();
            }
        }
    }
};

#endif