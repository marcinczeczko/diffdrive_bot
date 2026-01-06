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
    void enqueue(const PidSideTestCommand& cmd)
    {
        PidTestCommand wrapper{};
        wrapper.type = PidTestCmdType::SIDE;
        wrapper.side = cmd;

        xQueueOverwrite(queue, &wrapper);
    }

    void enqueue(const PidTestAllCommand& cmd)
    {
        PidTestCommand wrapper{};
        wrapper.type = PidTestCmdType::BOTH;
        wrapper.both = cmd;

        xQueueOverwrite(queue, &wrapper);
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
            if (xQueueReceive(queue, &cmd, portMAX_DELAY) != pdTRUE)
                continue;

            // =================================================
            // 1. HARD STOP + RESET
            // =================================================
            controller->hardStop();
            vTaskDelay(pdMS_TO_TICKS(1000));

            controller->resetEncoders();
            controller->resetForPidTest();

            // =================================================
            // 2. APPLY CONFIG (SIDE vs BOTH)
            // =================================================
            if (cmd.type == PidTestCmdType::SIDE)
            {
                const auto& c = cmd.side;

                controller->resetMotorState(c.motor);
                controller->setRampType(c.rampType);

                if (c.motor == MOTOR_SIDE_LEFT)
                {
                    controller->updatePids(MOTOR_SIDE_LEFT, c.kp, c.ki, c.kff, c.alpha);
                }
                else if (c.motor == MOTOR_SIDE_RIGHT)
                {
                    controller->updatePids(MOTOR_SIDE_RIGHT, c.kp, c.ki, c.kff, c.alpha);
                }

                vTaskDelay(pdMS_TO_TICKS(150));

                // =================================================
                // 3. START TEST – SINGLE MOTOR
                // =================================================
                if (c.motor == MOTOR_SIDE_LEFT)
                    controller->setRps(c.testRps, 0.0f);
                else
                    controller->setRps(0.0f, c.testRps);
            }
            else // ===== BOTH MOTORS =====
            {
                const auto& c = cmd.both;

                controller->resetMotorState(MOTOR_SIDE_BOTH);
                controller->setRampType(c.l_rampType);

                // Apply PID LEFT
                controller->updatePids(MOTOR_SIDE_LEFT, c.l_kp, c.l_ki, c.l_kff, c.l_alpha);

                // Apply PID RIGHT
                controller->updatePids(MOTOR_SIDE_RIGHT, c.r_kp, c.r_ki, c.r_kff, c.r_alpha);

                vTaskDelay(pdMS_TO_TICKS(150));

                // =================================================
                // 3. START TEST – BOTH MOTORS
                // =================================================
                controller->setRps(c.l_testRps, c.r_testRps);
            }

            // =================================================
            // 4. RUN WINDOW
            // =================================================
            vTaskDelay(pdMS_TO_TICKS(10000));

            // =================================================
            // 5. STOP
            // =================================================
            controller->hardStop();
        }
    }
};

#endif