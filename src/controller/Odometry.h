#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "config/Config.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <math.h>

struct Pose
{
    float x;     // cm
    float y;     // cm
    float theta; // radians
};

class Odometry
{
  private:
    Pose pose;

    SemaphoreHandle_t syncSemaphore;

  public:
    Odometry() : pose({0, 0, 0})
    {
    }

    void begin()
    {
        syncSemaphore = xSemaphoreCreateBinary();
        if (syncSemaphore != nullptr)
        {
            xSemaphoreGive(syncSemaphore); // Binary semaphores start at 0 (locked)
        }
    }

    void update(float dDist, float dTheta)
    {
        if (xSemaphoreTake(syncSemaphore, 0))
        { // Nie blokujemy taska PID, jeśli zajęty
            // Obliczamy kąt w połowie drogi
            float midTheta = pose.theta + (dTheta / 2.0F);

            // Aktualizacja pozycji przy użyciu średniego kąta
            pose.x += dDist * cos(midTheta);
            pose.y += dDist * sin(midTheta);
            pose.theta += dTheta;

            // Normalize degree -PI to PI
            while (pose.theta > PI)
            {
                pose.theta -= 2.0F * PI;
            }
            while (pose.theta < -PI)
            {
                pose.theta += 2.0F * PI;
            }
            // LOG_DATA_3("ODO", "X,Y,THETA", pose.x, pose.x, pose.theta);
            xSemaphoreGive(syncSemaphore);
        }
    }

    Pose getPose()
    {
        Pose temp = {0, 0, 0};
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY))
        {
            temp = pose;
            xSemaphoreGive(syncSemaphore);
        }
        return temp;
    }

    void reset()
    {
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY))
        {
            pose = {0, 0, 0};
            xSemaphoreGive(syncSemaphore);
        }
    }
};

#endif