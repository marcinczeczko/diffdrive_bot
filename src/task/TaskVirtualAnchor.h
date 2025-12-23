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

        // 1. "Rzucamy kotwicę" w obecnym miejscu
        Pose anchor = odo.getPose();
        LOG_INFO("Anchor set! Try to move me.");

        // Parametry "sprężyny" (P-Controller)
        const float Kp_lin = 1.5f;   // Siła powrotu liniowego
        const float Kp_ang = 2.0f;   // Siła powrotu obrotowego
        const float MAX_V = 15.0f;   // Max prędkość powrotu [cm/s]
        const float MAX_W = 2.0f;    // Max prędkość obrotu [rad/s]
        const float DEADZONE = 0.5f; // Tolerancja 0.5 cm

        for (;;)
        {
            Pose current = odo.getPose();

            // 2. Obliczamy błędy pozycji
            float errorX = anchor.x - current.x;
            float errorY = anchor.y - current.y;
            float errorTheta = anchor.theta - current.theta;

            // Normalizacja błędu kąta do zakresu -PI..PI
            while (errorTheta > PI)
                errorTheta -= 2.0f * PI;
            while (errorTheta < -PI)
                errorTheta += 2.0f * PI;

            // 3. Obliczamy dystans do punktu zakotwiczenia
            float distance = sqrt(errorX * errorX + errorY * errorY);

            float v = 0;
            float w = 0;

            // 4. Logika powrotu (uproszczona - robot dąży do X,Y i Theta niezależnie)
            if (distance > DEADZONE || abs(errorTheta) > 0.05f)
            {

                // Prędkość liniowa (proporcjonalna do błędu)
                // Uproszczenie: robot jedzie przód/tył bazując na errorX w swoim układzie
                // Dla pełnego 2D należałoby obrócić wektor błędu do układu robota
                float localErrorX = errorX * cos(current.theta) + errorY * sin(current.theta);
                v = localErrorX * Kp_lin;

                // Prędkość kątowa
                w = errorTheta * Kp_ang;

                // 5. Ograniczenia prędkości (Saturacja)
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
                // Jesteśmy w domu - luzujemy silniki
                vc->setTwist(0, 0);
            }

            vTaskDelay(pdMS_TO_TICKS(50)); // Sprawdzaj 20 razy na sekundę
        }
    }
};

#endif