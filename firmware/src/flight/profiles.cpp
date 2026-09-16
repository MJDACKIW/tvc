#include "profiles.h"

#include <math.h>

namespace tvc::flight::profiles {

void targetAngles(Profile p, float t, float& target_pitch_deg, float& target_yaw_deg) {
    target_pitch_deg = 0.0f;
    target_yaw_deg = 0.0f;

    switch (p) {
        case Profile::kHold:
            break;

        case Profile::kStepX:
        case Profile::kStepY: {
            float ph = fmodf(t, kStepPeriodS) / kStepPeriodS;
            float v = (ph < 0.5f) ? kAmpDeg : -kAmpDeg;
            if (p == Profile::kStepX) target_pitch_deg = v;
            else target_yaw_deg = v;
            break;
        }

        case Profile::kRampX: {
            float ph = fmodf(t, kRampPeriodS) / kRampPeriodS;  // 0..1
            target_pitch_deg = kAmpDeg * (4.0f * fabsf(ph - 0.5f) - 1.0f);  // triangle wave
            break;
        }

        case Profile::kChirpX: {
            // Logarithmic sweep kChirpF0Hz -> kChirpF1Hz over kChirpDurS. Phase is the
            // integral of instantaneous frequency (legacy comment: getting this wrong smears
            // the frequency axis of the resulting Bode estimate).
            float k = powf(kChirpF1Hz / kChirpF0Hz, 1.0f / kChirpDurS);
            float u = fminf(t, kChirpDurS);
            float phase = 2.0f * static_cast<float>(M_PI) * kChirpF0Hz * (powf(k, u) - 1.0f) / logf(k);
            target_pitch_deg = kAmpDeg * sinf(phase);
            break;
        }

        default:
            break;
    }
}

}  // namespace tvc::flight::profiles
