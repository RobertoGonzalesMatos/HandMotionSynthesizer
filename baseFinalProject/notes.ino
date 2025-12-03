#include <math.h>

const char* NOTE12[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

int hzToMidi(int hz) {
    if (hz <= 0) return 0;
    float n = 69.0f + 12.0f * (log((float)hz / 440.0f) / log(2.0f));
    return (int)lroundf(n);
}
