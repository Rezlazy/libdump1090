#ifndef ADSB_DECODER_H
#define ADSB_DECODER_H

#include <stdint.h>

struct ADSBResult {
    uint32_t icao;
    char callsign[9]; /* null-terminated */
    float lat;
    float lon;
    int altitude;
    int valid_position;
    int valid_altitude;
    int valid_callsign;
};

int decode_adsb_frame(const uint8_t *iq_samples, int sample_count,
                      float receiver_lat, float receiver_lon,
                      struct ADSBResult *out);

#endif /* ADSB_DECODER_H */
