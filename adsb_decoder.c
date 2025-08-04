#include <stdint.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <time.h>

#include "adsb_decoder.h"

/* Definitions from dump1090.c needed for linking to its internal functions */

#define ANET_ERR_LEN 256
#define MODES_NET_MAX_FD 1024
#define MODES_CLIENT_BUF_SIZE 1024
#define MODES_LONG_MSG_BYTES (112/8)

struct client {
    int fd;
    int service;
    char buf[MODES_CLIENT_BUF_SIZE+1];
    int buflen;
};

struct aircraft {
    uint32_t addr;
    char hexaddr[7];
    char flight[9];
    int altitude;
    int speed;
    int track;
    time_t seen;
    long messages;
    int odd_cprlat;
    int odd_cprlon;
    int even_cprlat;
    int even_cprlon;
    double lat, lon;
    long long odd_cprtime, even_cprtime;
    struct aircraft *next;
};

/* Forward declaration of RTL-SDR type */
typedef struct rtlsdr_dev rtlsdr_dev_t;

struct modesMessage {
    unsigned char msg[MODES_LONG_MSG_BYTES];
    int msgbits;
    int msgtype;
    int crcok;
    uint32_t crc;
    int errorbit;
    int aa1, aa2, aa3;
    int phase_corrected;
    int ca;
    int metype;
    int mesub;
    int heading_is_valid;
    int heading;
    int aircraft_type;
    int fflag;
    int tflag;
    int raw_latitude;
    int raw_longitude;
    char flight[9];
    int ew_dir;
    int ew_velocity;
    int ns_dir;
    int ns_velocity;
    int vert_rate_source;
    int vert_rate_sign;
    int vert_rate;
    int velocity;
    int fs;
    int dr;
    int um;
    int identity;
    int altitude, unit;
};

struct modesState {
    pthread_t reader_thread;
    pthread_mutex_t data_mutex;
    pthread_cond_t data_cond;
    unsigned char *data;
    uint16_t *magnitude;
    uint32_t data_len;
    int fd;
    int data_ready;
    uint32_t *icao_cache;
    uint16_t *maglut;
    int exit;
    int dev_index;
    int gain;
    int enable_agc;
    rtlsdr_dev_t *dev;
    int freq;
    char aneterr[ANET_ERR_LEN];
    struct client *clients[MODES_NET_MAX_FD];
    int maxfd;
    int sbsos;
    int ros;
    int ris;
    int https;
    char *filename;
    int loop;
    int fix_errors;
    int check_crc;
    int raw;
    int debug;
    int net;
    int net_only;
    int interactive;
    int interactive_rows;
    int interactive_ttl;
    int stats;
    int onlyaddr;
    int metric;
    int aggressive;
    struct aircraft *aircrafts;
    long long interactive_last_update;
    long long stat_valid_preamble;
    long long stat_demodulated;
    long long stat_goodcrc;
    long long stat_badcrc;
    long long stat_fixed;
    long long stat_single_bit_fix;
    long long stat_two_bits_fix;
    long long stat_http_requests;
    long long stat_sbs_connections;
    long long stat_out_of_phase;
};

extern struct modesState Modes;

/* External functions from dump1090.c */
void modesInitConfig(void);
void modesInit(void);
void computeMagnitudeVector(void);
void detectModeS(uint16_t *m, uint32_t mlen);
int cprModFunction(int a, int b);
int cprNLFunction(double lat);
int cprNFunction(double lat, int isodd);
double cprDlonFunction(double lat, int isodd);

/* ======================================================================= */
/* Local helpers                                                           */
/* ======================================================================= */

static struct ADSBResult *decoder_out;
static float decoder_rx_lat;
static float decoder_rx_lon;
static int decoder_got_result = 0;

static int decodeCPRRelative(struct modesMessage *mm, double rx_lat,
                             double rx_lon, double *out_lat, double *out_lon) {
    int fflag = mm->fflag;
    double cprlat = mm->raw_latitude / 131072.0;
    double cprlon = mm->raw_longitude / 131072.0;
    double dlat = fflag ? (360.0/59.0) : (360.0/60.0);
    double j = floor(rx_lat / dlat) + fflag;
    double lat = dlat * (cprModFunction((int)j, fflag ? 59 : 60) + cprlat);
    if (lat >= 270) lat -= 360;
    int nl = cprNLFunction(lat) - fflag;
    if (nl < 1) return 0;
    double dlon = 360.0 / nl;
    double m = floor(rx_lon / dlon) + cprlon;
    double lon = dlon * (cprModFunction((int)m, nl) + (m - floor(m)));
    if (lon > 180) lon -= 360;
    *out_lat = lat;
    *out_lon = lon;
    return 1;
}

/* Override of dump1090's useModesMessage to capture the first message */
void useModesMessage(struct modesMessage *mm) {
    if (decoder_got_result || decoder_out == NULL) return;

    memset(decoder_out, 0, sizeof(*decoder_out));
    decoder_out->icao = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3;

    if (mm->msgtype == 17) {
        if (mm->metype >= 1 && mm->metype <= 4) {
            memcpy(decoder_out->callsign, mm->flight, sizeof(decoder_out->callsign));
            decoder_out->valid_callsign = 1;
        }
        if (mm->metype >= 9 && mm->metype <= 18) {
            decoder_out->altitude = mm->altitude;
            decoder_out->valid_altitude = 1;
            double lat, lon;
            if (decodeCPRRelative(mm, decoder_rx_lat, decoder_rx_lon, &lat, &lon)) {
                decoder_out->lat = lat;
                decoder_out->lon = lon;
                decoder_out->valid_position = 1;
            }
        }
    } else if (mm->msgtype == 0 || mm->msgtype == 4 || mm->msgtype == 20) {
        decoder_out->altitude = mm->altitude;
        decoder_out->valid_altitude = 1;
    }
    decoder_got_result = 1;
}

/* ======================================================================= */
/* Public API                                                              */
/* ======================================================================= */

int decode_adsb_frame(const uint8_t *iq_samples, int sample_count,
                      float receiver_lat, float receiver_lon,
                      struct ADSBResult *out) {
    if (!iq_samples || !out || sample_count <= 0) return 0;

    static int initialized = 0;
    if (!initialized) {
        modesInitConfig();
        modesInit();
        initialized = 1;
    }

    int bytes = sample_count * 2;
    if ((uint32_t)bytes > Modes.data_len) bytes = Modes.data_len;
    memcpy(Modes.data, iq_samples, bytes);
    Modes.data_len = bytes;
    computeMagnitudeVector();

    decoder_out = out;
    decoder_rx_lat = receiver_lat;
    decoder_rx_lon = receiver_lon;
    decoder_got_result = 0;
    detectModeS(Modes.magnitude, Modes.data_len/2);
    return decoder_got_result;
}

