#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "adsb_decoder.h"

#define MODES_DATA_LEN             (16*16384)
#define MODES_PREAMBLE_US          8
#define MODES_LONG_MSG_BITS        112
#define MODES_SHORT_MSG_BITS       56
#define MODES_FULL_LEN             (MODES_PREAMBLE_US+MODES_LONG_MSG_BITS)
#define MODES_LONG_MSG_BYTES       (112/8)
#define MODES_SHORT_MSG_BYTES      (56/8)
#define MODES_UNIT_FEET            0
#define MODES_UNIT_METERS          1

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
    unsigned char *data;
    uint16_t *magnitude;
    uint32_t data_len;
    uint16_t *maglut;
    int fix_errors;
    int check_crc;
    int aggressive;
};

static struct modesState Modes;

/* ======================================================================= */
/* CRC and error correction                                                 */
/* ======================================================================= */

static uint32_t modes_checksum_table[112] = {
0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

static uint32_t modesChecksum(unsigned char *msg, int bits) {
    uint32_t crc = 0;
    int offset = (bits == 112) ? 0 : (112-56);
    int j;
    for (j = 0; j < bits; j++) {
        int byte = j/8;
        int bit = j%8;
        int bitmask = 1 << (7-bit);
        if (msg[byte] & bitmask)
            crc ^= modes_checksum_table[j+offset];
    }
    return crc;
}

static int modesMessageLenByType(int type) {
    if (type == 16 || type == 17 ||
        type == 19 || type == 20 ||
        type == 21)
        return MODES_LONG_MSG_BITS;
    else
        return MODES_SHORT_MSG_BITS;
}

static int fixSingleBitErrors(unsigned char *msg, int bits) {
    int j;
    unsigned char aux[MODES_LONG_MSG_BITS/8];
    for (j = 0; j < bits; j++) {
        int byte = j/8;
        int bitmask = 1 << (7-(j%8));
        uint32_t crc1, crc2;
        memcpy(aux,msg,bits/8);
        aux[byte] ^= bitmask;
        crc1 = ((uint32_t)aux[(bits/8)-3] << 16) |
               ((uint32_t)aux[(bits/8)-2] << 8) |
                (uint32_t)aux[(bits/8)-1];
        crc2 = modesChecksum(aux,bits);
        if (crc1 == crc2) {
            memcpy(msg,aux,bits/8);
            return j;
        }
    }
    return -1;
}

static int fixTwoBitsErrors(unsigned char *msg, int bits) {
    int j,i;
    unsigned char aux[MODES_LONG_MSG_BITS/8];
    for (j = 0; j < bits; j++) {
        int byte1 = j/8;
        int bitmask1 = 1 << (7-(j%8));
        for (i = j+1; i < bits; i++) {
            int byte2 = i/8;
            int bitmask2 = 1 << (7-(i%8));
            uint32_t crc1, crc2;
            memcpy(aux,msg,bits/8);
            aux[byte1] ^= bitmask1;
            aux[byte2] ^= bitmask2;
            crc1 = ((uint32_t)aux[(bits/8)-3] << 16) |
                   ((uint32_t)aux[(bits/8)-2] << 8) |
                    (uint32_t)aux[(bits/8)-1];
            crc2 = modesChecksum(aux,bits);
            if (crc1 == crc2) {
                memcpy(msg,aux,bits/8);
                return j | (i<<8);
            }
        }
    }
    return -1;
}

/* ======================================================================= */
/* Altitude decoding                                                        */
/* ======================================================================= */

static int decodeAC13Field(unsigned char *msg, int *unit) {
    int m_bit = msg[3] & (1<<6);
    int q_bit = msg[3] & (1<<4);
    if (!m_bit) {
        *unit = MODES_UNIT_FEET;
        if (q_bit) {
            int n = ((msg[2]&31)<<6) |
                    ((msg[3]&0x80)>>2) |
                    ((msg[3]&0x20)>>1) |
                     (msg[3]&15);
            return n*25-1000;
        }
    } else {
        *unit = MODES_UNIT_METERS;
    }
    return 0;
}

static int decodeAC12Field(unsigned char *msg, int *unit) {
    int q_bit = msg[5] & 1;
    if (q_bit) {
        *unit = MODES_UNIT_FEET;
        int n = ((msg[5]>>1)<<4) | ((msg[6]&0xF0) >> 4);
        return n*25-1000;
    } else {
        return 0;
    }
}

/* ======================================================================= */
/* CPR helpers                                                              */
/* ======================================================================= */

int cprModFunction(int a, int b) {
    int res = a % b;
    if (res < 0) res += b;
    return res;
}

int cprNLFunction(double lat) {
    if (lat < 0) lat = -lat;
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}

int cprNFunction(double lat, int isodd) {
    int nl = cprNLFunction(lat) - isodd;
    if (nl < 1) nl = 1;
    return nl;
}

double cprDlonFunction(double lat, int isodd) {
    return 360.0 / cprNFunction(lat, isodd);
}

/* ======================================================================= */
/* Message decoding                                                         */
/* ======================================================================= */

static void decodeModesMessage(struct modesMessage *mm, unsigned char *msg) {
    uint32_t crc2;
    char *ais_charset = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";
    memcpy(mm->msg,msg,MODES_LONG_MSG_BYTES);
    msg = mm->msg;
    mm->msgtype = msg[0]>>3;
    mm->msgbits = modesMessageLenByType(mm->msgtype);
    mm->crc = ((uint32_t)msg[(mm->msgbits/8)-3] << 16) |
              ((uint32_t)msg[(mm->msgbits/8)-2] << 8) |
               (uint32_t)msg[(mm->msgbits/8)-1];
    crc2 = modesChecksum(msg,mm->msgbits);
    mm->errorbit = -1;
    mm->crcok = (mm->crc == crc2);
    if (!mm->crcok && Modes.fix_errors &&
        (mm->msgtype == 11 || mm->msgtype == 17)) {
        if ((mm->errorbit = fixSingleBitErrors(msg,mm->msgbits)) != -1) {
            mm->crc = modesChecksum(msg,mm->msgbits);
            mm->crcok = 1;
        } else if (Modes.aggressive && mm->msgtype == 17 &&
                   (mm->errorbit = fixTwoBitsErrors(msg,mm->msgbits)) != -1) {
            mm->crc = modesChecksum(msg,mm->msgbits);
            mm->crcok = 1;
        }
    }
    mm->ca = msg[0] & 7;
    mm->aa1 = msg[1];
    mm->aa2 = msg[2];
    mm->aa3 = msg[3];
    mm->metype = msg[4] >> 3;
    mm->mesub = msg[4] & 7;
    mm->fs = msg[0] & 7;
    mm->dr = msg[1] >> 3 & 31;
    mm->um = ((msg[1] & 7)<<3)| msg[2]>>5;
    {
        int a,b,c,d;
        a = ((msg[3] & 0x80) >> 5) |
            ((msg[2] & 0x02) >> 0) |
            ((msg[2] & 0x08) >> 3);
        b = ((msg[3] & 0x02) << 1) |
            ((msg[3] & 0x08) >> 2) |
            ((msg[3] & 0x20) >> 5);
        c = ((msg[2] & 0x01) << 2) |
            ((msg[2] & 0x04) >> 1) |
            ((msg[2] & 0x10) >> 4);
        d = ((msg[3] & 0x01) << 2) |
            ((msg[3] & 0x04) >> 1) |
            ((msg[3] & 0x10) >> 4);
        mm->identity = a*1000 + b*100 + c*10 + d;
    }
    if (mm->msgtype == 0 || mm->msgtype == 4 ||
        mm->msgtype == 16 || mm->msgtype == 20) {
        mm->altitude = decodeAC13Field(msg, &mm->unit);
    }
    if (mm->msgtype == 17) {
        if (mm->metype >= 1 && mm->metype <= 4) {
            mm->aircraft_type = mm->metype-1;
            mm->flight[0] = ais_charset[msg[5]>>2];
            mm->flight[1] = ais_charset[((msg[5]&3)<<4)|(msg[6]>>4)];
            mm->flight[2] = ais_charset[((msg[6]&15)<<2)|(msg[7]>>6)];
            mm->flight[3] = ais_charset[msg[7]&63];
            mm->flight[4] = ais_charset[msg[8]>>2];
            mm->flight[5] = ais_charset[((msg[8]&3)<<4)|(msg[9]>>4)];
            mm->flight[6] = ais_charset[((msg[9]&15)<<2)|(msg[10]>>6)];
            mm->flight[7] = ais_charset[msg[10]&63];
            mm->flight[8] = '\0';
        } else if (mm->metype >= 9 && mm->metype <= 18) {
            mm->fflag = msg[6] & (1<<2);
            mm->tflag = msg[6] & (1<<3);
            mm->altitude = decodeAC12Field(msg,&mm->unit);
            mm->raw_latitude = ((msg[6] & 3) << 15) |
                                (msg[7] << 7) |
                                (msg[8] >> 1);
            mm->raw_longitude = ((msg[8]&1) << 16) |
                                 (msg[9] << 8) |
                                 msg[10];
        } else if (mm->metype == 19 && mm->mesub >= 1 && mm->mesub <= 4) {
            if (mm->mesub == 1 || mm->mesub == 2) {
                mm->ew_dir = (msg[5]&4) >> 2;
                mm->ew_velocity = ((msg[5]&3) << 8) | msg[6];
                mm->ns_dir = (msg[7]&0x80) >> 7;
                mm->ns_velocity = ((msg[7]&0x7f) << 3) | ((msg[8]&0xe0) >> 5);
                mm->vert_rate_source = (msg[8]&0x10) >> 4;
                mm->vert_rate_sign = (msg[8]&0x8) >> 3;
                mm->vert_rate = ((msg[8]&7) << 6) | ((msg[9]&0xfc) >> 2);
                mm->velocity = sqrt(mm->ns_velocity*mm->ns_velocity+
                                    mm->ew_velocity*mm->ew_velocity);
                if (mm->velocity) {
                    int ewv = mm->ew_velocity;
                    int nsv = mm->ns_velocity;
                    double heading;
                    if (mm->ew_dir) ewv *= -1;
                    if (mm->ns_dir) nsv *= -1;
                    heading = atan2(ewv,nsv);
                    mm->heading = heading * 360 / (M_PI*2);
                    if (mm->heading < 0) mm->heading += 360;
                } else {
                    mm->heading = 0;
                }
            } else if (mm->mesub == 3 || mm->mesub == 4) {
                mm->heading_is_valid = msg[5] & (1<<2);
                mm->heading = (360.0/128) * (((msg[5] & 3) << 5) |
                                              (msg[6] >> 3));
            }
        }
    }
    mm->phase_corrected = 0;
}

/* ======================================================================= */
/* Mode S message detection                                                 */
/* ======================================================================= */

static int detectOutOfPhase(uint16_t *m) {
    if (m[3] > m[2]/3) return 1;
    if (m[10] > m[9]/3) return 1;
    if (m[6] > m[7]/3) return -1;
    if (m[-1] > m[1]/3) return -1;
    return 0;
}

static void computeMagnitudeVector(void) {
    uint16_t *m = Modes.magnitude;
    unsigned char *p = Modes.data;
    uint32_t j;
    for (j = 0; j < Modes.data_len; j += 2) {
        int i = p[j]-127;
        int q = p[j+1]-127;
        if (i < 0) i = -i;
        if (q < 0) q = -q;
        m[j/2] = Modes.maglut[i*129+q];
    }
}

static void useModesMessage(struct modesMessage *mm);

static void detectModeS(uint16_t *m, uint32_t mlen) {
    unsigned char bits[MODES_LONG_MSG_BITS];
    unsigned char msg[MODES_LONG_MSG_BITS/2];
    uint16_t aux[MODES_LONG_MSG_BITS*2];
    uint32_t j;
    int use_correction = 0;
    for (j = 0; j < mlen - MODES_FULL_LEN*2; j++) {
        int low, high, delta, i, errors;
        int good_message = 0;
        if (use_correction) goto good_preamble;
        if (!(m[j] > m[j+1] &&
            m[j+1] < m[j+2] &&
            m[j+2] > m[j+3] &&
            m[j+3] < m[j] &&
            m[j+4] < m[j] &&
            m[j+5] < m[j] &&
            m[j+6] < m[j] &&
            m[j+7] > m[j+8] &&
            m[j+8] < m[j+9] &&
            m[j+9] > m[j+6])) {
            continue;
        }
        high = (m[j]+m[j+2]+m[j+7]+m[j+9])/6;
        if (m[j+4] >= high ||
            m[j+5] >= high) {
            continue;
        }
        if (m[j+11] >= high ||
            m[j+12] >= high ||
            m[j+13] >= high ||
            m[j+14] >= high) {
            continue;
        }
        int oit = detectOutOfPhase(m+j);
        if (oit) {
            if (oit > 0) {
                if (j < MODES_PREAMBLE_US*2) continue;
                j -= oit;
            }
            m[j+MODES_PREAMBLE_US*2-oit] = m[j+MODES_PREAMBLE_US*2-oit-1];
            m[j+MODES_PREAMBLE_US*2+1-oit] = m[j+MODES_PREAMBLE_US*2-oit];
            use_correction = 1;
        }
        good_preamble:
        errors = 0;
        if (use_correction)
            memcpy(aux,m+j+MODES_PREAMBLE_US*2,sizeof(aux));
        for (i = 0; i < MODES_LONG_MSG_BITS*2; i += 2) {
            low = m[j+MODES_PREAMBLE_US*2+i];
            high = m[j+MODES_PREAMBLE_US*2+i+1];
            if (use_correction && i > 0 &&
                m[j+MODES_PREAMBLE_US*2+i-1] > m[j+MODES_PREAMBLE_US*2+i] &&
                m[j+MODES_PREAMBLE_US*2+i-1] > m[j+MODES_PREAMBLE_US*2+i-2]) {
                low = m[j+MODES_PREAMBLE_US*2+i-1];
                high = low;
            }
            if (high - low < 255) {
                if (i < MODES_SHORT_MSG_BITS*2) errors++;
                bits[i/2] = 2;
            } else if (low > high) {
                bits[i/2] = 1;
            } else {
                bits[i/2] = 0;
            }
        }
        if (use_correction)
            memcpy(m+j+MODES_PREAMBLE_US*2,aux,sizeof(aux));
        for (i = 0; i < MODES_LONG_MSG_BITS; i += 8) {
            msg[i/8] =
                bits[i]<<7 |
                bits[i+1]<<6 |
                bits[i+2]<<5 |
                bits[i+3]<<4 |
                bits[i+4]<<3 |
                bits[i+5]<<2 |
                bits[i+6]<<1 |
                bits[i+7];
        }
        int msgtype = msg[0]>>3;
        int msglen = modesMessageLenByType(msgtype)/8;
        delta = 0;
        for (i = 0; i < msglen*8*2; i += 2) {
            delta += abs(m[j+i+MODES_PREAMBLE_US*2]-
                         m[j+i+MODES_PREAMBLE_US*2+1]);
        }
        delta /= msglen*4;
        if (delta < 10*255) {
            use_correction = 0;
            continue;
        }
        if (errors == 0 || (Modes.aggressive && errors < 3)) {
            struct modesMessage mm;
            decodeModesMessage(&mm,msg);
            if (mm.crcok) {
                j += (MODES_PREAMBLE_US+(msglen*8))*2;
                good_message = 1;
                if (use_correction)
                    mm.phase_corrected = 1;
            }
            useModesMessage(&mm);
        }
        if (!good_message && !use_correction) {
            j--;
            use_correction = 1;
        } else {
            use_correction = 0;
        }
    }
}

/* ======================================================================= */
/* Result capturing                                                         */
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

static void useModesMessage(struct modesMessage *mm) {
    if (decoder_got_result || decoder_out == NULL || !mm->crcok)
        return;
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
    }
    decoder_got_result = 1;
}

/* ======================================================================= */
/* Initialization and public API                                           */
/* ======================================================================= */

static void modesInitConfig(void) {
    Modes.fix_errors = 1;
    Modes.check_crc = 1;
    Modes.aggressive = 0;
}

static void modesInit(void) {
    int i, q;
    Modes.data_len = MODES_DATA_LEN + (MODES_FULL_LEN-1)*4;
    Modes.data = malloc(Modes.data_len);
    Modes.magnitude = malloc(Modes.data_len*2);
    Modes.maglut = malloc(129*129*2);
    for (i = 0; i <= 128; i++) {
        for (q = 0; q <= 128; q++) {
            Modes.maglut[i*129+q] = round(sqrt(i*i+q*q)*360);
        }
    }
    memset(Modes.data,127,Modes.data_len);
}

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

