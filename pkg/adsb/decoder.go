package adsb

import (
	"errors"
	"math"
)

const (
	modesDataLen       = 16 * 16384
	modesPreambleUS    = 8
	modesLongMsgBits   = 112
	modesShortMsgBits  = 56
	modesFullLen       = modesPreambleUS + modesLongMsgBits
	modesLongMsgBytes  = 112 / 8
	modesShortMsgBytes = 56 / 8
	modesUnitFeet      = 0
	modesUnitMeters    = 1
)

type modesMessage struct {
	msg            [modesLongMsgBytes]byte
	msgbits        int
	msgtype        int
	crcok          bool
	crc            uint32
	errorbit       int
	aa1, aa2, aa3  int
	phaseCorrected bool
	ca             int
	metype         int
	mesub          int
	headingIsValid bool
	heading        float64
	aircraftType   int
	fflag          int
	tflag          int
	rawLatitude    int
	rawLongitude   int
	flight         [9]byte
	ewDir          int
	ewVelocity     int
	nsDir          int
	nsVelocity     int
	vertRateSource int
	vertRateSign   int
	vertRate       int
	velocity       int
	fs             int
	dr             int
	um             int
	identity       int
	altitude       int
	unit           int
}

type modesState struct {
	data       []byte
	magnitude  []uint16
	dataLen    uint32
	maglut     []uint16
	fixErrors  bool
	checkCRC   bool
	aggressive bool
}

var modes modesState

// ADSBResult contains the decoded information from a single frame.
type ADSBResult struct {
	ICAO          uint32
	Callsign      string
	Lat           float64
	Lon           float64
	Altitude      int
	ValidPosition bool
	ValidAltitude bool
	ValidCallsign bool
}

// Decoder provides ADS-B frame decoding capabilities.
type Decoder struct {
	rxLat float64
	rxLon float64
}

// NewDecoder creates a new Decoder configured with receiver coordinates.
func NewDecoder(rxLat, rxLon float64) *Decoder {
	modesInitConfig()
	return &Decoder{rxLat: rxLat, rxLon: rxLon}
}

var modesChecksumTable = [112]uint32{
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
	0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
}

func modesChecksum(msg []byte, bits int) uint32 {
	var crc uint32
	offset := 0
	if bits != 112 {
		offset = 112 - 56
	}
	for j := 0; j < bits; j++ {
		byteIndex := j / 8
		bit := j % 8
		bitmask := byte(1 << (7 - bit))
		if msg[byteIndex]&bitmask != 0 {
			crc ^= modesChecksumTable[j+offset]
		}
	}
	return crc
}

func modesMessageLenByType(t int) int {
	if t == 16 || t == 17 || t == 19 || t == 20 || t == 21 {
		return modesLongMsgBits
	}
	return modesShortMsgBits
}

func fixSingleBitErrors(msg []byte, bits int) int {
	aux := make([]byte, bits/8)
	for j := 0; j < bits; j++ {
		byteIndex := j / 8
		bitmask := byte(1 << (7 - (j % 8)))
		copy(aux, msg[:bits/8])
		aux[byteIndex] ^= bitmask
		crc1 := uint32(aux[bits/8-3])<<16 | uint32(aux[bits/8-2])<<8 | uint32(aux[bits/8-1])
		crc2 := modesChecksum(aux, bits)
		if crc1 == crc2 {
			copy(msg, aux[:bits/8])
			return j
		}
	}
	return -1
}

func fixTwoBitsErrors(msg []byte, bits int) int {
	aux := make([]byte, bits/8)
	for j := 0; j < bits; j++ {
		byte1 := j / 8
		bitmask1 := byte(1 << (7 - (j % 8)))
		for i := j + 1; i < bits; i++ {
			byte2 := i / 8
			bitmask2 := byte(1 << (7 - (i % 8)))
			copy(aux, msg[:bits/8])
			aux[byte1] ^= bitmask1
			aux[byte2] ^= bitmask2
			crc1 := uint32(aux[bits/8-3])<<16 | uint32(aux[bits/8-2])<<8 | uint32(aux[bits/8-1])
			crc2 := modesChecksum(aux, bits)
			if crc1 == crc2 {
				copy(msg, aux[:bits/8])
				return j | (i << 8)
			}
		}
	}
	return -1
}

func decodeAC13Field(msg []byte, unit *int) int {
	mBit := msg[3] & (1 << 6)
	qBit := msg[3] & (1 << 4)
	if mBit == 0 {
		*unit = modesUnitFeet
		if qBit != 0 {
			n := int((msg[2]&31)<<6 | ((msg[3] & 0x80) >> 2) | ((msg[3] & 0x20) >> 1) | (msg[3] & 15))
			return n*25 - 1000
		}
	} else {
		*unit = modesUnitMeters
	}
	return 0
}

func decodeAC12Field(msg []byte, unit *int) int {
	qBit := msg[5] & 1
	if qBit != 0 {
		*unit = modesUnitFeet
		n := int(((msg[5] >> 1) << 4) | ((msg[6] & 0xF0) >> 4))
		return n*25 - 1000
	}
	return 0
}

func cprModFunction(a, b int) int {
	res := a % b
	if res < 0 {
		res += b
	}
	return res
}

func cprNLFunction(lat float64) int {
	if lat < 0 {
		lat = -lat
	}
	switch {
	case lat < 10.47047130:
		return 59
	case lat < 14.82817437:
		return 58
	case lat < 18.18626357:
		return 57
	case lat < 21.02939493:
		return 56
	case lat < 23.54504487:
		return 55
	case lat < 25.82924707:
		return 54
	case lat < 27.93898710:
		return 53
	case lat < 29.91135686:
		return 52
	case lat < 31.77209708:
		return 51
	case lat < 33.53993436:
		return 50
	case lat < 35.22899598:
		return 49
	case lat < 36.85025108:
		return 48
	case lat < 38.41241892:
		return 47
	case lat < 39.92256684:
		return 46
	case lat < 41.38651832:
		return 45
	case lat < 42.80914012:
		return 44
	case lat < 44.19454951:
		return 43
	case lat < 45.54626723:
		return 42
	case lat < 46.86733252:
		return 41
	case lat < 48.16039128:
		return 40
	case lat < 49.42776439:
		return 39
	case lat < 50.67150166:
		return 38
	case lat < 51.89342469:
		return 37
	case lat < 53.09516153:
		return 36
	case lat < 54.27817472:
		return 35
	case lat < 55.44378444:
		return 34
	case lat < 56.59318756:
		return 33
	case lat < 57.72747354:
		return 32
	case lat < 58.84763776:
		return 31
	case lat < 59.95459277:
		return 30
	case lat < 61.04917774:
		return 29
	case lat < 62.13216659:
		return 28
	case lat < 63.20427479:
		return 27
	case lat < 64.26616523:
		return 26
	case lat < 65.31845310:
		return 25
	case lat < 66.36171008:
		return 24
	case lat < 67.39646774:
		return 23
	case lat < 68.42322022:
		return 22
	case lat < 69.44242631:
		return 21
	case lat < 70.45451075:
		return 20
	case lat < 71.45986473:
		return 19
	case lat < 72.45884545:
		return 18
	case lat < 73.45177442:
		return 17
	case lat < 74.43893416:
		return 16
	case lat < 75.42056257:
		return 15
	case lat < 76.39684391:
		return 14
	case lat < 77.36789461:
		return 13
	case lat < 78.33374083:
		return 12
	case lat < 79.29428225:
		return 11
	case lat < 80.24923213:
		return 10
	case lat < 81.19801349:
		return 9
	case lat < 82.13956981:
		return 8
	case lat < 83.07199445:
		return 7
	case lat < 83.99173563:
		return 6
	case lat < 84.89166191:
		return 5
	case lat < 85.75541621:
		return 4
	case lat < 86.53536998:
		return 3
	case lat < 87.00000000:
		return 2
	default:
		return 1
	}
}

func cprNFunction(lat float64, isodd int) int {
	nl := cprNLFunction(lat) - isodd
	if nl < 1 {
		nl = 1
	}
	return nl
}

func cprDlonFunction(lat float64, isodd int) float64 {
	return 360.0 / float64(cprNFunction(lat, isodd))
}

func decodeModesMessage(mm *modesMessage, msg []byte) {
	copy(mm.msg[:], msg[:modesLongMsgBytes])
	msg = mm.msg[:]
	mm.msgtype = int(msg[0] >> 3)
	mm.msgbits = modesMessageLenByType(mm.msgtype)
	mm.crc = uint32(msg[(mm.msgbits/8)-3])<<16 | uint32(msg[(mm.msgbits/8)-2])<<8 | uint32(msg[(mm.msgbits/8)-1])
	crc2 := modesChecksum(msg, mm.msgbits)
	mm.errorbit = -1
	mm.crcok = mm.crc == crc2
	if !mm.crcok && modes.fixErrors && (mm.msgtype == 11 || mm.msgtype == 17) {
		if mm.errorbit = fixSingleBitErrors(msg, mm.msgbits); mm.errorbit != -1 {
			mm.crc = modesChecksum(msg, mm.msgbits)
			mm.crcok = true
		} else if modes.aggressive && mm.msgtype == 17 {
			if mm.errorbit = fixTwoBitsErrors(msg, mm.msgbits); mm.errorbit != -1 {
				mm.crc = modesChecksum(msg, mm.msgbits)
				mm.crcok = true
			}
		}
	}
	mm.ca = int(msg[0] & 7)
	mm.aa1 = int(msg[1])
	mm.aa2 = int(msg[2])
	mm.aa3 = int(msg[3])
	mm.metype = int(msg[4] >> 3)
	mm.mesub = int(msg[4] & 7)
	mm.fs = int(msg[0] & 7)
	mm.dr = int(msg[1]>>3) & 31
	mm.um = (int(msg[1]&7) << 3) | int(msg[2]>>5)
	a := ((int(msg[3]&0x80) >> 5) | (int(msg[2]&0x02) >> 0) | (int(msg[2]&0x08) >> 3))
	b := ((int(msg[3]&0x02) << 1) | (int(msg[3]&0x08) >> 2) | (int(msg[3]&0x20) >> 5))
	c := ((int(msg[2]&0x01) << 2) | (int(msg[2]&0x04) >> 1) | (int(msg[2]&0x10) >> 4))
	d := ((int(msg[3]&0x01) << 2) | (int(msg[3]&0x04) >> 1) | (int(msg[3]&0x10) >> 4))
	mm.identity = a*1000 + b*100 + c*10 + d
	if mm.msgtype == 0 || mm.msgtype == 4 || mm.msgtype == 16 || mm.msgtype == 20 {
		mm.altitude = decodeAC13Field(msg, &mm.unit)
	}
	if mm.msgtype == 17 {
		aisCharset := []byte("?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????")
		if mm.metype >= 1 && mm.metype <= 4 {
			mm.aircraftType = mm.metype - 1
			mm.flight[0] = aisCharset[msg[5]>>2]
			mm.flight[1] = aisCharset[((msg[5]&3)<<4)|(msg[6]>>4)]
			mm.flight[2] = aisCharset[((msg[6]&15)<<2)|(msg[7]>>6)]
			mm.flight[3] = aisCharset[msg[7]&63]
			mm.flight[4] = aisCharset[msg[8]>>2]
			mm.flight[5] = aisCharset[((msg[8]&3)<<4)|(msg[9]>>4)]
			mm.flight[6] = aisCharset[((msg[9]&15)<<2)|(msg[10]>>6)]
			mm.flight[7] = aisCharset[msg[10]&63]
			mm.flight[8] = 0
		} else if mm.metype >= 9 && mm.metype <= 18 {
			mm.fflag = int(msg[6] & (1 << 2))
			mm.tflag = int(msg[6] & (1 << 3))
			mm.altitude = decodeAC12Field(msg, &mm.unit)
			mm.rawLatitude = int(((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1))
			mm.rawLongitude = int(((msg[8] & 1) << 16) | (msg[9] << 8) | msg[10])
		} else if mm.metype == 19 && mm.mesub >= 1 && mm.mesub <= 4 {
			if mm.mesub == 1 || mm.mesub == 2 {
				mm.ewDir = int((msg[5] & 4) >> 2)
				mm.ewVelocity = int((msg[5]&3)<<8 | msg[6])
				mm.nsDir = int((msg[7] & 0x80) >> 7)
				mm.nsVelocity = int((msg[7]&0x7f)<<3 | ((msg[8] & 0xe0) >> 5))
				mm.vertRateSource = int((msg[8] & 0x10) >> 4)
				mm.vertRateSign = int((msg[8] & 0x8) >> 3)
				mm.vertRate = int((msg[8]&7)<<6 | ((msg[9] & 0xfc) >> 2))
				mm.velocity = int(math.Sqrt(float64(mm.nsVelocity*mm.nsVelocity + mm.ewVelocity*mm.ewVelocity)))
				if mm.velocity != 0 {
					ewv := mm.ewVelocity
					nsv := mm.nsVelocity
					if mm.ewDir != 0 {
						ewv = -ewv
					}
					if mm.nsDir != 0 {
						nsv = -nsv
					}
					heading := math.Atan2(float64(ewv), float64(nsv))
					mm.heading = heading * 360 / (math.Pi * 2)
					if mm.heading < 0 {
						mm.heading += 360
					}
				} else {
					mm.heading = 0
				}
			} else if mm.mesub == 3 || mm.mesub == 4 {
				mm.headingIsValid = (msg[5] & (1 << 2)) != 0
				mm.heading = (360.0 / 128) * float64(((msg[5]&3)<<5)|(msg[6]>>3))
			}
		}
	}
	mm.phaseCorrected = false
}

func detectOutOfPhase(m []uint16, j int) int {
	if m[j+3] > m[j+2]/3 {
		return 1
	}
	if m[j+10] > m[j+9]/3 {
		return 1
	}
	if m[j+6] > m[j+7]/3 {
		return -1
	}
	if j > 0 && m[j-1] > m[j+1]/3 {
		return -1
	}
	return 0
}

func computeMagnitudeVector() {
	m := modes.magnitude
	p := modes.data
	for j := 0; j < int(modes.dataLen); j += 2 {
		i := int(p[j]) - 127
		q := int(p[j+1]) - 127
		if i < 0 {
			i = -i
		}
		if q < 0 {
			q = -q
		}
		m[j/2] = modes.maglut[i*129+q]
	}
}

func detectModeS(m []uint16, mlen int, d *Decoder, out *ADSBResult, gotResult *bool) {
	var (
		bits [modesLongMsgBits]byte
		msg  [modesLongMsgBits / 2]byte
		aux  [modesLongMsgBits * 2]uint16
	)
	useCorrection := false
	for j := 0; j < mlen-modesFullLen*2; j++ {
		if !useCorrection {
			if !(m[j] > m[j+1] &&
				m[j+1] < m[j+2] &&
				m[j+2] > m[j+3] &&
				m[j+3] < m[j] &&
				m[j+4] < m[j] &&
				m[j+5] < m[j] &&
				m[j+6] < m[j] &&
				m[j+7] > m[j+8] &&
				m[j+8] < m[j+9] &&
				m[j+9] > m[j+6]) {
				continue
			}
			high := (m[j] + m[j+2] + m[j+7] + m[j+9]) / 6
			if m[j+4] >= high || m[j+5] >= high {
				continue
			}
			if m[j+11] >= high || m[j+12] >= high || m[j+13] >= high || m[j+14] >= high {
				continue
			}
			if oit := detectOutOfPhase(m, j); oit != 0 {
				if oit > 0 {
					if j < modesPreambleUS*2 {
						continue
					}
					j -= oit
				}
				idx := j + modesPreambleUS*2 - oit
				m[idx] = m[idx-1]
				m[idx+1] = m[idx]
				useCorrection = true
			}
		}

		errors := 0
		if useCorrection {
			copy(aux[:], m[j+modesPreambleUS*2:])
		}
		for i := 0; i < modesLongMsgBits*2; i += 2 {
			low := m[j+modesPreambleUS*2+i]
			high := m[j+modesPreambleUS*2+i+1]
			if useCorrection && i > 0 &&
				m[j+modesPreambleUS*2+i-1] > m[j+modesPreambleUS*2+i] &&
				m[j+modesPreambleUS*2+i-1] > m[j+modesPreambleUS*2+i-2] {
				low = m[j+modesPreambleUS*2+i-1]
				high = low
			}
			if high-low < 255 {
				if i < modesShortMsgBits*2 {
					errors++
				}
				bits[i/2] = 2
			} else if low > high {
				bits[i/2] = 1
			} else {
				bits[i/2] = 0
			}
		}
		if useCorrection {
			copy(m[j+modesPreambleUS*2:], aux[:])
		}
		for i := 0; i < modesLongMsgBits; i += 8 {
			msg[i/8] = bits[i]<<7 | bits[i+1]<<6 | bits[i+2]<<5 | bits[i+3]<<4 | bits[i+4]<<3 | bits[i+5]<<2 | bits[i+6]<<1 | bits[i+7]
		}
		msgtype := int(msg[0] >> 3)
		msglen := modesMessageLenByType(msgtype) / 8
		delta := 0
		for i := 0; i < msglen*8*2; i += 2 {
			delta += intAbs(int(m[j+i+modesPreambleUS*2]) - int(m[j+i+modesPreambleUS*2+1]))
		}
		delta /= msglen * 4
		if delta < 10*255 {
			useCorrection = false
			continue
		}
		if errors == 0 || (modes.aggressive && errors < 3) {
			var mm modesMessage
			decodeModesMessage(&mm, msg[:])
			if mm.crcok {
				j += (modesPreambleUS + (msglen * 8)) * 2
				if useCorrection {
					mm.phaseCorrected = true
				}
			}
			useModesMessage(d, out, gotResult, &mm)
		}
		if !*gotResult && !useCorrection {
			j--
			useCorrection = true
		} else {
			useCorrection = false
		}
	}
}

func intAbs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func decodeCPRRelative(mm *modesMessage, rxLat, rxLon float64) (lat, lon float64, ok bool) {
	fflag := mm.fflag
	cprlat := float64(mm.rawLatitude) / 131072.0
	cprlon := float64(mm.rawLongitude) / 131072.0
	dlat := 360.0 / 60.0
	if fflag != 0 {
		dlat = 360.0 / 59.0
	}
	j := math.Floor(rxLat/dlat) + float64(fflag)
	nlat := 60
	if fflag != 0 {
		nlat = 59
	}
	lat = dlat * (float64(cprModFunction(int(j), nlat)) + cprlat)
	if lat >= 270 {
		lat -= 360
	}
	nl := cprNLFunction(lat) - fflag
	if nl < 1 {
		return 0, 0, false
	}
	dlon := 360.0 / float64(nl)
	m := math.Floor(rxLon/dlon) + cprlon
	lon = dlon * (float64(cprModFunction(int(m), nl)) + (m - math.Floor(m)))
	if lon > 180 {
		lon -= 360
	}
	return lat, lon, true
}

func useModesMessage(d *Decoder, out *ADSBResult, gotResult *bool, mm *modesMessage) {
	if *gotResult || out == nil || !mm.crcok {
		return
	}
	*out = ADSBResult{}
	out.ICAO = uint32(mm.aa1<<16 | mm.aa2<<8 | mm.aa3)
	if mm.msgtype == 17 {
		if mm.metype >= 1 && mm.metype <= 4 {
			out.Callsign = string(mm.flight[:8])
			out.ValidCallsign = true
		}
		if mm.metype >= 9 && mm.metype <= 18 {
			out.Altitude = mm.altitude
			out.ValidAltitude = true
			if lat, lon, ok := decodeCPRRelative(mm, d.rxLat, d.rxLon); ok {
				out.Lat = lat
				out.Lon = lon
				out.ValidPosition = true
			}
		}
	}
	*gotResult = true
}

func modesInitConfig() {
	modes.fixErrors = true
	modes.checkCRC = true
	modes.aggressive = false
}

func modesInit() {
	modes.dataLen = modesDataLen + (modesFullLen-1)*4
	modes.data = make([]byte, modes.dataLen)
	modes.magnitude = make([]uint16, modes.dataLen)
	modes.maglut = make([]uint16, 129*129)
	for i := 0; i <= 128; i++ {
		for q := 0; q <= 128; q++ {
			modes.maglut[i*129+q] = uint16(math.Round(math.Sqrt(float64(i*i+q*q)) * 360))
		}
	}
	for i := range modes.data {
		modes.data[i] = 127
	}
}

// DecodeFrame decodes a single Mode S frame provided as raw bytes.
func (d *Decoder) DecodeFrame(frame []byte) (ADSBResult, error) {
	if len(frame) == 0 {
		return ADSBResult{}, errors.New("empty frame")
	}
	var mm modesMessage
	decodeModesMessage(&mm, frame)
	var out ADSBResult
	gotResult := false
	useModesMessage(d, &out, &gotResult, &mm)
	if !gotResult {
		return ADSBResult{}, errors.New("unable to decode frame")
	}
	return out, nil
}
