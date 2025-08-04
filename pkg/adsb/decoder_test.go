package adsb

import (
	"encoding/hex"
	"testing"
)

func TestDecodeFrame(t *testing.T) {
	hexmsg := "8D451E8B99019699C00B0A81F36E"
	msg, err := hex.DecodeString(hexmsg)
	if err != nil {
		t.Fatalf("failed to decode hex: %v", err)
	}
	d := NewDecoder(0, 0)
	res, err := d.DecodeFrame(msg)
	if err != nil {
		t.Fatalf("DecodeFrame returned error: %v", err)
	}
	if res.ICAO != 0x451E8B {
		t.Fatalf("unexpected ICAO: got %X", res.ICAO)
	}
}

func TestDecodeFrameEmpty(t *testing.T) {
	d := NewDecoder(0, 0)
	_, err := d.DecodeFrame(nil)
	if err == nil {
		t.Fatalf("expected error for empty frame")
	}
}
