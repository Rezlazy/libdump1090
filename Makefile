CFLAGS?=-O2 -g -Wall -W $(shell pkg-config --cflags librtlsdr)
LDLIBS+=$(shell pkg-config --libs librtlsdr) -lpthread -lm
CC?=gcc
PROGNAME=dump1090

all: dump1090 libadsb_decoder.a

%.o: %.c
	$(CC) $(CFLAGS) -c $<

dump1090: dump1090.o anet.o
	$(CC) -g -o dump1090 dump1090.o anet.o $(LDFLAGS) $(LDLIBS)

libadsb_decoder.a: adsb_decoder.o
	ar rcs libadsb_decoder.a adsb_decoder.o

clean:
	rm -f *.o dump1090 libadsb_decoder.a
