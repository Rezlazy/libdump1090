CC?=gcc
CFLAGS?=-O2 -g -Wall -W -fPIC $(shell pkg-config --cflags librtlsdr)
LDFLAGS?=$(shell pkg-config --libs librtlsdr) -lpthread -lm
OBJS=dump1090.o anet.o

all: libdump1090.a libdump1090.so dump1090

%.o: %.c
	$(CC) $(CFLAGS) -c $<

libdump1090.a: $(OBJS)
	ar rcs $@ $(OBJS)

libdump1090.so: $(OBJS)
	$(CC) -shared -o $@ $(OBJS) $(LDFLAGS)

dump1090: main.o libdump1090.a
	$(CC) $(CFLAGS) -o $@ main.o libdump1090.a $(LDFLAGS)

clean:
	rm -f $(OBJS) main.o libdump1090.a libdump1090.so dump1090
