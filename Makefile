CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -std=gnu99
LDFLAGS ?= -static

sinfo: sinfo.c sensors.h
	$(CC) $(CFLAGS) -o $@ sinfo.c $(LDFLAGS)

clean:
	rm -f sinfo

.PHONY: clean
