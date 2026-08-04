# -Os: on MIPS32 the small I-caches make larger code slower, not faster.
CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -Os -Wall -Wextra -std=gnu99
LDFLAGS ?= -static

OBJS := sinfo.o sinfo_hw.o

sinfo: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

sinfo.o: sinfo.c sinfo_hw.h sensors.h
sinfo_hw.o: sinfo_hw.c sinfo_hw.h

# Regenerate the sensor table from sensors.csv (the source of truth)
sensors:
	python3 tools/gen-sensors.py

# Fail if sensors.h was not regenerated after a sensors.csv edit
check-table:
	python3 tools/gen-sensors.py --check

test: check-table
	$(MAKE) -C tests

clean:
	rm -f sinfo $(OBJS)
	$(MAKE) -C tests clean 2>/dev/null || true

.PHONY: clean test sensors check-table
