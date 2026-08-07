# Size-tuned: -Oz plus section GC. On MIPS32 the small I-caches make
# smaller code faster, not just smaller.
CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
# (raptor's -flto is deliberately absent: it exists to drop unused
# library code, and sinfo links nothing but prebuilt libc)
CFLAGS ?= -Oz -Wall -Wextra -std=gnu99 -ffunction-sections -fdata-sections \
	  -fno-asynchronous-unwind-tables -fmerge-all-constants -fno-ident
LDFLAGS ?= -static -Wl,--gc-sections -Wl,-z,max-page-size=0x1000

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
