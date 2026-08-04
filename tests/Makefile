# Host-side tests under AddressSanitizer + UBSan.
CC ?= gcc
CFLAGS ?= -g -O1 -Wall -Wextra -std=gnu99 -fsanitize=address,undefined \
	  -fno-sanitize-recover=all

test: test_sinfo
	./test_sinfo

test_sinfo: test_sinfo.c mock_hw.c mock_hw.h ../sinfo.c ../sinfo_hw.h ../sensors.h
	$(CC) $(CFLAGS) -I.. -o $@ test_sinfo.c mock_hw.c

clean:
	rm -f test_sinfo

.PHONY: test clean
