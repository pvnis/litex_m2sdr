# SPDX-License-Identifier: BSD-2-Clause
# M2SDR VFIO User-Mode Driver

CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Wextra -fPIC
CFLAGS  += -I.

LIB     = libm2sdr_vfio.a
SRCS    = vfio_setup.c vfio_dma.c vfio_poll.c vfio_csr.c
OBJS    = $(SRCS:.c=.o)

TEST    = m2sdr_vfio_test

all: $(LIB) $(TEST)

$(LIB): $(OBJS)
	$(AR) rcs $@ $^

$(TEST): m2sdr_vfio_test.c $(LIB)
	$(CC) $(CFLAGS) -o $@ $< -L. -lm2sdr_vfio

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(LIB) $(TEST)

.PHONY: all clean
