.PHONY: clean

CC = mgcc
CFLAGS = -g -Wall -D__marte__
	
SRCS := $(wildcard *.c)
OBJS := $(patsubst %.c,%.o,$(wildcard *.c))
	
default: $(OBJS)

%.o: %.c $(SRCS)
	$(CC) -c $(CFLAGS) $<

clean:
	@find \( -name '*.[oa]' -or -name '*.ali' -or -name '*~*' -or -name 'mprogram*' \
		-or -name '*.exe' \) -print -delete
