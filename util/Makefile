CC = gcc
CFLAGS = -Wall
LDFLAGS = -lraw1394
EXEC = info1394 quad1394 block1394 time1394

all: $(EXEC)

%: %.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

quad1394: block1394.c
	$(CC) -DQUAD1394 $(CFLAGS) block1394.c -o quad1394 $(LDFLAGS)

clean:
	rm -rf $(EXEC)
