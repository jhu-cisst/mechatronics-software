CC = gcc
CFLAGS = -Wall
LDFLAGS = -lraw1394
EXEC = info1394 quad1394 block1394 time1394

all: $(EXEC)

%: %.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

quad1394: block1394
	cp block1394 quad1394

clean:
	rm -rf $(EXEC)
