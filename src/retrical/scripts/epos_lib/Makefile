CC = g++
CFLAGS = -I.
DEPS = epos_lib.h
NAME = EPOS
OBJ = eposSin.o epos_lib.o
LIBS = -lEposCmd

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) ${LIBS} -c -o $@ $<

all: $(OBJ)
	${CC} $(CFLAGS) -o $@ $^
