CC = gcc
CFLAGS = -Wall -Wextra -Iincludes

SRC = $(wildcard src/*.c)
INC = $(wildcard includes/*.c)
OBJ = $(SRC:.c=.o) $(INC:.c=.o)

TARGET = app

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

re: clean all

.PHONY: all clean re
