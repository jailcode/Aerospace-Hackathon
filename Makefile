CC = gcc
CFLAGS = -Wall -Wextra -Iincludes

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)

TARGET = app

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ)

fclean: clean
	rm -f $(TARGET)

re: fclean all

.PHONY: all clean fclean re
