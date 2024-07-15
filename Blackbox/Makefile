TARGET = txt2bbl
LIBS = -lm
CC = gcc
CFLAGS = -g -Wall

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard *.c))
HEADERS = $(wildcard *.h)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall $(LIBS) -o $@

# Run the program
run: $(TARGET)
	./$(TARGET)

clean:
ifeq ($(OS),Windows_NT)
	-del /f *.o
	-del /f *.bbl
	-del /f $(TARGET).exe
else
	-rm -f *.o
	-rm -f *.bbl
	-rm -f $(TARGET)
endif
