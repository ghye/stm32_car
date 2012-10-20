EN = ais.exe

# List C source files here. (C dependencies are automatically generated.)
SSRC	= \
main.c\
proto.c\
nmea0813.c

# Optional compiler flags.
#  -g:        generate debugging information (for GDB, or for COFF conversion)
#  -O*:       optimization level
#  -f...:     tuning, see gcc manual and avr-libc documentation
#  -Wall...:  warning level
#  -Wa,...:   tell GCC to pass this to the assembler.
#    -ahlms:  create assembler listing
CFLAGS = -Wall -O -g

CC = gcc

REMOVE = rm -f

# Define all object files.
SOBJ = $(SSRC:.c=.o)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = $(CFLAGS)

# Default target.
all: $(EN) 

$(EN): $(SOBJ)
	$(CC) $(ALL_CFLAGS) $(SOBJ) --output $@ 

# Compile: create object files from C source files.
%.o : %.c
	$(CC) -c $(ALL_CFLAGS) $< -o $@

# Target: clean project.
clean: clean_list

clean_list :
	$(REMOVE) $(EN)
	$(REMOVE) $(SOBJ) $(ROBJ)


.PHONY : all clean clean_list

