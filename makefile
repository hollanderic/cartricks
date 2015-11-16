
OUTPUT_NAME=cartricks

TOOLCHAIN_PREFIX :=


CC  :=$(TOOLCHAIN_PREFIX)gcc
RM	:=rm

C_SOURCE_FILES = \
./main.c \
./lpms.c \
./gps.c

CFLAGS += 

LDFLAGS +=

BUILD_DIRECTORY = _build

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_OBJECTS = $(addprefix $(BUILD_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o))



default: cartricks




cartricks: clean $(BUILD_DIRECTORY) $(C_OBJECTS)
	$(CC) -pthread $(LDFLAGS) $(C_OBJECTS) -o $(BUILD_DIRECTORY)/$(OUTPUT_NAME)



$(BUILD_DIRECTORY):
	@mkdir $(BUILD_DIRECTORY)

$(BUILD_DIRECTORY)/%.o: %.c
	$(CC) $(CFLAGS) -lpthread $(INC_PATHS) -O3 -c -o $@ $<

clean:
	@$(RM) -rf $(BUILD_DIRECTORY)



