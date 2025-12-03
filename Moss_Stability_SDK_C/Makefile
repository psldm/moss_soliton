CC = gcc
CFLAGS = -Wall -Wextra -O3 -I./include -std=c99
CFLAGS_DEBUG = -Wall -Wextra -O0 -g -DDEBUG -fsanitize=address,undefined
CFLAGS_RELEASE = -Wall -Wextra -O3 -DNDEBUG -flto -march=native -mtune=native
LDFLAGS = -lm

TARGET_LIB = libmoss_stability.a
TARGET_DEMO = moss_sim
TARGET_TEST_TERNARY = test_ternary_core
TARGET_TEST_REASON = test_reason_solver
TARGET_MEMORY_POOL = memory_pool_example

SRC = src/reason_solver.c src/ternary_core.c
OBJ = $(SRC:.c=.o)

all: $(TARGET_LIB) $(TARGET_DEMO) $(TARGET_TEST_TERNARY) $(TARGET_TEST_REASON) $(TARGET_MEMORY_POOL)

$(TARGET_LIB): $(OBJ)
	ar rcs $@ $^

$(TARGET_DEMO): examples/flight_loop_sim.c $(TARGET_LIB)
	$(CC) $(CFLAGS) $< -L. -lmoss_stability -o $@ $(LDFLAGS)

$(TARGET_TEST_TERNARY): tests/test_ternary_core.c $(TARGET_LIB)
	$(CC) $(CFLAGS) $< -L. -lmoss_stability -o $@ $(LDFLAGS)

$(TARGET_TEST_REASON): tests/test_reason_solver.c $(TARGET_LIB)
	$(CC) $(CFLAGS) $< -L. -lmoss_stability -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

test: $(TARGET_TEST_TERNARY) $(TARGET_TEST_REASON)
	./$(TARGET_TEST_TERNARY)
	./$(TARGET_TEST_REASON)

clean:
	rm -f src/*.o examples/*.o *.a $(TARGET_DEMO) $(TARGET_TEST_TERNARY) $(TARGET_TEST_REASON)

.PHONY: all test clean
