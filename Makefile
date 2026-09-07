CC      := cc
CSTD    := -std=c11
PKG     := sdl2
CFLAGS  := $(CSTD) -Wall -Wextra -Wpedantic -Werror -O2 $(shell pkg-config --cflags $(PKG))
LDFLAGS := $(shell pkg-config --libs $(PKG)) -lm

SRC     := src/main.c src/mechanism.c src/solver.c src/linalg.c src/render.c src/export.c src/ui.c src/cam.c src/synth.c src/joints.c src/templates.c src/status.c src/scene.c src/gearing.c src/mesh3d.c src/print3d.c src/xalloc.c
OBJ     := $(SRC:.c=.o)
HDRS    := $(wildcard src/*.h)
BIN     := linkage_design

TEST_SRC := tests/test_mechanism.c src/mechanism.c src/solver.c src/linalg.c src/export.c src/ui.c src/cam.c src/synth.c src/joints.c src/templates.c src/status.c src/scene.c src/gearing.c src/mesh3d.c src/print3d.c src/xalloc.c
TEST_BIN := tests/test_mechanism

.PHONY: all test clean docs-images

all: $(BIN)

$(BIN): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

$(OBJ): $(HDRS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(TEST_SRC)
	$(CC) $(CSTD) -Wall -Wextra -Wpedantic -Werror -O2 $(TEST_SRC) -lm -o $@

# Regenerates every picture in docs/images/ from the program itself. Needs
# ffmpeg; see tools/make-docs-images.sh.
docs-images: $(BIN)
	./tools/make-docs-images.sh

clean:
	rm -f $(OBJ) $(BIN) $(TEST_BIN)
