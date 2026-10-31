# cUR as a mainline-MicroPython user C module for the Nexus port.
#
# This file lives in the MicroPython port, not in the Krux source tree.
# The cUR sources themselves remain the single submodule at
# krux/vendor/cUR.  scripts/build/build-micropython.sh stages a copy of
# this directory in the disposable build tree, so no symlinks are needed
# and the compiler's object paths stay under build-standard.

CUR_MOD_DIR := $(USERMOD_DIR)

SRC_USERMOD += $(CUR_MOD_DIR)/uUR.c
SRC_USERMOD += $(wildcard $(CUR_MOD_DIR)/src/*.c)
SRC_USERMOD += $(wildcard $(CUR_MOD_DIR)/src/types/*.c)
# The Nexus target has no K210 SHA peripheral, so use cUR's portable
# bundled SHA-256 rather than the MaixPy/K210 backend.
SRC_USERMOD += $(CUR_MOD_DIR)/src/sha256/sha256.c

CFLAGS_USERMOD += -I$(CUR_MOD_DIR) -I$(CUR_MOD_DIR)/src

# cUR deliberately keeps some fountain-coordinate arithmetic in double for
# interoperability, and its MicroPython print path promotes a float literal.
# The mainline unix port builds with -Werror; keep these as warnings rather
# than treating platform/compiler diagnostics as build failures.
CFLAGS_USERMOD += -Wno-error=double-promotion -Wno-error=float-conversion
