# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_BATTERY_SAVER) += battery_saver.o

BATTSAVE_PREFIX ?= /usr/local
BATTSAVE_BINDIR ?= $(BATTSAVE_PREFIX)/bin

.PHONY: install install-batsave uninstall-batsave

install: install-batsave

install-batsave:
	install -Dm755 tools/batsave $(DESTDIR)$(BATTSAVE_BINDIR)/batsave

uninstall-batsave:
	rm -f $(DESTDIR)$(BATTSAVE_BINDIR)/batsave
