# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(opt3001.c)
zephyr_library_sources_ifdef(CONFIG_OPT3001_TRIGGER opt3001_trigger.c)
