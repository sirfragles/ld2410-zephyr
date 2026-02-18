# Copyright (c) 2024 Matthew Macdonald-Wallace
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LD2410)
  add_subdirectory(drivers/sensor/ld2410)
endif()

zephyr_include_directories(include)
