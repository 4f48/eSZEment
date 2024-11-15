#!/bin/bash

sudo arduino-cli compile -b esp32:esp32:lilygo_t_display
sudo arduino-cli upload -b esp32:esp32:lilygo_t_display
