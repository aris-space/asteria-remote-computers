#!/bin/bash

GPIO=19 # blue GPIO19

blink() {
    local state=1
    for ms in "$@"; do
        gpioset gpiochip0 $GPIO=$state
        sleep "$(echo "scale=3; $ms/1000" | bc)"
        state=$((1 - state))
    done
}

get_ssid() {
    wpa_cli -i wlan0 status 2>/dev/null | awk -F= '/^ssid/{print $2}'
}

get_wpa_state() {
    wpa_cli -i wlan0 status 2>/dev/null | awk -F= '/^wpa_state/{print $2}'
}

while true; do
    if [ "$(get_wpa_state)" = "COMPLETED" ]; then
        case "$(get_ssid)" in
            "asd")  blink 900 100 ;;
            "asd1") blink 700 100 100 100 ;;
            *)      blink 500 500 ;;
        esac
    else
        blink 250 250
    fi
done