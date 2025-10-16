#!/bin/bash

BINARY=build/motor_controller.bin

if [ ! -f "$BINARY" ]; then
    echo "Error: $BINARY not found!"
    exit 1
fi

echo "=== Flashing with OpenOCD (ST-Link) ==="

# Retry logic for flash operation
MAX_RETRIES=2
RETRY_COUNT=0

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    openocd -f interface/stlink.cfg \
            -f target/stm32f4x.cfg \
            -c "init" \
            -c "reset halt" \
            -c "sleep 200" \
            -c "flash write_image erase $BINARY 0x08000000" \
            -c "verify_image $BINARY 0x08000000" \
            -c "reset halt" \
            -c "mww 0xE000EDF0 0xA05F0003" \
            -c "shutdown"

    if [ $? -eq 0 ]; then
        echo ""
        echo "=== Flash Complete! ==="
        echo ""
        echo "Now run: st-info --probe"
        echo "Then start Agent"
        exit 0
    else
        RETRY_COUNT=$((RETRY_COUNT + 1))
        if [ $RETRY_COUNT -lt $MAX_RETRIES ]; then
            echo ""
            echo "=== Flash failed, retrying ($RETRY_COUNT/$MAX_RETRIES)... ==="
            echo ""
            sleep 1
        fi
    fi
done

echo ""
echo "=== Flash Failed after $MAX_RETRIES attempts! ==="
exit 1
