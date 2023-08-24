#!/bin/bash
esptool.py --chip esp32s3 \
    -p /dev/cu.usbmodem2101 \
    -b 460800 \
    --before=default_reset \
    --after=hard_reset \
    write_flash 0x90000 ./prboom-go/build/prboom-go.bin


# esptool.py --chip esp32s3 \
#     -p /dev/cu.usbmodem2101 \
#     erase_flash


# esptool.py --chip esp32s3 \
#     -p /dev/cu.usbmodem101 \
#     -b 460800 \
#     --before=default_reset \
#     write_flash 0x8000 ./launcher/build/partition_table/partition-table.bin




#                  0x1000 bootloader/bootloader.bin \
#                  0x10000 hello_world.bin

# #   --flash_mode dio \
# esptool.py -p /dev/cu.usbmodem1101 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash 0x90000 retro-go_1.39-pre-4-gb7cb0_flow3r.fw