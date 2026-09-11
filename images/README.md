# Pinned B2xx images

These unmodified assets come from the [UHD 4.8.0.0 manifest](https://raw.githubusercontent.com/EttusResearch/uhd/v4.8.0.0/images/manifest.txt): FPGA revision `c37b318`, FX3 firmware/bootloader revision `7f7d016`. `checksums.json` records the archive URLs, manifest SHA256 values, and SHA256 values of all six extracted files. The FPGA synthesis reports are retained under `notices/`.

Copyright holders include Ettus Research LLC, National Instruments, and Cypress Semiconductor Corporation. Upstream license texts are retained under `notices/upstream/`. The FX3 application and bootloader identify GPL-3.0-or-later in their source headers; the USRP3 FPGA tree identifies LGPLv3 for Ettus code, with separately marked third-party components. Individual upstream notices remain authoritative.

Corresponding source and build instructions:

- [FPGA source at c37b318](https://github.com/EttusResearch/uhd/tree/c37b318/fpga/usrp3)
- [FX3 source at 7f7d016](https://github.com/EttusResearch/uhd/tree/7f7d016/firmware/fx3/b200)
- [FX3 application notice](https://github.com/EttusResearch/uhd/blob/7f7d016/firmware/fx3/b200/firmware/b200_main.c)
- [FX3 bootloader notice](https://github.com/EttusResearch/uhd/blob/7f7d016/firmware/fx3/b200/bootloader/main.c)

Maintainers can run `python3 images/refresh-images.py` to fetch the pinned archives with curl, verify their SHA256 values, and regenerate the files and checksums. Updating the pinned revisions also requires updating `Image::pinned_hash` and its verification test. No Cargo build downloads images. The bootloader is a catalog asset only; opening a device never installs it.
