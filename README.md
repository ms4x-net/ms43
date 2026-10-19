# Siemens MS43

Definition files, stock firmware and background documentation for the **Siemens MS43** engine control unit.

📖 Documentation: **[Siemens MS43](https://www.ms4x.net/index.php?title=Siemens_MS43)** · [Pinout](https://www.ms4x.net/index.php?title=Siemens_MS43_Pinout) · [CAN Bus](https://www.ms4x.net/index.php?title=Siemens_MS43_CAN_Bus) · [PCB Components](https://www.ms4x.net/index.php?title=Siemens_MS43_PCB_Components) · [Community Patchlist](https://www.ms4x.net/index.php?title=TunerPro_MS43_Community_Patchlist) · [Extended Load Limit](https://www.ms4x.net/index.php?title=Siemens_MS43_Extended_Load_Limit)

🚀 Looking for boost control, flex fuel or launch control? → **[MS43X Custom Firmware](https://github.com/ms4x-net/MS43X-Custom-Firmware)**

## What's in this repository

| Folder | Content |
|---|---|
| [`definitions/`](definitions/) | TunerPro XDFs for MS430037, 0055, 0056, 0064, 0066 and 430069 – 512 KB full read and 64 KB calibration |
| [`firmware/`](firmware/) | Stock **MS430069** binaries for E36/7 Z3, E39, E46 and E53 (M52TUB28, M54B22/B25/B30, EU2/EU4/US) |
| [`explainations/`](explainations/) | Technical write-ups, e.g. [SSG shift behavior](explainations/ssg-shift-behavior.md) |

## Which file do I need?

| You have… | Use |
|---|---|
| a full 512 KB read | `…_512K.xdf` |
| a 64 KB calibration read | `…_64K.xdf` |
| an older software (0037–0066) and want the latest features | update to 430069 with a matching file from [`firmware/`](firmware/) first |

## Usage

1. Read the DME with a [flashing tool](https://www.ms4x.net/index.php?title=Flashing_Tools) and note the software version.
2. Open the binary in [TunerPro RT](https://www.tunerpro.net) with the matching XDF.
3. Optional: apply patches from the [Community Patchlist](https://www.ms4x.net/index.php?title=TunerPro_MS43_Community_Patchlist).
4. Keep an untouched backup of the original read.

## Contributing

Bug in a definition or missing firmware variant? Please [open an issue](https://github.com/ms4x-net/ms43/issues).

## License & disclaimer

Definition files and documentation are released under [GPL-3.0](LICENSE). Firmware binaries are provided for preservation and repair purposes.

> [!WARNING]
> For off-road / closed-course use only. No warranty – you modify your ECU at your own risk.
