# Sessantaquattro LSL Interface

A Rust-based TCP server that connects to OT Bioelettronica Sessantaquattro bioelectrical devices and streams real-time data via Lab Streaming Layer (LSL) for research applications.

Link to device: [OT Bioelettronica Sessantaquattro](https://otbioelettronica.it/en/sessantaquattro/)

## Features

- **Real-time bioelectrical streaming**: Up to 68 channels (64 bio + 2 AUX + 2 accessory)
- **Multiple acquisition modes**: Monopolar, bipolar, differential, accelerometers, impedance check, test mode
- **Flexible sampling rates**: 500, 1000, 2000, or 4000 Hz
- **High resolution support**: 16-bit or 24-bit ADC
- **Configurable gain**: 2x, 4x, 6x, or 8x preamp gain
- **High-pass filter**: Optional DC offset removal
- **LSL integration**: Seamless streaming with proper channel metadata
- **Robust connectivity**: Configurable timeouts and error handling
- **Cross-platform**: Runs on Linux, macOS, and Windows

## Channel Configuration

The Sessantaquattro supports variable channel configurations:

- **8, 16, 32, or 64 bioelectrical channels**: High-quality signal acquisition
- **2 AUX channels**: Auxiliary signal inputs
- **2 Accessory channels**: Additional sensor inputs

## Installation

### Prerequisites

- Rust toolchain (install from [rustup.rs](https://rustup.rs/))
- Lab Streaming Layer (LSL) library

### Build from source

```bash
git clone <repository-url>
cd sessantaquattro-lsl-interface
cargo build --release
```

The binary will be available at `target/release/sessantaquattro-lsl-interface`.

## Usage

### Basic usage

```bash
./sessantaquattro-lsl-interface
```

This will start the server with default settings:

- 64 channels at 2000 Hz
- Monopolar mode
- 16-bit resolution
- Gain 8x
- No high-pass filter

### Command-line options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--host` | String | `0.0.0.0` | Host address to bind TCP server |
| `--port` | u16 | `54320` | Port number for TCP connection |
| `--connection-timeout` | u64 | `30` | Connection timeout in seconds |
| `--data-timeout` | u64 | `5` | Data reception timeout in seconds |
| `--sampling-frequency` | u16 | `2000` | Sampling rate in Hz (500, 1000, 2000, 4000) |
| `--channels` | u8 | `64` | Number of bioelectrical channels (8, 16, 32, 64) |
| `--gain` | u8 | `8` | Preamp gain (2, 4, 6, 8) |
| `--high-res` | Flag | `false` | Enable 24-bit resolution |
| `--hpf` | Flag | `false` | Enable high-pass filter |
| `--test-mode` | Flag | `false` | Enable test mode (generates ramp signals) |
| `--no-conversion` | Flag | `false` | Stream raw ADC values instead of microvolts |
| `--lsl-uuid` | String | `sessantaquattro-001` | LSL stream source ID |

### Example configurations

**Standard EMG recording (64 channels, 2 kHz, 16-bit):**

```bash
./sessantaquattro-lsl-interface
```

**High-resolution DC recording (32 channels, 1 kHz, 24-bit, no HPF):**

```bash
./sessantaquattro-lsl-interface --channels 32 --sampling-frequency 1000 --high-res
```

**Fast acquisition with HPF (16 channels, 4 kHz, 16-bit, HPF on):**

```bash
./sessantaquattro-lsl-interface --channels 16 --sampling-frequency 4000 --hpf
```

**Test mode for development:**

```bash
./sessantaquattro-lsl-interface --test-mode
```

**Low-noise recording with maximum gain:**

```bash
./sessantaquattro-lsl-interface --gain 8 --high-res --hpf
```

## Device Protocol

The Sessantaquattro uses the TCP Communication Protocol v1.8. The interface sends a 2-byte configuration command:

- **CONTROL BYTE 0**: `[GETSET][FSAMP1:0][NCH1:0][MODE2:0]`
- **CONTROL BYTE 1**: `[HRES][HPF][GAIN1:0][TRIG1:0][REC][GO/STOP]`

### Working Modes

- **Monopolar** (000): All channels referenced to common ground
- **Bipolar** (001): Differential pairs using AD8x1SE adapter
- **Differential** (010): Consecutive channel differences
- **Accelerometers** (011): 8 channels with higher sampling rates
- **Impedance Check** (110): Electrode impedance measurement
- **Test Mode** (111): Ramp signal generation for testing

### Gain and Resolution

The combination of gain and resolution determines signal range and precision:

| Gain | HRES=0 (16-bit) | HRES=1 (24-bit) |
|------|-----------------|-----------------|
| 8x   | 286.1 nV, ±9.4 mV | 71.5 nV, ±300 mV |
| 6x   | 381.5 nV, ±12.5 mV | 95.4 nV, ±400 mV |
| 4x   | 572.2 nV, ±18.8 mV | 143.0 nV, ±600 mV |
| 2x   | N/A | 286.1 nV, ±1200 mV |

## LSL Integration

The streamer creates an LSL outlet with the following properties:

- **Stream name**: `SESSANTAQUATTRO`
- **Stream type**: `SESSANTAQUATTRO_DATA`
- **Channel count**: 12, 20, 36, or 68 (depending on configuration)
- **Sample rate**: 500.0, 1000.0, 2000.0, or 4000.0 Hz
- **Channel format**: Float32 (converted) or Int32 (raw)
- **Source ID**: `sessantaquattro-001` (configurable with `--lsl-uuid`)

### Channel Metadata

Each channel includes proper metadata for analysis software:

- **Bioelectrical channels**: Type="BIO", Units="microvolts" (or "dimensionless")
- **AUX channels**: Type="AUX", Units="dimensionless"
- **Accessory channels**: Type="ACCESSORY", Units="dimensionless"

### Device Metadata

The LSL stream includes device information:

- Manufacturer: OTBioelettronica
- Device: Sessantaquattro
- Conversion factor (nV per bit)
- Resolution (16-bit or 24-bit)
- Gain setting
- HPF status
- Working mode

## Error Handling

The application handles various error conditions:

- **Connection timeout**: No device connection within specified time
- **Data timeout**: No data received from connected device
- **Configuration errors**: Invalid parameter combinations
- **IO errors**: Network or communication failures
- **LSL errors**: Streaming layer issues

## Network Configuration

The Sessantaquattro device needs to connect to the PC running this interface:

1. **Sessantaquattro as Access Point**: The device automatically gets the PC's IP address when connected
2. **External Access Point**: Configure the "Server IP address" in the device's web interface to point to the PC running this software

The default port is 54320 (different from Muovi's 54321).

## Contributing

This project is developed for research purposes. For issues or contributions, please contact the maintainer.

## License

This project is licensed under the GPL-3.0 License - see the [LICENSE](LICENSE.md) file for details.

## Acknowledgments

- OT Bioelettronica for the Sessantaquattro bioelectrical system
- Lab Streaming Layer (LSL) project
- Rust community for excellent tooling

## Technical Notes

### Data Format

- **16-bit mode**: 2 bytes per channel, little-endian signed integer
- **24-bit mode**: 3 bytes per channel, little-endian signed integer with sign extension

### Timestamp Precision

Each sample receives an LSL timestamp with sub-millisecond precision using `lsl::local_clock()`.

### Buffer Management

The interface uses exact-size read operations to ensure frame alignment and prevent data corruption.
