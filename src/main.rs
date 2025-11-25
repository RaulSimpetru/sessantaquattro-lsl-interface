//! Sessantaquattro bioelectrical device TCP server and LSL streamer
//!
//! This module implements a TCP server that connects to OT Bioelettronica Sessantaquattro devices
//! and streams multi-channel bioelectrical data via Lab Streaming Layer (LSL). The Sessantaquattro
//! is a high-density wireless acquisition system supporting up to 64 bioelectrical channels plus
//! auxiliary and accessory channels.
//!
//! ## Features
//! - TCP server with configurable host/port and timeouts
//! - Support for multiple acquisition modes (monopolar, bipolar, differential, accelerometers, impedance check, test)
//! - Configurable sampling rates (500, 1000, 2000, 4000 Hz)
//! - Variable channel counts (8, 16, 32, 64 bioelectrical + 2 AUX + 2 accessory)
//! - 16-bit or 24-bit resolution
//! - Configurable gain settings
//! - High-pass filter option
//! - Real-time data conversion from raw ADC values to volts
//! - LSL streaming with proper channel metadata
//! - Builder pattern API for easy configuration
//!
//! ## Device Protocol
//! The Sessantaquattro uses a 13-byte configuration protocol with detailed control over
//! acquisition parameters. Data streaming occurs after sending configuration commands.
//!
//! For complete device documentation and specifications, see:
//! TCP Communication Protocol v1.8 (included with device)

use byteorder::{BigEndian, ReadBytesExt};
use chrono::Datelike;
use clap::Parser;
use lsl::Pushable;
use socket2::{Domain, Socket, Type};
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::{Duration, Instant};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum SessantaquattroError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("LSL error: {0}")]
    Lsl(#[from] lsl::Error),
    #[error("Connection timeout: No connection received within {0} seconds")]
    ConnectionTimeout(u64),
    #[error("Data timeout: No data received within {0} seconds")]
    DataTimeout(u64),
    #[error("Configuration error: {0}")]
    Configuration(String),
}

type Result<T> = std::result::Result<T, SessantaquattroError>;

/// Sampling frequency options
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SamplingFrequency {
    Hz4000 = 0b11,
    Hz2000 = 0b10,
    Hz1000 = 0b01,
    Hz500 = 0b00,
}

impl SamplingFrequency {
    pub fn as_f64(&self) -> f64 {
        match self {
            SamplingFrequency::Hz500 => 500.0,
            SamplingFrequency::Hz1000 => 1000.0,
            SamplingFrequency::Hz2000 => 2000.0,
            SamplingFrequency::Hz4000 => 4000.0,
        }
    }

    pub fn from_u16(value: u16) -> Result<Self> {
        match value {
            500 => Ok(SamplingFrequency::Hz500),
            1000 => Ok(SamplingFrequency::Hz1000),
            2000 => Ok(SamplingFrequency::Hz2000),
            4000 => Ok(SamplingFrequency::Hz4000),
            _ => Err(SessantaquattroError::Configuration(format!(
                "Invalid sampling frequency: {}. Must be 500, 1000, 2000, or 4000",
                value
            ))),
        }
    }
}

/// Number of channels configuration
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelCount {
    Ch64 = 0b11,
    Ch32 = 0b10,
    Ch16 = 0b01,
    Ch8 = 0b00,
}

impl ChannelCount {
    /// Returns the NCH setting value (not the actual channel count)
    pub fn nch_setting(&self) -> usize {
        match self {
            ChannelCount::Ch8 => 8,
            ChannelCount::Ch16 => 16,
            ChannelCount::Ch32 => 32,
            ChannelCount::Ch64 => 64,
        }
    }

    /// Returns actual bioelectrical channels for a given mode
    /// In Bipolar mode (001), channel count is halved
    pub fn bio_channels(&self, mode: WorkingMode) -> usize {
        let base = self.nch_setting();
        if mode == WorkingMode::Bipolar {
            base / 2 // Bipolar mode halves the channel count
        } else {
            base
        }
    }

    /// Returns total channels (bio + 2 AUX + 2 accessory) for a given mode
    pub fn total_channels(&self, mode: WorkingMode) -> usize {
        self.bio_channels(mode) + 4 // bio + 2 AUX + 2 accessory
    }

    pub fn from_u8(value: u8) -> Result<Self> {
        match value {
            8 => Ok(ChannelCount::Ch8),
            16 => Ok(ChannelCount::Ch16),
            32 => Ok(ChannelCount::Ch32),
            64 => Ok(ChannelCount::Ch64),
            _ => Err(SessantaquattroError::Configuration(format!(
                "Invalid channel count: {}. Must be 8, 16, 32, or 64",
                value
            ))),
        }
    }
}

/// Working mode options
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WorkingMode {
    Monopolar = 0b000,
    Bipolar = 0b001,
    Differential = 0b010,
    Accelerometers = 0b011,
    BipolarAD4x8SP = 0b100, // Sessantaquattro+ only: External preamps with gain of 5
    ImpedanceCheckAdvanced = 0b101,
    ImpedanceCheck = 0b110,
    TestMode = 0b111,
}

impl WorkingMode {
    pub fn from_str(s: &str) -> Result<Self> {
        match s.to_lowercase().as_str() {
            "monopolar" => Ok(WorkingMode::Monopolar),
            "bipolar" => Ok(WorkingMode::Bipolar),
            "differential" => Ok(WorkingMode::Differential),
            "accelerometers" | "accel" => Ok(WorkingMode::Accelerometers),
            "bipolar-ad4x8sp" | "ad4x8sp" => Ok(WorkingMode::BipolarAD4x8SP),
            "impedance-advanced" => Ok(WorkingMode::ImpedanceCheckAdvanced),
            "impedance" => Ok(WorkingMode::ImpedanceCheck),
            "test" => Ok(WorkingMode::TestMode),
            _ => Err(SessantaquattroError::Configuration(format!(
                "Invalid mode: {}. Valid modes: monopolar, bipolar, differential, accelerometers, bipolar-ad4x8sp, impedance, impedance-advanced, test",
                s
            ))),
        }
    }

    /// Returns the external gain factor for modes with external preamps
    pub fn external_gain(&self) -> f32 {
        match self {
            WorkingMode::BipolarAD4x8SP => 5.0, // AD4x8SP has external preamps with gain of 5
            _ => 1.0,
        }
    }
}

/// Preamp gain options
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gain {
    Gain2 = 0b00, // Only when HRES=1
    Gain4 = 0b01,
    Gain6 = 0b10,
    Gain8 = 0b11,
}

impl Gain {
    pub fn from_u8(value: u8) -> Result<Self> {
        match value {
            2 => Ok(Gain::Gain2),
            4 => Ok(Gain::Gain4),
            6 => Ok(Gain::Gain6),
            8 => Ok(Gain::Gain8),
            _ => Err(SessantaquattroError::Configuration(format!(
                "Invalid gain: {}. Must be 2, 4, 6, or 8",
                value
            ))),
        }
    }

    #[allow(non_snake_case)]
    pub fn get_resolution__nV(&self, high_res: bool) -> f32 {
        match (self, high_res) {
            (Gain::Gain8, true) => 71.5,
            (Gain::Gain8, false) => 286.1,
            (Gain::Gain6, true) => 95.4,
            (Gain::Gain6, false) => 381.5,
            (Gain::Gain4, true) => 143.0,
            (Gain::Gain4, false) => 572.2,
            (Gain::Gain2, true) => 286.1,
            (Gain::Gain2, false) => 286.1, // Same as gain 8 when HRES=0
        }
    }
}

#[derive(Parser)]
#[command(name = "sessantaquattro-lsl-interface")]
#[command(
    about = "Sessantaquattro LSL Interface - streams bioelectrical data via Lab Streaming Layer"
)]
struct Args {
    #[arg(long, default_value = "0.0.0.0", help = "Host to bind to")]
    host: String,

    #[arg(long, default_value = "45454", help = "Port to listen on")]
    port: u16,

    #[arg(long, default_value = "30", help = "Connection timeout in seconds")]
    connection_timeout: u64,

    #[arg(long, default_value = "5", help = "Data timeout in seconds")]
    data_timeout: u64,

    #[arg(
        long,
        default_value = "2000",
        help = "Sampling frequency in Hz (500, 1000, 2000, 4000)"
    )]
    sampling_frequency: u16,

    #[arg(
        long,
        default_value = "64",
        help = "Number of bioelectrical channels (8, 16, 32, 64)"
    )]
    channels: u8,

    #[arg(
        long,
        default_value = "8",
        help = "Preamp gain (2, 4, 6, 8). Note: gain 2 only available with --high-res"
    )]
    gain: u8,

    #[arg(long, help = "Enable high resolution mode (24-bit)")]
    high_res: bool,

    #[arg(long, help = "Enable high pass filter")]
    hpf: bool,

    #[arg(
        long,
        default_value = "monopolar",
        help = "Working mode: monopolar, bipolar, differential, accelerometers, bipolar-ad4x8sp, impedance, impedance-advanced, test"
    )]
    mode: String,

    #[arg(long, help = "Stream raw ADC values instead of volts")]
    no_conversion: bool,

    #[arg(
        long,
        default_value = "sessantaquattro-001",
        help = "LSL stream source ID"
    )]
    lsl_uuid: String,
}

/// A TCP server that connects to Sessantaquattro devices and streams data via LSL.
#[allow(non_snake_case)]
pub struct SessantaquattroReader {
    host: String,
    port: u16,
    connection_timeout__s: u64,
    data_timeout__s: u64,
    sampling_frequency: SamplingFrequency,
    channel_count: ChannelCount,
    working_mode: WorkingMode,
    high_res: bool,
    hpf: bool,
    gain: Gain,
    apply_conversion: bool,
    lsl_uuid: String,
}

impl Default for SessantaquattroReader {
    fn default() -> Self {
        Self {
            host: "0.0.0.0".to_string(),
            port: 54320,
            connection_timeout__s: 30,
            data_timeout__s: 5,
            sampling_frequency: SamplingFrequency::Hz2000,
            channel_count: ChannelCount::Ch64,
            working_mode: WorkingMode::Monopolar,
            high_res: false,
            hpf: true,
            gain: Gain::Gain4,
            apply_conversion: true,
            lsl_uuid: "sessantaquattro-001".to_string(),
        }
    }
}

#[allow(non_snake_case)]
impl SessantaquattroReader {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn host(mut self, host: &str) -> Self {
        self.host = host.to_string();
        self
    }

    pub fn port(mut self, port: u16) -> Self {
        self.port = port;
        self
    }

    pub fn connection_timeout(mut self, timeout_s: u64) -> Self {
        self.connection_timeout__s = timeout_s;
        self
    }

    pub fn data_timeout(mut self, timeout_s: u64) -> Self {
        self.data_timeout__s = timeout_s;
        self
    }

    pub fn sampling_frequency(mut self, freq: SamplingFrequency) -> Self {
        self.sampling_frequency = freq;
        self
    }

    pub fn channel_count(mut self, count: ChannelCount) -> Self {
        self.channel_count = count;
        self
    }

    pub fn working_mode(mut self, mode: WorkingMode) -> Self {
        self.working_mode = mode;
        self
    }

    pub fn high_res(mut self, high_res: bool) -> Self {
        self.high_res = high_res;
        self
    }

    pub fn hpf(mut self, hpf: bool) -> Self {
        self.hpf = hpf;
        self
    }

    pub fn gain(mut self, gain: Gain) -> Self {
        self.gain = gain;
        self
    }

    pub fn apply_conversion(mut self, apply_conversion: bool) -> Self {
        self.apply_conversion = apply_conversion;
        self
    }

    pub fn lsl_uuid(mut self, lsl_uuid: &str) -> Self {
        self.lsl_uuid = lsl_uuid.to_string();
        self
    }

    /// Returns the actual sampling frequency, accounting for mode-specific multipliers
    /// In accelerometers mode, FSAMP encoding represents 4x the normal frequency
    fn get_actual_sampling_frequency(&self) -> f64 {
        let base_freq = self.sampling_frequency.as_f64();
        if self.working_mode == WorkingMode::Accelerometers {
            base_freq * 4.0
        } else {
            base_freq
        }
    }

    /// Returns the conversion factor from ADC values to microvolts
    /// Accounts for both internal preamp gain and external gain (e.g., AD4x8SP)
    fn get_conversion_factor(&self) -> f32 {
        let base_resolution = self.gain.get_resolution__nV(self.high_res);
        let external_gain = self.working_mode.external_gain();
        base_resolution * external_gain
    }

    /// Converts raw ADC values to microvolts
    fn convert_data__f32(&self, raw_data: &[i32]) -> Vec<f32> {
        let factor = self.get_conversion_factor() / 1000.0; // Convert nV to microvolts
        raw_data.iter().map(|&x| x as f32 * factor).collect()
    }

    /// Returns raw ADC values without conversion
    fn convert_data__i32(&self, raw_data: &[i32]) -> Vec<i32> {
        raw_data.to_vec()
    }

    /// Creates LSL stream info with proper channel metadata
    fn create_stream_info(&self) -> Result<lsl::StreamInfo> {
        let total_channels = self.channel_count.total_channels(self.working_mode);
        let bio_channels = self.channel_count.bio_channels(self.working_mode);

        let mut info = lsl::StreamInfo::new(
            "SESSANTAQUATTRO",
            "SESSANTAQUATTRO_DATA",
            total_channels as u32,
            self.get_actual_sampling_frequency(),
            if self.apply_conversion {
                lsl::ChannelFormat::Float32
            } else {
                lsl::ChannelFormat::Int32
            },
            &self.lsl_uuid,
        )?;

        // Add device metadata
        let mut desc = info.desc();
        desc.append_child_value("manufacturer", "OTBioelettronica");
        desc.append_child_value("device", "Sessantaquattro");
        desc.append_child_value(
            "conversion_factor_nv",
            &self.get_conversion_factor().to_string(),
        );
        desc.append_child_value(
            "resolution",
            if self.high_res { "24-bit" } else { "16-bit" },
        );
        desc.append_child_value("gain", &format!("{:?}", self.gain));
        desc.append_child_value("hpf", &self.hpf.to_string());
        desc.append_child_value("mode", &format!("{:?}", self.working_mode));

        // Add channel information
        let mut chns = desc.append_child("channels");

        // Bioelectrical channels
        for i in 0..bio_channels {
            let mut ch = chns.append_child("channel");
            ch.append_child_value("label", &format!("CH {}", i + 1));
            ch.append_child_value(
                "unit",
                if self.apply_conversion {
                    "microvolts"
                } else {
                    "dimensionless"
                },
            );
            ch.append_child_value("type", "BIO");
        }

        // AUX channels
        for i in 0..2 {
            let mut ch = chns.append_child("channel");
            ch.append_child_value("label", &format!("AUX {}", i + 1));
            ch.append_child_value("unit", "dimensionless");
            ch.append_child_value("type", "AUX");
        }

        // Accessory channels
        for i in 0..2 {
            let mut ch = chns.append_child("channel");
            ch.append_child_value("label", &format!("ACC {}", i + 1));
            ch.append_child_value("unit", "dimensionless");
            ch.append_child_value("type", "ACCESSORY");
        }

        Ok(info)
    }

    /// Waits for a TCP connection from the Sessantaquattro device
    fn wait_for_connection(&self) -> Result<TcpStream> {
        let address: SocketAddr = format!("{}:{}", self.host, self.port)
            .parse::<SocketAddr>()
            .map_err(|_| {
                std::io::Error::new(std::io::ErrorKind::InvalidInput, "Invalid address")
            })?;

        let socket = Socket::new(Domain::IPV4, Type::STREAM, None)?;
        socket.set_reuse_address(true)?;
        socket.bind(&address.into())?;
        socket.listen(1)?;

        let listener: TcpListener = socket.into();
        listener.set_nonblocking(true)?;

        println!(
            "Waiting for Sessantaquattro connection on {}:{} (timeout: {}s)",
            self.host, self.port, self.connection_timeout__s
        );

        let start_time = Instant::now();

        loop {
            if start_time.elapsed().as_secs() > self.connection_timeout__s {
                return Err(SessantaquattroError::ConnectionTimeout(
                    self.connection_timeout__s,
                ));
            }

            match listener.accept() {
                Ok((stream, addr)) => {
                    println!("Sessantaquattro connected from {}", addr);
                    stream.set_nonblocking(false)?;
                    stream.set_nodelay(true)?;
                    stream.set_read_timeout(Some(Duration::from_secs(self.data_timeout__s)))?;
                    stream.set_write_timeout(Some(Duration::from_secs(self.data_timeout__s)))?;
                    return Ok(stream);
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }
                #[cfg(windows)]
                Err(ref e) if e.raw_os_error() == Some(10035) => {
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }
                Err(e) => return Err(SessantaquattroError::Io(e)),
            }
        }
    }

    /// Sends device configuration command
    fn configure_device(&self, stream: &mut TcpStream) -> Result<()> {
        // CONTROL BYTE 0: [GETSET][FSAMP1][FSAMP0][NCH1][NCH0][MODE2][MODE1][MODE0]
        let control_byte_0: u8 = 0u8 << 7 // GETSET = 0 (SET command)
            | (self.sampling_frequency as u8) << 5
            | (self.channel_count as u8) << 3
            | (self.working_mode as u8);

        // CONTROL BYTE 1: [HRES][HPF][GAIN1][GAIN0][TRIG1][TRIG0][REC][GO/STOP]
        let control_byte_1: u8 = (self.high_res as u8) << 7
            | (self.hpf as u8) << 6
            | (self.gain as u8) << 4
            | 0b00 << 2 // TRIG = 00 (controlled by GO/STOP bit)
            | 0 << 1 // REC = 0 (no SD card recording)
            | 1; // GO/STOP = 1 (start streaming)

        println!("Configuring device:");
        println!(
            "\tSampling frequency: {} Hz",
            self.get_actual_sampling_frequency()
        );
        println!(
            "\tChannels: {} bio + 2 AUX + 2 accessory",
            self.channel_count.bio_channels(self.working_mode)
        );
        println!("\tMode: {:?}", self.working_mode);
        println!(
            "\tResolution: {}",
            if self.high_res { "24-bit" } else { "16-bit" }
        );
        println!("\tHPF: {}", if self.hpf { "enabled" } else { "disabled" });
        println!("\tGain: {:?}", self.gain);
        println!(
            "\tControl bytes: 0x{:02X} 0x{:02X}",
            control_byte_0, control_byte_1
        );

        stream.write_all(&[control_byte_0, control_byte_1])?;
        stream.flush()?;

        Ok(())
    }

    /// Runs the main acquisition loop
    pub fn run(&self) -> Result<()> {
        let mut stream = self.wait_for_connection()?;

        let outlet = lsl::StreamOutlet::new(&self.create_stream_info()?, 0, 360)?;

        self.configure_device(&mut stream)?;

        let total_channels = self.channel_count.total_channels(self.working_mode);
        let bytes_per_sample = if self.high_res { 3 } else { 2 };
        let buffer_size = total_channels * bytes_per_sample;
        let mut buffer: Vec<u8> = vec![0u8; buffer_size];

        println!("Reading data... (Ctrl+C to stop)");

        loop {
            match stream.read_exact(&mut buffer) {
                Ok(()) => {
                    // Parse data based on resolution
                    let mut raw_sample: Vec<i32> = Vec::with_capacity(total_channels);
                    let mut cursor = std::io::Cursor::new(&buffer);

                    for _ in 0..total_channels {
                        let value = if self.high_res {
                            // 24-bit signed integer (big-endian: MSB first)
                            let byte1 = cursor.read_u8()?;
                            let byte2 = cursor.read_u8()?;
                            let byte3 = cursor.read_u8()?;
                            let raw = (byte1 as i32) << 16 | (byte2 as i32) << 8 | (byte3 as i32);
                            // Sign extend from 24-bit to 32-bit
                            if raw & 0x00800000 != 0 {
                                raw | 0xFF000000u32 as i32
                            } else {
                                raw
                            }
                        } else {
                            // 16-bit signed integer (big-endian: MSB first)
                            cursor.read_i16::<BigEndian>()? as i32
                        };
                        raw_sample.push(value);
                    }

                    // Convert and push to LSL (let LSL assign timestamp automatically)
                    if self.apply_conversion {
                        let converted = self.convert_data__f32(&raw_sample);
                        outlet.push_sample(&converted)?;
                    } else {
                        let converted = self.convert_data__i32(&raw_sample);
                        outlet.push_sample(&converted)?;
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    return Err(SessantaquattroError::DataTimeout(self.data_timeout__s));
                }
                Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                    println!("Connection closed by Sessantaquattro");
                    break;
                }
                Err(e) => return Err(SessantaquattroError::Io(e)),
            }
        }

        // Stop streaming
        let control_byte_0 = 0u8; // GETSET = 0
        let control_byte_1 = 0u8; // GO/STOP = 0
        let _ = stream.write_all(&[control_byte_0, control_byte_1]);

        Ok(())
    }
}

fn main() -> Result<()> {
    // Display GPL license notice
    let current_year = chrono::Utc::now().year();
    let copyright_year = if current_year == 2025 {
        "2025".to_string()
    } else {
        format!("2025-{}", current_year)
    };

    println!(
        "sessantaquattro-lsl-interface Copyright (C) {} Raul C. Sîmpetru",
        copyright_year
    );
    println!("This program comes with ABSOLUTELY NO WARRANTY; for details see");
    println!("https://www.gnu.org/licenses/gpl-3.0.html#license-text.");
    println!("This is free software, and you are welcome to redistribute it");
    println!(
        "under certain conditions; for details see https://www.gnu.org/licenses/gpl-3.0.html#license-text."
    );
    println!();

    let args = Args::parse();

    // Parse and validate arguments
    let sampling_frequency = SamplingFrequency::from_u16(args.sampling_frequency)?;
    let mut channel_count = ChannelCount::from_u8(args.channels)?;
    let gain = Gain::from_u8(args.gain)?;
    let working_mode = WorkingMode::from_str(&args.mode)?;

    // Validate gain 2 only with high resolution
    if gain == Gain::Gain2 && !args.high_res {
        eprintln!("Error: Gain 2 is only available with --high-res");
        std::process::exit(1);
    }

    // Bipolar mode halves the channel count
    if working_mode == WorkingMode::Bipolar {
        let actual_channels = channel_count.bio_channels(working_mode);
        println!(
            "Note: Bipolar mode provides {} bio channels (pairs from {} inputs: Ch3-Ch1, Ch4-Ch2, etc.)",
            actual_channels, channel_count.nch_setting()
        );
    }

    // Accelerometers mode has special behavior
    if working_mode == WorkingMode::Accelerometers {
        // Forces 8 channels regardless of NCH setting
        if channel_count != ChannelCount::Ch8 {
            println!(
                "Warning: Accelerometers mode forces 8 channels (you specified {}). Using 8 channels.",
                channel_count.nch_setting()
            );
            channel_count = ChannelCount::Ch8;
        }
        // FSAMP has different meaning: 500→2000, 1000→4000, 2000→8000, 4000→16000 Hz
        let actual_freq = sampling_frequency.as_f64() * 4.0;
        println!(
            "Note: In accelerometers mode, sampling frequency is {}Hz (4x the normal rate)",
            actual_freq
        );
    }

    let reader = SessantaquattroReader::new()
        .host(&args.host)
        .port(args.port)
        .connection_timeout(args.connection_timeout)
        .data_timeout(args.data_timeout)
        .sampling_frequency(sampling_frequency)
        .channel_count(channel_count)
        .working_mode(working_mode)
        .high_res(args.high_res)
        .hpf(args.hpf)
        .gain(gain)
        .apply_conversion(!args.no_conversion)
        .lsl_uuid(&args.lsl_uuid);

    match reader.run() {
        Ok(()) => println!("Connection closed"),
        Err(e) => {
            eprintln!("Error: {}", e);
            std::process::exit(1);
        }
    }

    Ok(())
}
