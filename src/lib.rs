//! A simple GPIO based DCF77 decoder
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2

#![deny(warnings)]
#![no_std]

/// A structure to facilitate the decoding of a DCF77 signal which consists of 59 consecutive bits
/// of data
pub struct DCF77Time(pub u64);

impl DCF77Time {
    /// Generate an empty value for the storage of the DCF77 data
    pub fn new(dcf77bits: u64) -> Self {
        DCF77Time { 0: dcf77bits }
    }

    /// Validate the correct value of the start bit
    pub fn validate_start(&self) -> Result<(), ()> {
        if (self.0 & (1 << 0)) != 0 {
            Err(())
        } else {
            Ok(())
        }
    }

    /// Return whether summer time is signalled (without verifying the information)
    pub fn cest_unchecked(&self) -> bool {
        if (self.0 & (1 << 17)) != 0 {
            return true;
        }

        false
    }

    /// Return whether summer time is signalled with verification of the counter bit
    pub fn cest(&self) -> Result<bool, ()> {
        let cest = self.cest_unchecked();

        if ((self.0 & (1 << 18)) != 0) == cest {
            Err(())
        } else {
            Ok(cest)
        }
    }

    /// Return the current minutes of the hour (without verifying the information)
    pub fn minutes_unchecked(&self) -> u8 {
        self.calculate_2digit_bcd(21, 27)
    }

    /// Return the current minutes of the hour and verify parity and value < 60
    pub fn minutes(&self) -> Result<u8, ()> {
        let parity = self.calculate_parity(21, 27);
        let minutes = self.minutes_unchecked();

        if ((self.0 & (1 << 28)) != 0) != parity || minutes > 59 {
            Err(())
        } else {
            Ok(minutes)
        }
    }

    /// Return the current hours of the day (without verifying the information)
    pub fn hours_unchecked(&self) -> u8 {
        self.calculate_2digit_bcd(29, 34)
    }

    /// Return the current hours of the day and verify parity and value < 23
    pub fn hours(&self) -> Result<u8, ()> {
        let parity = self.calculate_parity(29, 34);
        let hours = self.hours_unchecked();

        if ((self.0 & (1 << 35)) != 0) != parity || hours > 23 {
            Err(())
        } else {
            Ok(hours)
        }
    }

    /// Return the current day of month (without verifying the information)
    pub fn day_unchecked(&self) -> u8 {
        self.calculate_2digit_bcd(36, 41)
    }

    /// Return the current day of month and do a basic value check
    pub fn day(&self) -> Result<u8, ()> {
        let day = self.day_unchecked();
        if day > 31 {
            Err(())
        } else {
            Ok(day)
        }
    }

    /// Return the current day of the week (without verifying the information)
    /// 1 meaning Monday ... 7 meaning Sunday
    pub fn weekday_unchecked(&self) -> u8 {
        self.calculate_2digit_bcd(42, 44)
    }

    /// Return the current month of the year (without verifying the information)
    pub fn month_unchecked(&self) -> u8 {
        self.calculate_2digit_bcd(45, 49)
    }

    /// Return the current year (without verifying the information)
    pub fn year_unchecked(&self) -> u16 {
        let century: u16 = 2000;
        century + <u8 as Into<u16>>::into(self.calculate_2digit_bcd(50, 57))
    }

    /// Return a tuple of (year, month, day, weekday) if it passes a parity check
    pub fn date(&self) -> Result<(u16, u8, u8, u8), ()> {
        let parity = self.calculate_parity(36, 57);

        if ((self.0 & (1 << 58)) != 0) != parity {
            return Err(());
        }

        let year = self.year_unchecked();
        let month = self.month_unchecked();
        let day = self.day_unchecked();
        let weekday = self.weekday_unchecked();

        if year > 2100 || month > 12 || day > 31 || weekday > 7 {
            Err(())
        } else {
            Ok((year, month, day, weekday))
        }
    }

    fn calculate_parity(&self, start: usize, end: usize) -> bool {
        let mut parity = false;
        let mut mask: u64 = 1 << start;
        for _ in start..=end {
            parity ^= (self.0 & mask) != 0;
            mask = mask.wrapping_shl(1);
        }
        parity
    }

    fn calculate_2digit_bcd(&self, start: usize, end: usize) -> u8 {
        let length = end - start + 1;
        let mask = (1u64 << length) - 1;
        let bcd = (self.0 >> start) & mask;
        let high_nibble: u8 = (bcd & 0xF0) as u8 >> 4;
        let low_nibble: u8 = bcd as u8 & 0x0F;
        high_nibble * 10 + low_nibble
    }
}

enum SimpleDCF77DecoderState {
    WaitingForPhase,
    PhaseFound,
    BitReceived,
    FaultyBit,
    EndOfCycle,
    Idle,
}

/// A structure for a simple timeslot based DCF77 decoder
pub struct SimpleDCF77Decoder {
    /// Number of samples since the last phase change, that always starts with a high signal and is
    /// max 2000 ms long
    sample_count: u8,
    /// Number of high samples during the first 100 ms in the scan phase to check if it might be a
    /// transmitted 0
    zero_bit_count: u8,
    /// Number of high samples between 100 and 200 ms in the scan phase to check if it might be a
    /// transmitted 1
    one_bit_count: u8,
    /// Number of non-idle samples after a valid bit was detected
    non_idle_count: u8,
    /// Current state of the decoder
    state: SimpleDCF77DecoderState,
    /// The raw data received from the DCF77 signal
    data: u64,
    /// Current position in the bitstream
    data_pos: usize,
}

/// The SimpleDCF77Decoder implements a simple state machine to decode a DCF77 signal from a fed-in
/// readout of a GPIO pin connected to a DCF77 receiver. To use this, create the structure, set up
/// the GPIO pin the receiver is connected to as an input and call the `read_bit` method every
/// 10ms with a parameter value of `true` for a high signal (low rf amplitude) level or `false` for
/// a low signal level (high rf amplitude).
impl SimpleDCF77Decoder {
    /// Create a new decoder state machine
    pub fn new() -> Self {
        Self {
            sample_count: 0,
            zero_bit_count: 0,
            one_bit_count: 0,
            non_idle_count: 0,
            state: SimpleDCF77DecoderState::WaitingForPhase,
            data: 0,
            data_pos: 0,
        }
    }

    /// Return the raw data as `u64` value for decoding of the current date/time
    pub fn raw_data(&self) -> u64 {
        self.data
    }

    /// Returns true as soon as an individual bit was received
    pub fn bit_complete(&self) -> bool {
        match self.state {
            SimpleDCF77DecoderState::BitReceived => true,
            _ => false,
        }
    }

    /// Returns true if the last bit couldn't be identified as high/low
    pub fn bit_faulty(&self) -> bool {
        match self.state {
            SimpleDCF77DecoderState::FaultyBit => true,
            _ => false,
        }
    }

    /// Returns true if the end of a 59s cycle was detected
    pub fn end_of_cycle(&self) -> bool {
        match self.state {
            SimpleDCF77DecoderState::EndOfCycle => true,
            _ => false,
        }
    }

    /// Returns the value of the latest received bit. Mainly useful for live display of the
    /// received bits
    pub fn latest_bit(&self) -> bool {
        (self.data & (1 << (self.data_pos - 1))) != 0
    }

    /// Return the current position of the bit counter after the latest recognized end of a cycle
    /// which is identical to the current second of the minute
    pub fn seconds(&self) -> usize {
        self.data_pos
    }

    /// Ingest the latest sample of the GPIO input the DCF77 receiver is connected to judge the /
    /// current position and value of the DCF77 signal bitstream
    pub fn read_bit(&mut self, bit: bool) {
        self.state = match self.state {
            // wait for the first phase change 0->1 or abort if no phase change is detected within
            // 1800 ms (180 samples)
            SimpleDCF77DecoderState::EndOfCycle
            | SimpleDCF77DecoderState::WaitingForPhase
            | SimpleDCF77DecoderState::FaultyBit => {
                if bit {
                    self.zero_bit_count = 1;
                    self.one_bit_count = 0;
                    self.sample_count = 0;
                    self.non_idle_count = 0;
                    SimpleDCF77DecoderState::PhaseFound
                } else {
                    if self.sample_count > 180 {
                        self.data_pos = 0;
                        self.sample_count = 0;
                        SimpleDCF77DecoderState::EndOfCycle
                    } else {
                        SimpleDCF77DecoderState::WaitingForPhase
                    }
                }
            }
            // count the number of high bits in the first 100 ms and the second 100 ms to determine
            // if a 0 or 1 was transmitted
            SimpleDCF77DecoderState::PhaseFound => {
                if self.sample_count < 20 {
                    if bit {
                        if self.sample_count < 10 {
                            self.zero_bit_count += 1;
                        } else {
                            self.one_bit_count += 1;
                        }
                    }
                    SimpleDCF77DecoderState::PhaseFound
                } else {
                    let data_pos = self.data_pos;
                    self.data_pos += 1;
                    if self.one_bit_count > 3 {
                        self.data |= 1 << data_pos;
                        SimpleDCF77DecoderState::BitReceived
                    } else if self.zero_bit_count > 3 {
                        self.data &= !(1 << data_pos);
                        SimpleDCF77DecoderState::BitReceived
                    } else {
                        // Bad signal, let's start over
                        self.data_pos = 0;
                        SimpleDCF77DecoderState::FaultyBit
                    }
                }
            }
            // wait until the 900 ms of the bit are over and then check if the signal was not idle
            // for max 10 samples to start the next bit
            SimpleDCF77DecoderState::BitReceived | SimpleDCF77DecoderState::Idle => {
                if bit {
                    self.non_idle_count += 1;
                }

                if self.sample_count >= 90 {
                    if self.non_idle_count < 10 {
                        SimpleDCF77DecoderState::WaitingForPhase
                    }
                    else{
                        // Bad signal, let's start over
                        self.data_pos = 0;
                        SimpleDCF77DecoderState::FaultyBit
                    }
                } else {
                    SimpleDCF77DecoderState::Idle
                }
            }
        };

        self.sample_count += 1;
    }
}
