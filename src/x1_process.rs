use std::ffi::c_float;
use std::sync::mpsc;
use std::time::Duration;

use midir::{MidiInput, MidiInputConnection, MidiOutput, MidiOutputConnection};
use midir::os::unix::{VirtualInput, VirtualOutput};
use rusb::{Device, DeviceHandle, UsbContext};

use crate::conf::YamlConfig;
use crate::utils::{hex2bin, hex2bool, knob_to_midi};
use crate::x1_board::{ButtonType, X1mk1Board};

const USB_WRITE_FD: u8 = 0x01;
const USB_UNLOCK_FD: u8 = 0x81;
const USB_READ_FD: u8 = 0x84;
const LED_DIM: u8 = 0x05;
const LED_DIM_PULSE: u8 = 0x25;
const LED_BRIGHT: u8 = 0x7F;                
const MIDI_CHANNEL: u8 = 0xB0;              // Base MIDI channel for buttons/knobs/encoders
const MIDI_CHANNEL_LED: u8 = 0xB2;          // Base MIDI channel for LED control
const MIDI_CHANNEL_HOTCUE: u8 = 0xB3;       // Base MIDI channel for HOTCUE buttons

struct BlinkState {
    set_value: u8,
    value: c_float,
    direction: bool,
    speed: c_float,
    min: c_float,
    max: c_float,
}

impl BlinkState {
    fn blinky_compute(&mut self) {                
        if self.direction {
            if self.value < self.max {
                self.value += self.speed;
            } else {
                self.direction = false;
                self.value -= self.speed;
            }
        } else {
            if self.value > self.min {
                self.value -= self.speed;
            } else {
                self.direction = true;
                self.value += self.speed;
            }
        }
    }

    fn blink_value(&mut self) -> u8 {
        self.blinky_compute();                
        return self.value as u8;
    }

    fn set_dim(&mut self) {
        self.set_value = LED_DIM;
        self.speed= 0.25;   // to match 40 frames
        self.reset();
    }

    fn set_bright(&mut self) {
        self.set_value = LED_BRIGHT;
        self.speed= 2.5;    // to match 40 frames
        self.reset();
    }

    fn reset(&mut self) {
        println!("Resetting BlinkState of {}", self.value);
        self.value = self.set_value.into();
        self.direction = true;
        if self.set_value == LED_DIM {
            self.min = LED_DIM as c_float;
            self.max = 3.0 * (LED_DIM as c_float);
        } else {
            self.min = LED_BRIGHT as c_float - 100.0;
            self.max = LED_BRIGHT as c_float;
        }
        self.value = self.min;
    }
}
        

const BLINKSTATE_DIM: BlinkState = BlinkState {
    set_value: LED_DIM,
    value: LED_DIM as c_float,
    direction: true,
    speed: 0.25,
    min: LED_DIM as c_float,
    max: 2.0*(LED_DIM as c_float),
};

const BLINKSTATE_BRIGHT: BlinkState = BlinkState {
    set_value: LED_BRIGHT,
    value: LED_BRIGHT as c_float,
    direction: true,
    speed: 2.5,
    min: LED_BRIGHT as c_float - 100.0,
    max: LED_BRIGHT as c_float,
};
pub struct X1mk1<T: UsbContext> {
    pub device: Device<T>,
    pub handle: DeviceHandle<T>,
    pub serial_number: String,
    midi_conn_out: MidiOutputConnection,
    midi_conn_in: Option<MidiInputConnection<()>>,
    board: X1mk1Board,
    usb_buffer: [u8; 24],
    usb_timeout: Duration,
    usb_endpoint: Endpoint,
    led: [u8; 32],
    led_hotcue: [u8; 16],
    led_layerindicator: [BlinkState; 4],
    led_blinky: u8,
    led_blinky_direction: bool,
    shift: u8,
    shiftHotcue: u8,
    layer_a: u8,
    layer_b: u8,
    hotcue: bool,
}

struct Endpoint {
    config: u8,
    interface: u8,
    setting: u8,
    address: u8,
}

impl<T: UsbContext> X1mk1<T> {
    pub fn new(device: Device<T>, handle: DeviceHandle<T>, serial_number: String, yaml_config: YamlConfig) -> Self {
        let midi_out = MidiOutput::new("MIDI Kontrol X1 Mk1").unwrap();
        let midi_conn_out = midi_out.create_virtual(serial_number.as_str()).unwrap();
        let board = X1mk1Board::from_yaml(&yaml_config);
        let mut leds = [0x05; 32];
        let led_hotcue = [0x05; 16];
        leds[0] = 0x0C;
        leds[31] = 0;
        let usb_buffer = [0; 24];
        let usb_endpoint = Endpoint {
            address: USB_READ_FD,
            config: 1,
            interface: 0,
            setting: 0,
        };

        Self {
            device,
            handle,
            serial_number,
            midi_conn_out,
            midi_conn_in: None,
            board,
            usb_buffer,
            usb_timeout: Duration::from_millis(50),
            usb_endpoint,
            led: leds,
            led_hotcue,
            led_layerindicator: [BLINKSTATE_DIM; 4],
            led_blinky: LED_DIM,
            led_blinky_direction: true,
            shift: 0,
            shiftHotcue: 0,
            layer_a: 0,
            layer_b: 0,
            hotcue: false,
        }
    }

    pub(crate) fn init(&mut self, sender: mpsc::Sender<Vec<u8>>) {
        let midi_in = MidiInput::new("MIDI Kontrol X1 Mk1").unwrap();
        let midi_conn_in = midi_in.create_virtual(
            self.serial_number.as_str(),
            move |_stamp, message, _| {
                sender.send(message.to_vec()).unwrap();
            }, ()).unwrap();
        self.midi_conn_in = Some(midi_conn_in); // Prevents the connection from being dropped
    }

    pub(crate) fn read(&mut self) -> rusb::Result<()> {
        println!("Reading from {}", self.serial_number);
        let (midi_tx, midi_rx) = mpsc::channel::<Vec<u8>>();

        self.init(midi_tx);
        self.configure_endpoint()?;
        self.update_leds();
        loop {
            match midi_rx.try_recv() {
                Ok(message) => {
                    let i = message[1] as usize;
                    if (0..32).contains(&i) {
                        if message[0] == MIDI_CHANNEL_LED {
                            self.led[i] = message[2];
                        } else if message[0] == MIDI_CHANNEL_HOTCUE {
                            self.led_hotcue[i] = if message[2] != 0 { LED_BRIGHT } else { LED_DIM };
                        }
                        self.update_leds();
                    } else {
                        eprintln!("Invalid LED index: {}", i)
                    }
                }
                Err(_) => {}
            }
            match self.handle.read_bulk(self.usb_endpoint.address, &mut self.usb_buffer, self.usb_timeout) {
                Ok(len) => {
                    if len != self.usb_buffer.len() {
                        // rusb crate consider partially read data as ok but we do not.
                        continue;
                    }
                    self.read_state(self.usb_buffer);
                }
                Err(e) => {
                    if e == rusb::Error::Timeout {
                        // Weird timeout occurring when all knobs are at 0 position and no button is pressed.
                        // We do not want to break because there's no need to call configure_endpoint again.
                        continue;
                    }
                    return Err(e);
                }
            }
            self.update_leds();
        }
    }

    fn configure_endpoint(&mut self) -> rusb::Result<()> {
        self.handle.set_active_configuration(self.usb_endpoint.config)?;
        self.handle.claim_interface(self.usb_endpoint.interface)?;
        self.handle.set_alternate_setting(self.usb_endpoint.interface, self.usb_endpoint.setting)?;
        Ok(())
    }

    fn read_state(&mut self, buf: [u8; 24]) {
        let mut binbyte: [[bool; 8]; 5] = [[false; 8]; 5];

        hex2bool(buf[1], &mut binbyte[0]);
        hex2bool(buf[2], &mut binbyte[1]);
        hex2bool(buf[3], &mut binbyte[2]);
        hex2bool(buf[4], &mut binbyte[3]);
        hex2bool(buf[5], &mut binbyte[4]);

        for (ctrl_name, button_type) in &mut self.board.buttons {
            match button_type {
                ButtonType::Toggle(ref mut button) => {
                    if self.hotcue && button.hotcue_ignore {
                        continue;
                    }
                    button.curr = binbyte[button.read_i as usize][button.read_j as usize];
                    if button.curr != button.prev {
                        if button.curr {
                            //self.led[button.write_idx as usize] = LED_DIM_PULSE; // push feedback
                            let _l = self.led[button.write_idx as usize];
                            let mut layer_offset = 0;
                            if button.layer_a {
                                layer_offset = self.layer_a;
                            } else if button.layer_b {
                                layer_offset = self.layer_b;
                            }
                            println!("Button {} pressed layerOffset={} {} {}", ctrl_name, layer_offset, self.layer_a, self.layer_b);
                            let mut skip = false;
                            if ctrl_name.eq("DECK_A_BUTTON_FX1") {
                                if self.shiftHotcue == 1 {
                                    //self.led[button.write_idx as usize] = LED_DIM_PULSE;
                                    if self.layer_a == 1 {
                                        self.layer_a = 0;
                                    } else {
                                        self.layer_a = 1;
                                    }
                                    skip = true;
                                }
                                if self.led[button.write_idx as usize] == LED_DIM {
                                    self.led_layerindicator[0].set_dim();
                                } else {
                                    self.led_layerindicator[0].set_bright();
                                }
                                println!("led = {}", self.led[button.write_idx as usize]);
                                for layerindicator in &mut self.led_layerindicator {
                                    layerindicator.reset();
                                }
                                println!("Layer A set to {} {}", self.layer_a, self.shiftHotcue);
                                //self.led[button.write_idx as usize] = LED_DIM_PULSE;
                                //LED_DIM_PULSE
                            }
                            if ctrl_name.eq("DECK_A_BUTTON_FX2") {
                                if self.shiftHotcue == 1 {
                                    //self.led[button.write_idx as usize] = LED_DIM_PULSE;
                                    if self.layer_a == 2 {
                                        self.layer_a = 0;
                                    } else {
                                        self.layer_a = 2;
                                    }
                                    skip = true;
                                }
                                if self.led[button.write_idx as usize] == LED_DIM {
                                    self.led_layerindicator[1].set_dim();
                                    } else {
                                    self.led_layerindicator[1].set_bright();
                                }
                                println!("led = {}", self.led[button.write_idx as usize]);
                                for layerindicator in &mut self.led_layerindicator {
                                    layerindicator.reset();
                                }

                                println!("Layer A set to {} {}", self.layer_a, self.shiftHotcue);
                                //self.led[button.write_idx as usize] = LED_DIM_PULSE;
                                //LED_DIM_PULSE
                            }
                            if ctrl_name.eq("DECK_B_BUTTON_FX1") {
                                if self.shiftHotcue == 1 {
                                    //self.led[button.write_idx as usize] = LED_DIM_PULSE;
                                    if self.layer_b == 1 {
                                        self.layer_b = 0;
                                    } else {
                                        self.layer_b = 1;
                                    }
                                    skip = true;
                                }
                                if self.led[button.write_idx as usize] == LED_DIM {
                                    self.led_layerindicator[2].set_dim();
                                } else {
                                    self.led_layerindicator[2].set_bright();
                                }
                                println!("led = {}", self.led[button.write_idx as usize]);
                                for layerindicator in &mut self.led_layerindicator {
                                    layerindicator.reset();
                                }
                                println!("Layer B set to {} {}", self.layer_b, self.shiftHotcue);
                                //self.led[button.write_idx as usize] = LED_DIM_PULSE;
                                //LED_DIM_PULSE
                            }
                            if ctrl_name.eq("DECK_B_BUTTON_FX2") {
                                if self.shiftHotcue == 1 {
                                    //self.led[button.write_idx as usize] = LED_DIM_PULSE;
                                    if self.layer_b == 2 {
                                        self.layer_b = 0;
                                    } else {
                                        self.layer_b = 2;
                                    }
                                    skip = true;
                                }
                                if self.led[button.write_idx as usize] == LED_DIM {
                                    self.led_layerindicator[3].set_dim();
                                } else {
                                    self.led_layerindicator[3].set_bright();
                                }
                                println!("led = {}", self.led[button.write_idx as usize]);
                                for layerindicator in &mut self.led_layerindicator {
                                    layerindicator.reset();
                                }
                                println!("Layer B set to {} {}", self.layer_b, self.shiftHotcue);
                                //self.led[button.write_idx as usize] = LED_DIM_PULSE;
                                //LED_DIM_PULSE
                            }
                            if !skip {  // skip sending midi for active layer buttons
                                let _ = self.midi_conn_out.send(&[MIDI_CHANNEL + self.shift + 2*layer_offset, button.midi_ctrl_ch, 127]);
                            }
                            if ctrl_name.eq("HOTCUE") && self.shift == 0 {
                                self.hotcue = !self.hotcue;
                                self.led[button.write_idx as usize] = if self.hotcue { LED_BRIGHT } else { LED_DIM };
                            } else if ctrl_name.eq("HOTCUE") && self.shift == 1 {
                                self.shiftHotcue = if self.shiftHotcue == 0 { 1 } else { 0 };
                                for layerindicator in &mut self.led_layerindicator {
                                    layerindicator.reset();
                                }
                            }
                        } else {
                            //this needs the current button status for toggle buttons
                            //self.led[button.write_idx as usize] = LED_DIM; // push feedback
                        }
                    }
                    button.prev = button.curr;
                }
                ButtonType::Hold(ref mut button) => {
                    if self.hotcue && button.hotcue_ignore {
                        continue;
                    }

                    let mut layer_offset = 0;
                    if button.layer_a {
                        layer_offset = self.layer_a;
                    } else if button.layer_b {
                        layer_offset = self.layer_b;
                    }

                    button.curr = binbyte[button.read_i as usize][button.read_j as usize];
                    if button.curr == button.prev {
                        continue;
                    } else if button.curr {    
                        if self.shiftHotcue == 1 {
                            // LED debug output for self.shiftHotcue
                            self.led[button.write_idx as usize] = LED_BRIGHT;
                        } else {
                            self.led[button.write_idx as usize] = LED_DIM_PULSE;
                        }

                        let _ = self.midi_conn_out.send(&[MIDI_CHANNEL + self.shift + 2*layer_offset, button.midi_ctrl_ch, 127]);
                        if ctrl_name.eq("SHIFT") {
                            self.shift = 1;
                        }
                    } else {
                        self.led[button.write_idx as usize] = LED_DIM;
                        if ctrl_name.eq("SHIFT") {
                            self.shift = 0;
                        }
                        let _ = self.midi_conn_out.send(&[MIDI_CHANNEL + self.shift + 2*layer_offset, button.midi_ctrl_ch, 0]);
                    }
                    button.prev = button.curr;
                }
                ButtonType::Hotcue(ref mut button) => {
                    if !self.hotcue {
                        continue;
                    }
                    button.curr = binbyte[button.read_i as usize][button.read_j as usize];
                    if button.curr == button.prev {
                        continue;
                    } else if button.curr {
                        let _ = self.midi_conn_out.send(&[MIDI_CHANNEL_HOTCUE + self.shift, button.midi_ctrl_ch, 127]);
                    } else {
                        let _ = self.midi_conn_out.send(&[MIDI_CHANNEL_HOTCUE + self.shift, button.midi_ctrl_ch, 0]);
                    }
                    button.prev = button.curr;
                }
                ButtonType::Knob(ref mut knob) => {
                    knob.curr = knob_to_midi(buf[knob.read_i as usize], buf[knob.read_j as usize]);
                    if knob.curr != knob.prev {
                        let mut layer_offset = 0;
                        if knob.layer_a {
                            layer_offset = self.layer_a;
                        } else if knob.layer_b {
                            layer_offset = self.layer_b;
                        }
                        let _ = self.midi_conn_out.send(&[MIDI_CHANNEL + self.shift + 2*layer_offset, knob.midi_ctrl_ch, knob.curr]);
                    }
                    knob.prev = knob.curr;
                }
                ButtonType::Encoder(ref mut encoder) => {
                    let mut binnum = [0; 8];
                    hex2bin(buf[encoder.read_i as usize], &mut binnum);
                    match encoder.read_pos {
                        's' => {
                            encoder.curr = binnum[0] + binnum[1] * 2 + binnum[2] * 4 + binnum[3] * 8;
                        }
                        'e' => {
                            encoder.curr = binnum[4] + binnum[5] * 2 + binnum[6] * 4 + binnum[7] * 8;
                        }
                        _ => panic!("Invalid read_pos"),
                    }
                    if encoder.curr != encoder.prev {
                        // Clockwise init
                        let mut velocity = 1;
                        if (encoder.prev == 15 && encoder.curr == 0) || (encoder.prev == 0 && encoder.curr == 15) {
                            // Full rotation special case
                            velocity = if encoder.prev == 15 { 1 } else { 127 };
                        } else if encoder.curr < encoder.prev {
                            // Clockwise
                            velocity = 127;
                        }
                        let mut layer_offset = 0;
                        if encoder.layer_a {
                            layer_offset = self.layer_a;
                        } else if encoder.layer_b {
                            layer_offset = self.layer_b;
                        }
                        let _ = self.midi_conn_out.send(&[MIDI_CHANNEL + self.shift + 2*layer_offset, encoder.midi_ctrl_ch, velocity]);
                    }
                    encoder.prev = encoder.curr;
                }
            }
        }
    }
    
    fn update_leds(&mut self) {
        let mut led = self.led;
        if self.hotcue {
            for i in 9..25 {
                led[i] = self.led_hotcue[i - 9];
            }
        }
        
        if self.layer_a > 0 || self.layer_b > 0 {
            if self.layer_a == 1 {
                led[25] = self.led_layerindicator[0].blink_value(); // deckA FX1 mode LED
            } else if self.layer_a == 2 {
                led[26] = self.led_layerindicator[1].blink_value(); // deckA FX2 mode LED
            }
            if self.layer_b == 1 {
                led[27] = self.led_layerindicator[2].blink_value(); // deckB FX1 mode LED
            } else if self.layer_b == 2 {
                led[28] = self.led_layerindicator[3].blink_value(); // deckB FX2 mode LED
            }
        } else {
            //led[0] = LED_DIM_PULSE; // normal mode LED
        }
        self.handle.write_bulk(USB_WRITE_FD, &led, self.usb_timeout).unwrap();
        match self.handle.read_bulk(USB_UNLOCK_FD, &mut [0; 1], self.usb_timeout) {
            Ok(_) => {}
            Err(e) => {
                eprintln!("Error reading from device: {:?}", e);
            }
        };
    }
}
