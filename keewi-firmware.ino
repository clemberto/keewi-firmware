/* Copyright (c) 2014, Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


/** @defgroup ble_uart_project_template ble_uart_project_template
@{
@ingroup projects
@brief Empty project that can be used as a template for new projects.

@details
This project is a firmware template for new projects.
The project will run correctly in its current state.
It can send data on the UART TX characteristic
It can receive data on the UART RX characterisitc.
With this project you have a starting point for adding your own application functionality.

The following instructions describe the steps to be made on the Windows PC:

 -# Install the Master Control Panel on your computer. Connect the Master Emulator
    (nRF2739) and make sure the hardware drivers are installed.

-# You can use the nRF UART app in the Apple iOS app store and Google Play for Android 4.3 for Samsung Galaxy S4
   with this UART template app

-# You can send data from the Arduino serial monitor, maximum length of a string is 19 bytes
   Set the line ending to "Newline" in the Serial monitor (The newline is also sent over the air

 *
 * Click on the "Serial Monitor" button on the Arduino IDE to reset the Arduino and start the application.
 * The setup() function is called first and is called only once for each reset of the Arduino.
 * The loop() function as the name implies is called in a loop.
 *
 * The setup() and loop() function are called in this way.
 * main()
 *  {
 *   setup();
 *   while(1)
 *   {
 *     loop();
 *   }
 * }
 *
 */
#include <SPI.h>

// Bluetooth LE variables:
#include <lib_aci.h>
#include <aci_setup.h>
#include "uart_over_ble.h"
#include "services.h"
static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;
static struct aci_state_t aci_state;
static hal_aci_evt_t  aci_data;
static bool timing_change_done          = false;
static uart_over_ble_t uart_over_ble;
static uint8_t         uart_buffer[20];
static uint8_t         uart_buffer_len = 0;
static uint8_t         dummychar = ' ';

/*
Description:

From uart project template: BTLE as a UART  can send and receive packets.
The maximum size of a packet is 20 bytes.
When a command it received a response(s) are transmitted back.
Since the response is done using a Notification the peer must have opened it(subscribed to it) before any packet is transmitted.
The pipe for the UART_TX becomes available once the peer opens it.
See section 20.4.1 -> Opening a Transmit pipe
In the master control panel, clicking Enable Services will open all the pipes on the nRF8001.

The ACI Evt Data Credit provides the radio level ack of a transmitted packet.
*/

// Keewi v0 board global variables:
#define KEEWI_V0_ACCEL_INT        2
#define KEEWI_V0_BT_READY         3
#define KEEWI_V0_BT_CS            4
#define KEEWI_V0_MEM_PROTECT      5
#define KEEWI_V0_VMCU_CADENCE     6

#define KEEWI_V0_PWRGOOD          A0
#define KEEWI_V0_BAT_VUSB_EN      A1
#define KEEWI_V0_RED_LED          A2
#define KEEWI_V0_GREEN_LED        A3
#define KEEWI_V0_BLUE_LED         A4
#define KEEWI_V0_VMCU_SPEED       A6

#define KEEWI_V0_MEM_CS           8
#define KEEWI_V0_BUZZER           9
#define KEEWI_V0_VDRIVE_LIGHTS    10

int prev_cadi = 0;                // cadence spikes (i-1)
int cadi = 0;                     // cadence spikes (i)
int cadence = 0;                  // cadence (rpm)
int cadence_avg = 0;              // average cadence (4s)
int prev_spi = 0;                 // dyamo spikes (i-1)
int spi = 1;                      // dyamo spikes (i)
float sp = 0;                     // speed (km/h)
int sp_avg = 0;                   // average speed (4s)
static volatile int cnt = 0;      // Timer2 counter
bool light_sw = 0;                // Light switch state
bool bat_vusb = 0;                // Battery supplying to USB state
bool charg_state = 0;             // DC-DC state

void setup(void) {
  cli();              // Disable interrupts
  
  // Debug output is 14400bauds on Keewi v0 board
  Serial.begin(115200);
  Serial.println(F("Keewi in UART mode!"));
  Serial.println(F("Connect to 'URT' AP with Keewi0 app"));

  // Keewi v0 board config
  pinMode(KEEWI_V0_BT_READY, INPUT);
  pinMode(KEEWI_V0_BT_CS, OUTPUT);
  pinMode(KEEWI_V0_VDRIVE_LIGHTS, OUTPUT);
  pinMode(KEEWI_V0_VMCU_CADENCE, INPUT);
  pinMode(KEEWI_V0_MEM_CS, OUTPUT);
  pinMode(KEEWI_V0_PWRGOOD, INPUT);
  pinMode(KEEWI_V0_RED_LED, OUTPUT);
  pinMode(KEEWI_V0_GREEN_LED, OUTPUT);
  pinMode(KEEWI_V0_BLUE_LED, OUTPUT);
  digitalWrite(KEEWI_V0_RED_LED, HIGH);
  digitalWrite(KEEWI_V0_GREEN_LED, LOW);
  digitalWrite(KEEWI_V0_BLUE_LED, HIGH);
  // Analog comparator
  ACSR = 0b01011111;      // Comparator ON, Ref voltage ON, Clear Inter. flag, Inter. enable ON, Capture to T1 ON, Inter. trig on Rising Edge
  ADMUX = 0b01000110;      // Volt. ref. AVcc, No left adj., ADC6 channel sel.
  ADCSRA = 0b00000000;      // ADEN = 0, No start, no auto conv.
  ADCSRB = 0b01000000;    // Comp. mux ON, Trigger free running
  // Timer1 20ms
  TIMSK1 = _BV(TOIE1);  // (Enable) Overflow Interrupt 
  TCCR1A = 0x00;      // (Set) Normal mode (counting up), no compare outputs A&B
  TCCR1B = 0b01000011; //  Positive edge det., No wav. out, No noise can., prescale 64
  // Timer2 327ms
  TIFR2  = 0x00;        // Clear Timer Overflow Flag
  TIMSK2  = 0x01;        // Overflow Interrupt Enable
  TCCR2A  = 0x00;        // Disable Wave Gen Mode normal
  TCCR2B  = 0x07;        // 1024 Timer prescaler.
  // PCINT22 interrupt
  PCICR = _BV(PCIE2);     // (Enable) pin change inter. on PCINT[23:16]
  PCMSK2 = _BV(PCINT22);  // (Set) pin change inter. on PCINT22 only
  
  // BT protocol (ACI) config
  aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;
  aci_state.aci_pins.board_name = BOARD_DEFAULT;
  aci_state.aci_pins.reqn_pin   = KEEWI_V0_BT_CS;
  aci_state.aci_pins.rdyn_pin   = KEEWI_V0_BT_READY;
  aci_state.aci_pins.mosi_pin   = MOSI;
  aci_state.aci_pins.miso_pin   = MISO;
  aci_state.aci_pins.sck_pin    = SCK;
  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 256kHz SPI speed
                                                             //SPI_CLOCK_DIV16 = 128MHz SPI speed
  aci_state.aci_pins.reset_pin              = UNUSED;
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;
  aci_state.aci_pins.interface_is_interrupt = false;
  aci_state.aci_pins.interrupt_number       = UNUSED;

  lib_aci_init(&aci_state, false);

  sei();              // Enable interrupts
}

ISR(ANALOG_COMP_vect) {
  // Edge detected on Dynamo input
    spi = ICR1L + (0x7FFF & (ICR1H<<8));
    uint16_t lSpi_diff = spi - prev_spi;

    if (lSpi_diff > 0) {
      // lSpi_diff is the time interval in µs between 2 sensor spikes
      sp = (7920 / lSpi_diff);        // PI*700mm = 2200mm = 7920µs.km/h = wheel linear length
    }
    else {
      // Not valid
      sp = 0;
      Serial.println("Wrong speed data");
    }
    //Serial.println(sp);
    prev_spi = spi;

    ACSR = 0b01011111;        // Clear analog comp. flag
}

ISR(TIMER1_OVF_vect) {
  // Counter overflow: signal it for speed calculation
  sp = 0;
  cadence = 0;
  TIFR1 = 0x01;      // Clear Overflow inter. flag

}

ISR(TIMER2_OVF_vect) {

  sp_avg += sp;
  cadence_avg += cadence;
  // Periodiccally sends Data to UART
  if (cnt++ == 10) {
    if (prev_spi == spi)    // No dynamo spike were received
      sp = 0;
    if (prev_cadi == cadi)    // No cadence spike were received
      cadence = 0;
    charg_state = digitalRead(KEEWI_V0_PWRGOOD);
    char led_state = digitalRead(KEEWI_V0_RED_LED) * 0x04 + digitalRead(KEEWI_V0_GREEN_LED) * 0x02 + digitalRead(KEEWI_V0_BLUE_LED);
    uart_buffer[0] = (char)( (sp_avg/110) % 10 ) + 0x30;
    uart_buffer[1] = (char)( (sp_avg/11) % 10 ) + 0x30;
    uart_buffer[2] = dummychar;
    uart_buffer[3] = (char)( (cadence_avg/110) % 10 ) + 0x30;
    uart_buffer[4] = (char)( (cadence_avg/11) % 10 ) + 0x30;
    uart_buffer[5] = dummychar;
    uart_buffer[6] = (char)light_sw + 0x30;
    uart_buffer[7] = dummychar;
    uart_buffer[8] = (char)bat_vusb + 0x30;
    uart_buffer[9] = dummychar;
    uart_buffer[10] = (char)charg_state + 0x30;
    uart_buffer[11] = dummychar;
    uart_buffer[12] = (char)led_state + 0x30;
    lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, uart_buffer, 13);
    cnt = 0;
    sp_avg = 0;
    cadence_avg = 0;
  }
  TCNT2  = 0;           // (Clear) Timer2 
  TIFR2  = 0x01;        // (Clear) Timer2 overflow flag

}

ISR(PCINT2_vect) {
  // An edge has been detected on Cadence sensor input
  // Debounce it! (0.8ms)
  delay(1);
  if ( digitalRead(KEEWI_V0_VMCU_CADENCE) == HIGH )
      return;
      
  // Read Timer1 value & compute interval
  cadi = ( 0x7FFF & (TCNT1H << 8) ) + TCNT1L ;
  uint16_t lCadi_diff = cadi - prev_cadi;

  if (lCadi_diff > 0) {
    // lCadi_diff is the time interval in µs between 2 sensor spikes
    cadence = (7920 / lCadi_diff);      // PI*700mm = 2200mm = 7920µs.km/h = wheel linear length
    Serial.print(cadence);
  }
  else {
    // Not valid
    cadence = 0;
    Serial.println("Wrong cadence data");
  }
  
  prev_cadi = cadi; // Update for next event
  
  PCIFR |=  _BV(PCIF2);       // (Clear) pin change inter. flag
}


// UART over BLE implementation
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("General BLE ERROR!");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}

void uart_over_ble_init(void) {
    uart_over_ble.uart_rts_local = true;
}

bool uart_tx(uint8_t *buffer, uint8_t buffer_len) {
  bool status = false;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) &&
      (aci_state.data_credit_available >= 1))
  {
    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, buffer_len);
    if (status)
      aci_state.data_credit_available--;
  }
  return status;
}

bool uart_process_control_point_rx(uint8_t *byte, uint8_t length) {
  bool status = false;
  aci_ll_conn_params_t *conn_params;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX) )
  {
    Serial.println(*byte, HEX);
    switch(*byte)
    {
      /*
      Queues a ACI Disconnect to the nRF8001 when this packet is received.
      May cause some of the UART packets being sent to be dropped
      */
      case UART_OVER_BLE_DISCONNECT:
        /*
        Parameters:
        None
        */
        lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
        status = true;
        break;

      /*
      Queues an ACI Change Timing to the nRF8001
      */
      case UART_OVER_BLE_LINK_TIMING_REQ:
        /*
        Parameters:
        Connection interval min: 2 bytes
        Connection interval max: 2 bytes
        Slave latency:           2 bytes
        Timeout:                 2 bytes
        Same format as Peripheral Preferred Connection Parameters (See nRFgo studio -> nRF8001 Configuration -> GAP Settings
        Refer to the ACI Change Timing Request in the nRF8001 Product Specifications
        */
        conn_params = (aci_ll_conn_params_t *)(byte+1);
        lib_aci_change_timing( conn_params->min_conn_interval,
                                conn_params->max_conn_interval,
                                conn_params->slave_latency,
                                conn_params->timeout_mult);
        status = true;
        break;

      /*
      Clears the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_STOP:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = false;
        status = true;
        break;

      /*
      Set the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_OK:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = true;
        status = true;
        break;
    }
  }
  return status;
}

void aci_loop()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;

    switch(aci_evt->evt_opcode)
    {
      /**
      As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            Serial.println(F("Evt Device Started: Setup"));
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            Serial.println(F("Evt Device Started: Standby"));
            //Looking for an iPhone by sending radio advertisements
            //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Magic number used to make sure the HW error event is handled correctly.
            }
            else
            {
            lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
            Serial.println(F("Advertising started"));
            }
            break;
        }
      }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.print(F("Evt Cmd respone: Status "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
            (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
        break;

      case ACI_EVT_CONNECTED:
        Serial.println(F("Evt Connected"));
        uart_over_ble_init();
        timing_change_done              = false;
        aci_state.data_credit_available = aci_state.data_credit_total;

        /*
        Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        lib_aci_device_version();
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done)) {
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                                            // Used to increase or decrease bandwidth
          timing_change_done = true;
        }
        break;

      case ACI_EVT_TIMING:
        Serial.println(F("Evt link connection interval changed"));
        lib_aci_set_local_data(&aci_state,
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                                (uint8_t *)&(aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
        break;

      case ACI_EVT_DISCONNECTED:
        Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
        Serial.println(F("Advertising started"));
        break;

      case ACI_EVT_DATA_RECEIVED:
        Serial.print(F("Pipe Number: "));
        Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);
        if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number) {
          Serial.print(F(" Data(Hex) : "));
          for(int i=0; i<aci_evt->len - 2; i++) {
            Serial.print((char)aci_evt->params.data_received.rx_data.aci_data[i]);
            uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
            //Serial.print(F(" "));
          }
          uart_buffer_len = aci_evt->len - 2;
          if (uart_buffer_len == 2) {
            light_sw = (uart_buffer[0] == '0' ? 0 : 1);
            bat_vusb = (uart_buffer[1] == '0' ? 0 : 1);
            digitalWrite(KEEWI_V0_VDRIVE_LIGHTS, light_sw);
            digitalWrite(KEEWI_V0_BAT_VUSB_EN, bat_vusb);
            Serial.print(" Lights & charge updated!");
          }
          if (uart_buffer_len == 3) {
            digitalWrite(KEEWI_V0_RED_LED, (uart_buffer[0] == '0' ? 0 : 1));
            digitalWrite(KEEWI_V0_GREEN_LED, (uart_buffer[1] == '0' ? 0 : 1));
            digitalWrite(KEEWI_V0_BLUE_LED, (uart_buffer[2] == '0' ? 0 : 1));
            Serial.print(" Leds updated!");
          }
          Serial.println(F(" "));
          if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX)) {
            /*Do this to test the loopback otherwise comment it out
            */
            /*
            if (!uart_tx(&uart_buffer[0], aci_evt->len - 2))
            {
              Serial.println(F("UART loopback failed"));
            }
            else
            {
              Serial.println(F("UART loopback OK"));
            }
            */
          }
        }
        if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number) {
            uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); //Subtract for Opcode and Pipe number
        }
        break;

      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code) {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();
        lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
        Serial.println(F("Advertising started"));
        break;

    }
  }
  else {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }

  if(setup_required) {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state)) {
      setup_required = false;
    }
  }
}


void loop() {

  //Process any ACI commands or events
  aci_loop();

}

