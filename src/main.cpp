/*
 * Norvi IIoT Signal Lights
 *
 * Using W5500 Ethernet chip, 
 * eModbus library: https://github.com/eModbus/eModbus/blob/7a5f8877e5ee426ae8eca634e96ec1dfd5724209/examples/TCPEthernetServer/main.cpp
 * 
 * 
 * Important: To get it working, line 28 of ~/.platformio/packages/framework-arduinoespressif32/cores/esp32/Server.h 
 * had to be changed from `virtual void begin(uint16_t port=0) =0;` 
 * to `virtual void begin() =0;`
 * 
 */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
// #include <ArduinoModbus.h>
#include "local_config.h"   // <--- Change settings for YOUR network here.


// Modbus server TCP
#include "ModbusServerEthernet.h"

/* 
 * GPIO config for signal lights
 */ 
const int GPIO_GREEN = 2;
const int GPIO_YELLOW = 4;
const int GPIO_RED = 12;
const int GPIO_BUZZER = 13;



void WizReset();
void prt_hwval(uint8_t refval);
void prt_ethval(uint8_t refval);

/*
 * Wiz W5500 reset function.  Change this for the specific reset
 * sequence required for your particular board or module.
 */
void WizReset() {
    Serial.print("Resetting Wiz W5500 Ethernet Board...  ");
    pinMode(RESET_P, OUTPUT);
    digitalWrite(RESET_P, HIGH);
    delay(250);
    digitalWrite(RESET_P, LOW);
    delay(50);
    digitalWrite(RESET_P, HIGH);
    delay(350);
    Serial.println("Done.");
}


/*
 * This is a crock. It's here in an effort
 * to help people debug hardware problems with
 * their W5100 ~ W5500 board setups.  It's 
 * a copy of the Ethernet library enums and
 * should, at the very least, be regenerated
 * from Ethernet.h automatically before the
 * compile starts (that's a TODO item).
 *
 */
/*
 * Print the result of the hardware status enum
 * as a string.
 * Ethernet.h currently contains these values:-
 *
 *  enum EthernetHardwareStatus {
 *      EthernetNoHardware,
 *      EthernetW5100,
 *      EthernetW5200,
 *      EthernetW5500
 *  };
 *
 */
void prt_hwval(uint8_t refval) {
    switch (refval) {
    case 0:
        Serial.println("No hardware detected.");
        break;
    case 1:
        Serial.println("WizNet W5100 detected.");
        break;
    case 2:
        Serial.println("WizNet W5200 detected.");
        break;
    case 3:
        Serial.println("WizNet W5500 detected.");
        break;
    default:
        Serial.println
            ("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
    }
}


/*
 * Print the result of the ethernet connection
 * status enum as a string.
 * Ethernet.h currently contains these values:-
 *
 *  enum EthernetLinkStatus {
 *     Unknown,
 *     LinkON,
 *     LinkOFF
 *  };
 *
 */
void prt_ethval(uint8_t refval) {
    switch (refval) {
    case 0:
        Serial.println("Uknown status.");
        break;
    case 1:
        Serial.println("Link flagged as UP.");
        break;
    case 2:
        Serial.println("Link flagged as DOWN. Check cable connection.");
        break;
    default:
        Serial.println
            ("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
    }
}

void checkConnection() {
    /*
     * Sanity checks for W5500 and cable connection.
     */
    Serial.print("Checking connection.");
    bool rdy_flag = false;
    for (uint8_t i = 0; i <= 20; i++) {
        if ((Ethernet.hardwareStatus() == EthernetNoHardware) || (Ethernet.linkStatus() == LinkOFF)) {
            Serial.print(".");
            rdy_flag = false;
            delay(80);
        } else {
            rdy_flag = true;
            break;
        }
    }
    if (rdy_flag == false) {
        Serial.println("\n\r\tHardware fault, or cable problem... cannot continue.");
        Serial.print("Hardware Status: ");
        prt_hwval(Ethernet.hardwareStatus());
        Serial.print("   Cable Status: ");
        prt_ethval(Ethernet.linkStatus());
        for (uint8_t i = 0; i < 50; i++) {
            delay(100);          // Halt.
        }
        Serial.println("Resetting");
        ESP.restart();          // Try reboot
    } else {
        Serial.println(" OK");
    }
}

void setupSignalLights() {
  pinMode(GPIO_GREEN, OUTPUT);
  pinMode(GPIO_YELLOW, OUTPUT);
  pinMode(GPIO_RED, OUTPUT);
  pinMode(GPIO_BUZZER, OUTPUT);
}

void blink(int gpio_pin, int duration=300, int pause=150) {
  digitalWrite(gpio_pin, HIGH);
  delay(duration);
  digitalWrite(gpio_pin, LOW);
  delay(pause);     
}

void testSignalLights() {
  blink(GPIO_GREEN);   
  blink(GPIO_YELLOW);   
  blink(GPIO_RED);   
  blink(GPIO_BUZZER);   
}

/** MODBUS SERVER **/

// Create server
ModbusServerEthernet MBserver;

uint16_t memo[32];                     // Test server memory: 32 words

// Server function to handle FC 0x03 and 0x04
ModbusMessage FC03(ModbusMessage request)
{
  Serial.println(request);
  ModbusMessage response; // The Modbus message we are going to give back
  uint16_t addr = 0;      // Start address
  uint16_t words = 0;     // # of words requested
  request.get(2, addr);   // read address from request
  request.get(4, words);  // read # of words from request

  // Address overflow?
  if ((addr + words) > 20) {
    // Yes - send respective error response
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  // Set up response
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
  // Request for FC 0x03?
  if (request.getFunctionCode() == READ_HOLD_REGISTER) {
    // Yes. Complete response
    for (uint8_t i = 0; i < words; ++i) {
      // send increasing data values
      response.add((uint16_t)(memo[addr + i]));
    }
  } else {
    // No, this is for FC 0x04. Response is random
    for (uint8_t i = 0; i < words; ++i) {
      // send increasing data values
      response.add((uint16_t)random(1, 65535));
    }
  }
  // Send response back
  return response;
}

void setupAsyncModbusServer() {

    Serial.println("Setting up ModbusTCP server ...");

    // Set up test memory
    for (uint16_t i = 0; i < 32; ++i) {
        memo[i] = 100 + i;
    }
    
    // Now set up the server for some function codes
    // MBserver.registerWorker(1, READ_HOLD_REGISTER, &FC03);      // FC=03 for serverID=1
    // MBserver.registerWorker(2, READ_HOLD_REGISTER, &FC03);      // FC=03 for serverID=2
    MBserver.registerWorker(1, READ_INPUT_REGISTER, &FC03);     // FC=04 for serverID=1
    
    
    // Start the server
    MBserver.start(502, 2, 20000);
}

void modbusLoop() {
    static uint32_t lastMillis = 0;

    if (millis() - lastMillis > 5000) {
        lastMillis = millis();
        Serial.printf("Millis: %10d - free heap: %d\n", lastMillis, ESP.getFreeHeap());
    }
}

/** MODBUS SERVER END **/

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\tNorvi IIoT Signal Lights\r\n");

    // Use Ethernet.init(pin) to configure the CS pin.
    Ethernet.init(5);           // GPIO5 on the ESP32.
    WizReset();

    /* 
     * Network configuration - all except the MAC are optional.
     *
     * IMPORTANT NOTE - The mass-produced W5500 boards do -not-
     *                  have a built-in MAC address (depite 
     *                  comments to the contrary elsewhere). You
     *                  -must- supply a MAC address here.
     */
    Serial.println("Starting ETHERNET connection...");
    Ethernet.begin(eth_MAC, eth_IP, eth_DNS, eth_GW, eth_MASK);

    delay(200);

    Serial.print("Ethernet IP is: ");
    Serial.println(Ethernet.localIP());

    checkConnection();


    setupAsyncModbusServer();


    setupSignalLights();
    testSignalLights();

    Serial.println("Setup done.");
}

void loop() {
    modbusLoop();
    checkConnection();
}