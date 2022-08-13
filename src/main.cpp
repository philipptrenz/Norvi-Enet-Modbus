/*
 *   $Id: ESP32_NTP.ino,v 1.8 2019/04/04 04:48:23 gaijin Exp $
 *
 *  UDP NTP client example program.
 * 
 *  Get the time from a Network Time Protocol (NTP) time server
 *  Demonstrates use of UDP sendPacket and ReceivePacket
 * 
 *  Created:  04 Sep 2010 by Michael Margolis
 *  Modified: 09 Apr 2012 by Tom Igoe
 *  Modified: 02 Sep 2015 by Arturo Guadalupi
 *  Munged:   04 Apr 2019 by PuceBaboon (for the ESP32 with a W5500 module)
 * 
 */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>
#include "local_config.h"   // <--- Change settings for YOUR network here.


/* 
 * GPIO config for signal lights
 */ 
const int GPIO_GREEN = 2;
const int GPIO_YELLOW = 4;
const int GPIO_RED = 12;
const int GPIO_BUZZER = 13;




/* 
 * ModbusTCP setup
 */ 
EthernetServer EthServer(502);
EthernetClient clients[4];
ModbusTCPServer ModbusTCPServer;
uint16_t adc0;
uint16_t adc0_result;
uint16_t test;


void WizReset();
void prt_hwval(uint8_t refval);
void prt_ethval(uint8_t refval);
void sendNTPpacket(const char *address);

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
        Serial.println
            ("\n\r\tHardware fault, or cable problem... cannot continue.");
        Serial.print("Hardware Status: ");
        prt_hwval(Ethernet.hardwareStatus());
        Serial.print("   Cable Status: ");
        prt_ethval(Ethernet.linkStatus());
        while (true) {
            delay(10);          // Halt.
        }
    } else {
        Serial.println(" OK");
    }
}

void setupModbus() {
    Serial.println("Setting up ModbusTCP ...");

    EthServer.begin();
    ModbusTCPServer.configureHoldingRegisters(0x00, 100);
    ModbusTCPServer.begin();

    test = 110;
}

void updateREG()  {
  //adc0 = ads.readADC_SingleEnded(1);
  ModbusTCPServer.holdingRegisterWrite(0x00, test);
  adc0_result = ModbusTCPServer.holdingRegisterRead(0x00);
  Serial.print("AIN0: "); 
  Serial.println(adc0_result);
  Serial.println(" ");
}

void handleModbus() {
    // Modbus server accept incoming connections
    EthernetClient client = EthServer.available();
    if (client) {
        Serial.println("New client");
        ModbusTCPServer.accept(client);
        while (client.connected()) {
            ModbusTCPServer.poll();
            updateREG();      
        }
    }
}



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

    setupModbus();


    Serial.println("Setup done.");
}

void loop() {
    handleModbus();
}