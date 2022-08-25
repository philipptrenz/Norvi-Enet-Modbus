/*
 * Norvi ENET Signal Lights
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
#include <ArduinoOTA.h>

#include <Wire.h>
#include "SSD1306Wire.h"

#include "local_config.h"   // <--- Change settings for YOUR network here.


// Modbus server TCP
#include "ModbusServerEthernet.h"


enum RegisterStatus: uint16_t { 
    OFF = 0, 
    ON = 1, 
    FLASHING_SLOW = 2, 
    FLASHING_FAST = 3 
};

const uint8_t gpioOutPorts[] = {
    2,      // T.0
    4,      // T.1
    12,     // T.2
    13,     // T.3
};
const uint8_t numHoldingRegisters = 4;
RegisterStatus holdingRegisterStates[] = {
    OFF,     // T.0
    OFF,     // T.1
    OFF,     // T.2
    OFF      // T.3
};

const uint8_t gpioInPorts[] = {
    21,      // I.0
    14,      // I.1
    33,      // I.2
    34,      // I.3
    35,      // I.4
    25,      // I.5
    32,      // I.6
    22       // I.7
};
const uint8_t numInputRegisters = 8;
RegisterStatus inputRegisterStates[] = {
    OFF,     // I.0
    OFF,     // I.1
    OFF,     // I.2
    OFF,     // I.3
    OFF,     // I.4
    OFF,     // I.5
    OFF,     // I.6
    OFF      // I.7
};

const int gpioButton = 36;
const uint8_t numButtons = 3;
RegisterStatus buttonStates[] = {
    OFF,      // S1
    OFF,      // S2
    OFF,      // S3
};

/* OUTPUT PORTS ERROR AND DEFAULT STATE CONFIG */

enum DeviceStatus: uint16_t { 
    DEVICE_OK = 10, 
    ETH_DISCONNECTED = 11, 
    ETH_CONNECTING = 12, 
    DEVICE_ERROR = 13 
};


// SSD1306 display configuration (128x64):
SSD1306Wire display(0x3c, 16, 17);  // ADDRESS, SDA, SCL

void flash(int gpio_pin, int duration=300, int pause=150) {
    digitalWrite(gpio_pin, HIGH);
    delay(duration);
    digitalWrite(gpio_pin, LOW);
    delay(pause);     
}

void WizReset();
String prt_hwval(uint8_t refval);
String prt_ethval(uint8_t refval);


void log(String text);
void updateDisplayInfo(String text);

/*
 * Wiz W5500 reset function.  Change this for the specific reset
 * sequence required for your particular board or module.
 */
void WizReset() {
    Serial.println("Resetting W5500 Ethernet ...");
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
String prt_hwval(uint8_t refval) {
    switch (refval) {
    case 0:
        return "No hardware detected.";
    case 1:
        return "WizNet W5100 detected.";
    case 2:
        return "WizNet W5200 detected.";
    case 3:
        return "WizNet W5500 detected.";
    default:
        return "UNKNOWN";
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
String prt_ethval(uint8_t refval) {
    switch (refval) {
    case 0:
        return "UNKNOWN";
    case 1:
        return "UP";
    case 2:
        return "DOWN";
    default:
        return "UNDEFINED";
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

        Serial.println("\n\r\tHardware fault, or cable issue, cannot continue:");
        Serial.println("  Hardware Status: " + prt_hwval(Ethernet.hardwareStatus()));
        Serial.println("     Cable Status: " + prt_ethval(Ethernet.linkStatus()));
        for (uint8_t i = 0; i < 50; i++) {
            delay(100);          // Halt.
        }

        Serial.println("Could not connect to Ethernet.");
    } else {
        Serial.println("Ethernet is ok.");
    }
}

void reconnect() {

    log("Starting Ethernet connection ...");

    WizReset();

    /* 
     * Network configuration - all except the MAC are optional.
     *
     * IMPORTANT NOTE - The mass-produced W5500 boards do -not-
     *                  have a built-in MAC address (despite 
     *                  comments to the contrary elsewhere). You
     *                  -must- supply a MAC address here.
     */
    
    Ethernet.begin(eth_MAC, eth_IP, eth_DNS, eth_GW, eth_MASK);

    delay(200);
    Serial.println("Ethernet IP is: " + Ethernet.localIP().toString());
}


bool wasOffline = false;

void checkConnectionLoop() {
    static uint32_t lastMillis = 0;

    bool isOffline = (Ethernet.hardwareStatus() == EthernetNoHardware) || (Ethernet.linkStatus() == LinkOFF);
    bool isIPvalid = Ethernet.localIP() != IPAddress(0,0,0,0);
    bool isInTimeout = wasOffline && millis() - lastMillis > 5 * 1000;     // greater than 5 seconds
    // Red FLASHING_SLOW when connection is lost
    if (isOffline && !isInTimeout) {
        if (!wasOffline) {
            Serial.println("Lost network connection");
            lastMillis = millis();
        }

        wasOffline = true;
    } else if (wasOffline && isIPvalid) {
        Serial.print("Network restored, Ethernet IP is: ");
        Serial.println(Ethernet.localIP());

        wasOffline = false;
        lastMillis = 0;
    } else if (isInTimeout) {
        if (millis() - lastMillis <= 30 * 1000) {   // less than 30 seconds
            // TODO: Wait 5 seconds between reconnect approaches
            Serial.println("Network still lost, resetting W5500 Ethernet connection ...");
            reconnect();
            
        } else {
            Serial.println("In network timeout state since one minute, restarting ESP ...");
            ESP.restart();
        }
    }
}

void setupOutputPorts() {
    Serial.println("Setting up output port ...");
    for (int i = 0; i < numHoldingRegisters; i++) {
        pinMode(gpioOutPorts[i], OUTPUT);
    }
}

void testOutputPorts() {
    for (uint8_t i = 0; i < numHoldingRegisters; i++) {
        flash(gpioOutPorts[i]);
    }
}

bool flashingStateSlow = true;
bool flashingStateFast = true;

void outputPortsLoop() {

    static uint32_t lastMillisSlow = 0;
    if (millis() - lastMillisSlow > 1200) {
        flashingStateSlow = !flashingStateSlow;
        lastMillisSlow = millis();
    }

    static uint32_t lastMillisFast = 0;
    if (millis() - lastMillisFast > 300) {
        flashingStateFast = !flashingStateFast;
        lastMillisFast = millis();
    }    

    for (uint8_t i = 0; i < numHoldingRegisters; i++) {

        switch (holdingRegisterStates[i]) {
            case OFF:
                digitalWrite(gpioOutPorts[i], LOW);
                break;
            case ON:
                digitalWrite(gpioOutPorts[i], HIGH);
                break;
            case FLASHING_SLOW:
                digitalWrite(gpioOutPorts[i], flashingStateSlow ? HIGH : LOW);
                break;
            case FLASHING_FAST:
                digitalWrite(gpioOutPorts[i], flashingStateFast ? HIGH : LOW);
                break;
        }
    }
}

void setupInputPorts() {
    for (uint8_t i = 0; i < numInputRegisters; i++) {
        pinMode(gpioInPorts[i], INPUT);
    }
}

void inputPortsLoop() {
    for (uint8_t i = 0; i < numInputRegisters; i++) {
        switch (digitalRead(gpioInPorts[i])) {
            case HIGH:
                inputRegisterStates[i] = OFF;
                break;
            case LOW:
                inputRegisterStates[i] = ON;
                break;
        }
    }
}

void setupButtonPort() {
    pinMode(gpioButton, INPUT);
}

uint32_t lastAllButtonsOffMillis = 0;
void buttonReadingLoop() {
    /*
     * Norvi ENET built-in buttons all are connected to GPIO 36 
     * using different resistor values. Therefore have to be decoded
     * using ADC; ADC values are 12bit (0...4095)
     */
    uint16_t newVal = (uint16_t) analogRead(gpioButton);

    if (newVal > 1300) {

        // Serial.print("Button analog reading: ");
        // Serial.println(newVal);

        if (newVal < 2000) {
            buttonStates[0] = ON;
            buttonStates[1] = OFF;
            buttonStates[2] = OFF;
            // Serial.println("Button S1 pressed");

        } else if (newVal < 2450) {
            buttonStates[0] = OFF;
            buttonStates[1] = ON;
            buttonStates[2] = OFF;
            // Serial.println("Button S2 pressed");

        } else if (newVal < 2950) {
            buttonStates[0] = ON;
            buttonStates[1] = ON;
            buttonStates[2] = OFF;
            // Serial.println("Button S1 and S2 pressed");

        } else if (newVal < 3280) {
            buttonStates[0] = OFF;
            buttonStates[1] = OFF;
            buttonStates[2] = ON;
            // Serial.println("Button S3 pressed");

        } else if (newVal < 3380) {
            buttonStates[0] = ON;
            buttonStates[1] = OFF;
            buttonStates[2] = ON;
            // Serial.println("Button S1 and S3 pressed");

        } else if (newVal < 3460) {
            buttonStates[0] = OFF;
            buttonStates[1] = ON;
            buttonStates[2] = ON;
            // Serial.println("Button S2 and S3 pressed");

        } else if (newVal < 3800) {
            buttonStates[0] = ON;
            buttonStates[1] = ON;
            buttonStates[2] = ON;
            // Serial.println("Button S1 and S2 and S3 pressed");

            if (lastAllButtonsOffMillis > 0 && millis() - lastAllButtonsOffMillis > 3*1000) {
                // If all three buttons are pressed simultaneously for at least three seconds
                Serial.println("Restart button sequence pressed, restarting ESP ...");
                ESP.restart();      // Restart ESP32
            }

        }


    } else {
        buttonStates[0] = OFF;
        buttonStates[1] = OFF;
        buttonStates[2] = OFF;

        lastAllButtonsOffMillis = millis();
    }
}

/** MODBUS SERVER **/

// Create server
ModbusServerEthernet MBserver;

ModbusMessage HANDLE_READ_HOLD_REGISTER(ModbusMessage request) {
    // Serial.println(request);
    ModbusMessage response; // The Modbus message we are going to give back
    uint16_t addr = 0;      // Start address
    uint16_t words = 0;     // # of words requested
    request.get(2, addr);   // read address from request
    request.get(4, words);  // read # of words from request

    // Address overflow?
    if ((addr + words) > numHoldingRegisters) {
        // Yes - send respective error response
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    }

    // Set up response
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

    if (request.getFunctionCode() == READ_HOLD_REGISTER) {
        for (uint8_t i = 0; i < words; ++i) {
            response.add(
                (uint16_t)(holdingRegisterStates[addr+i])
            );
        }
    }

    return response;
}

ModbusMessage HANDLE_WRITE_HOLD_REGISTER(ModbusMessage request) {
    // Serial.println(request);
    ModbusMessage response; // The Modbus message we are going to give back
    uint16_t addr = 0;      // Start address
    uint16_t value = 0;     // Register value
    request.get(2, addr);   // read address from request
    request.get(4, value);  // read received value

    // Address overflow?
    if (addr > numHoldingRegisters) {
        // Yes - send respective error response
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    }

    // Set up response
    response.add(request.getServerID(), request.getFunctionCode(), (uint16_t)(addr));

    if (request.getFunctionCode() == WRITE_HOLD_REGISTER) {

        if (value >= 0 && value <= 3) {

            RegisterStatus newValue = static_cast<RegisterStatus>(value);

            Serial.print("WRITE_HOLD_REGISTER: Changing holding register ");
            Serial.print(addr);
            Serial.print(" from ");
            Serial.print(holdingRegisterStates[addr]);
            Serial.print(" to ");
            Serial.println(newValue);

            holdingRegisterStates[addr] = newValue;
        }

        response.add(
            (uint16_t)(holdingRegisterStates[addr])
        );
    }

    return response;
}

ModbusMessage HANDLE_WRITE_MULT_REGISTERS(ModbusMessage request) {
    // Serial.println(request);
    ModbusMessage response; // The Modbus message we are going to give back
    uint16_t addr = 0;      // Start address
    uint16_t quantity = 0;     // Quantity of registers
    uint16_t byteCount = 0;     // Byte count
    request.get(2, addr);
    request.get(4, quantity);
    request.get(5, byteCount);


    // Address overflow?
    if ((addr+quantity) > numHoldingRegisters) {
        // Yes - send respective error response
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    }

    // Set up response
    response.add(request.getServerID(), request.getFunctionCode(), (uint16_t)(addr), (uint16_t)(quantity));

    if (request.getFunctionCode() == WRITE_MULT_REGISTERS) {

        for (uint8_t i = 0; i < quantity; i++) {

            uint16_t value = request.get(7+(2*i));

            if (value >= 0 && value <= 3) {

                RegisterStatus newValue = static_cast<RegisterStatus>(value);

                Serial.print("WRITE_MULT_REGISTERS: Changing holding register ");
                Serial.print(addr);
                Serial.print(" from ");
                Serial.print(holdingRegisterStates[addr]);
                Serial.print(" to ");
                Serial.println(newValue);

                holdingRegisterStates[addr] = newValue;
            }
        }
    }

    return response;
}

ModbusMessage HANDLE_READ_INPUT_REGISTER(ModbusMessage request) {
    // Serial.println(request);
    ModbusMessage response; // The Modbus message we are going to give back
    uint16_t addr = 0;      // Start address
    uint16_t words = 0;     // # of words requested
    request.get(2, addr);   // read address from request
    request.get(4, words);  // read # of words from request

    // Address overflow?
    if ((addr + words) > (numInputRegisters + numButtons)) {
        // Yes - send respective error response
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    }

    // Set up response
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

    if (request.getFunctionCode() == READ_INPUT_REGISTER) {
        for (uint8_t i = 0; i < words; ++i) {
            uint8_t registerId = addr+i;
            if (registerId < numInputRegisters) {
                // Return inputRegisterState at registerId
                response.add(
                    (uint16_t)(inputRegisterStates[registerId])
                );
            } else {
                // Return buttonState
                uint8_t buttonId = registerId - numInputRegisters;
                response.add(
                    (uint16_t)(buttonStates[buttonId])
                );
            }
        }
    }

    return response;
}

void setupModbusServer() {
    
    // Holding registers for output ports
    MBserver.registerWorker(1, READ_HOLD_REGISTER, &HANDLE_READ_HOLD_REGISTER);          // FC=03 for serverID=1
    MBserver.registerWorker(1, WRITE_HOLD_REGISTER, &HANDLE_WRITE_HOLD_REGISTER);        // FC=03 for serverID=1
    MBserver.registerWorker(1, WRITE_MULT_REGISTERS, &HANDLE_WRITE_MULT_REGISTERS);      // FC=03 for serverID=1
    
    // Input registers for input ports
    MBserver.registerWorker(1, READ_INPUT_REGISTER, &HANDLE_READ_INPUT_REGISTER);        // FC=04 for serverID=1
    
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


/** DISPLAY **/

void setupDisplay() {

    display.init();
    display.flipScreenVertically();
    display.setContrast(255);
    display.setFont(ArialMT_Plain_10);

    uint16_t w = display.getWidth();
    uint16_t h = display.getHeight();

    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    // display.drawString(w / 2, 6, "NORVI ENET ModbusTCP");
    display.drawString(w / 2, 6, "casayohana");
    display.drawLine(0, 13, w, 13);

    display.display();
}

void log(String text) {
    Serial.println(text);
    updateDisplayInfo(text);
}

void updateDisplayInfo(String text) {

    uint16_t w = display.getWidth();
    uint16_t h = display.getHeight();

    display.clear();

    // HEADER
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    // display.drawString(w / 2, 6, "NORVI ENET ModbusTCP");
    display.drawString(w / 2, 6, "casayohana");
    display.drawLine(0, 13, w, 13);

    // LOGGING TEXT
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    if (display.getStringWidth(text) > w) {
        display.drawStringMaxWidth(0, 14, w, text);
    } else {
        display.drawStringMaxWidth(0, 14 + 6, w, text);
    }
    

    // FOOTER
    display.drawLine(0, h - 22, w, h - 22);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    String linkStatus = prt_ethval(Ethernet.linkStatus());
    display.drawString(0, h - 21, "Link: " + linkStatus);

    String ip = Ethernet.localIP().toString();
    display.drawString(0, h - 10, "IP: " + ip);

    display.display();
}

/** DISPLAY END **/


void setup() {
    Serial.begin(115200);
    delay(500);

    setupDisplay();

    Serial.println("\n\tNorvi ENET ModbusTCP\r\n");

    log("Setting up IO ports ...");
    setupOutputPorts();
    setupInputPorts();
    setupButtonPort();

    log("Setting up Ethernet ...");
    // Use Ethernet.init(pin) to configure the CS pin.
    Ethernet.init(26);           // GPIO26 on the ESP32.

    reconnect();
    checkConnection();

    log("Starting Modbus TCP server ...");
    setupModbusServer();

    log("Testing output ports ...");
    testOutputPorts();

    log("Device ready.");
}

uint32_t displayUpdateMillis = millis();

void loop() {
    buttonReadingLoop();
    checkConnectionLoop();
    outputPortsLoop();
    inputPortsLoop();
    modbusLoop();

    // Update display every 500ms
    if (millis() - displayUpdateMillis > 500) {
        updateDisplayInfo("Device ready.");
        displayUpdateMillis = millis();
    }
}