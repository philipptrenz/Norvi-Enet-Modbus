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

void log(String text);
void updateDisplayInfo(String text);

/*
 * Wiz W5500 reset function.  Change this for the specific reset
 * sequence required for your particular board or module.
 */
void WizReset() {
    Serial.print("Resetting W5500 Ethernet ...");

    pinMode(RESET_P, OUTPUT);
    digitalWrite(RESET_P, HIGH);
    delay(250);
    digitalWrite(RESET_P, LOW);
    delay(50);
    digitalWrite(RESET_P, HIGH);
    delay(350);

    Serial.println(" Done.");
}

bool isW5500Found() {
    return Ethernet.hardwareStatus() == EthernetW5500;
}

String hardwareStatusString() {
    switch (Ethernet.hardwareStatus()) {
        case EthernetW5100: return "W5100";
		case EthernetW5200: return "W5200";
        case EthernetW5500: return "W5500";
		default: return "NOT FOUND";
    }
}

bool isLinkUp() {
    return Ethernet.linkStatus() == LinkON;
}

String linkStatusString() {
    switch (Ethernet.linkStatus()) {
        case LinkON: return "UP";
        case LinkOFF: return "DOWN";
        default: return "UNKNOWN";
    }
}

bool isIPValid() {
    return Ethernet.localIP() != IPAddress(0,0,0,0);
}

bool checkConnection() {
    /*
     * Sanity checks for W5500 and cable connection.
     */

    Serial.print("Checking network connection ...");

    bool rdy_flag = false;
    for (uint8_t i = 0; i <= 20; i++) {
        if (Ethernet.hardwareStatus() == EthernetNoHardware || Ethernet.linkStatus() == LinkOFF) {
            Serial.print(".");
            rdy_flag = false;
            delay(80);
        } else {
            rdy_flag = true;
            break;
        }
    }
    if (!rdy_flag) {

        Serial.println("\n\rHardware fault, or cable issue:");
        Serial.println("\tHardware: " + hardwareStatusString());
        Serial.println("\tLink: " + linkStatusString());
        Serial.println("\tIP: " + Ethernet.localIP().toString());
        for (uint8_t i = 0; i < 50; i++) {
            delay(100);          // Halt.
        }

        Serial.println("Could not connect to network.");
    } else {
        Serial.println(" Ok.");
    }

    return rdy_flag;
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


void checkConnectionLoop() {
    static uint32_t lastMillis = 0;
    static uint32_t lastResetMillis = 0;
    static bool wasOffline = false;

    bool isOnline = isW5500Found() && isLinkUp() && isIPValid();
    uint32_t lastUpdatedSeconds = millis() - lastMillis;

    if (!isOnline && !wasOffline) {
        Serial.println("Lost Ethernet connection.");
        lastMillis = millis();

        wasOffline = true;
    } else if (!isOnline && wasOffline && lastUpdatedSeconds > 5 * 1000) {
        
        uint32_t lastResetSeconds =  millis() - lastResetMillis;
        if (lastUpdatedSeconds < 30 * 1000 && lastResetSeconds > 5 * 1000) { 
            // If shorter than 30 seconds in timeout and at least 5 seconds since last reset
            Serial.println("Network still lost, resetting W5500 Ethernet connection ...");

            reconnect();
            checkConnection();

            lastResetMillis = millis();
        } else if (lastUpdatedSeconds > 30 * 1000) {
            Serial.println("In network timeout state since one minute, restarting ESP ...");
            ESP.restart();
        }
    } else if (isOnline && wasOffline) {
        Serial.println("Network restored, IP is: " + Ethernet.localIP().toString());

        wasOffline = false;
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

void outputPortsLoop() {

    static bool flashingStateSlow = true;
    static bool flashingStateFast = true;

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
        // Serial.printf("Millis: %10d - free heap: %d\n", lastMillis, ESP.getFreeHeap());
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
    display.drawString(0, h - 21, "Link: " + linkStatusString());

    String ip = Ethernet.localIP().toString();
    display.drawString(0, h - 10, "IP: " + ip);

    display.display();
}

/** DISPLAY END **/


/** ARDUINO OTA SETUP **/

void setupArduinoOTA() {

    ArduinoOTA.begin();
    
    ArduinoOTA.onStart([]() {
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        display.drawString(display.getWidth() / 2, display.getHeight() / 2 - 10, "OTA Update");
        display.display();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        display.drawProgressBar(4, 32, 120, 8, progress / (total / 100) );
        display.display();
    });

    ArduinoOTA.onEnd([]() {
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Restart");
        display.display();
    });
}

/** ARDUINO OTA SETUP END **/

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

    /*
    log("Setting up Arduino OTA ...");
    setupArduinoOTA();
    */

    log("Testing output ports ...");
    testOutputPorts();

    log("Ready.");
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
        updateDisplayInfo("Ready.");
        displayUpdateMillis = millis();
    }

    /*
    ArduinoOTA.handle();
    */

}