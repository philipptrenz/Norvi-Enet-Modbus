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

#include "local_config.h"   // <--- Change settings for YOUR network here.


// Modbus server TCP
#include "ModbusServerEthernet.h"


enum RegisterStatus: uint16_t { 
    OFF = 0, 
    ON = 1, 
    BLINKING_SLOW = 2, 
    BLINKING_FAST = 3 
};

const uint8_t gpioOutPorts[] = {
    2,      // T.0, GREEN
    4,      // T.1, YELLOW
    12,     // T.2, RED
    13,     // T.3, BUZZER
};
const uint8_t numHoldingRegisters = 4;
RegisterStatus holdingRegisterStates[] = {
    OFF,     // T.0, GREEN
    OFF,     // T.1, YELLOW
    OFF,     // T.2, RED
    OFF      // T.3, BUZZER
};

const uint8_t gpioInPorts[] = {
    34,      // I.0
    33,      // I.1
    14,      // I.2
    21,      // I.3
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
        Serial.println("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
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
        Serial.println("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
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
        Serial.println("Resetting ESP ...");
        ESP.restart();          // Try reboot
    } else {
        Serial.println(" OK");
    }
}


bool wasOffline = false;
void checkConnectionLoop() {
    static uint32_t lastMillis = 0;

    bool isOffline = (Ethernet.hardwareStatus() == EthernetNoHardware) || (Ethernet.linkStatus() == LinkOFF);
    bool isIPvalid = Ethernet.localIP() != IPAddress(0,0,0,0);
    bool isInTimeout = wasOffline && millis() - lastMillis > 10 * 1000;     // greater than 10 seconds
    // Red BLINKING_SLOW when connection is lost
    if (isOffline && !isInTimeout) {
        if (!wasOffline) {
            Serial.println("Lost network connection");
            lastMillis = millis();
        }

        holdingRegisterStates[0] = OFF;             // GREEN
        holdingRegisterStates[1] = OFF;             // YELLOW
        holdingRegisterStates[2] = BLINKING_SLOW;   // RED
        holdingRegisterStates[3] = OFF;             // BUZZER

        wasOffline = true;
    } else if (wasOffline && isIPvalid) {
        Serial.print("Network restored, Ethernet IP is: ");
        Serial.println(Ethernet.localIP());

        holdingRegisterStates[0] = OFF;  // GREEN
        holdingRegisterStates[1] = OFF;  // YELLOW
        holdingRegisterStates[2] = OFF;  // RED
        holdingRegisterStates[3] = OFF;  // BUZZER

        wasOffline = false;
        lastMillis = 0;
    } else if (isInTimeout) {
        if (millis() - lastMillis <= 60 * 1000) {   // less than one minute
            Serial.println("Network still lost, resetting W5500 ethernet connection ...");
            WizReset();
            Ethernet.begin(eth_MAC, eth_IP, eth_DNS, eth_GW, eth_MASK);
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

void blink(int gpio_pin, int duration=300, int pause=150) {
    digitalWrite(gpio_pin, HIGH);
    delay(duration);
    digitalWrite(gpio_pin, LOW);
    delay(pause);     
}

void testOutputPorts() {
    for (uint8_t i = 0; i < numHoldingRegisters; i++) {
        blink(gpioOutPorts[i]);
    }
}

bool blinkingStateSlow = true;
bool blinkingStateFast = true;

void outputPortsLoop() {

    static uint32_t lastMillisSlow = 0;
    if (millis() - lastMillisSlow > 1200) {
        blinkingStateSlow = !blinkingStateSlow;
        lastMillisSlow = millis();
    }

    static uint32_t lastMillisFast = 0;
    if (millis() - lastMillisFast > 300) {
        blinkingStateFast = !blinkingStateFast;
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
            case BLINKING_SLOW:
                digitalWrite(gpioOutPorts[i], blinkingStateSlow ? HIGH : LOW);
                break;
            case BLINKING_FAST:
                digitalWrite(gpioOutPorts[i], blinkingStateFast ? HIGH : LOW);
                break;
        }
    }
}

void setupInputPorts() {
    Serial.println("Setting up input ports ...");
    for (uint8_t i = 0; i < numInputRegisters; i++) {
        pinMode(gpioInPorts[i], INPUT);
    }
}

void inputPortsLoop() {
    for (uint8_t i = 0; i < numInputRegisters; i++) {
        switch (digitalRead(gpioInPorts[i])) {
            case LOW:
                inputRegisterStates[i] = OFF;
                break;
            case HIGH:
                inputRegisterStates[i] = ON;
                break;
        }
    }
}

void setupButtonPort() {
    Serial.println("Setting up button port ...");
    pinMode(gpioButton, INPUT);
}

void buttonReadingLoop() {
    /*
     * Norvi ENET built-in buttons all are connected to GPIO 36 
     * using different resistor values. Therefore have to be decoded
     * using ADC; ADC values are 12bit (0...4095)
     */
    uint16_t newVal = (uint16_t) analogRead(gpioButton);

    // TODO: Decode buttons
    // if (newVal >= 3290 && newVal <= 3310) {
    // } else if (newVal >= 3290 && newVal <= 3310) {
    // } else if (newVal >= 3290 && newVal <= 3310) {

    if (newVal >= 300) {

        Serial.print("Button analog reading: ");
        Serial.println(newVal);

        ESP.restart();      // Restart ESP32

    } else if (newVal < 300) {
        buttonStates[0] = OFF;
        buttonStates[1] = OFF;
        buttonStates[2] = OFF;
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
    if ((addr + words) > numInputRegisters) {
        // Yes - send respective error response
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    }

    // Set up response
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

    if (request.getFunctionCode() == READ_INPUT_REGISTER) {
        for (uint8_t i = 0; i < words; ++i) {
            response.add(
                (uint16_t)(inputRegisterStates[addr+i])
            );
        }
    }

    return response;
}

void setupModbusServer() {

    Serial.println("Setting up ModbusTCP server ...");
    
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

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\tNorvi ENET Signal Lights\r\n");

    // Use Ethernet.init(pin) to configure the CS pin.
    Ethernet.init(5);           // GPIO5 on the ESP32.
    WizReset();

    /* 
     * Network configuration - all except the MAC are optional.
     *
     * IMPORTANT NOTE - The mass-produced W5500 boards do -not-
     *                  have a built-in MAC address (despite 
     *                  comments to the contrary elsewhere). You
     *                  -must- supply a MAC address here.
     */
    Serial.println("Starting Ethernet connection ...");
    Ethernet.begin(eth_MAC, eth_IP, eth_DNS, eth_GW, eth_MASK);

    delay(200);

    Serial.print("Ethernet IP is: ");
    Serial.println(Ethernet.localIP());

    checkConnection();

    setupModbusServer();

    setupOutputPorts();
    testOutputPorts();

    setupInputPorts();

    setupButtonPort();

    Serial.println("Setup done.");
}

void loop() {
    buttonReadingLoop();
    checkConnectionLoop();
    outputPortsLoop();
    inputPortsLoop();
    modbusLoop();
}