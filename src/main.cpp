//#define DISABLE_THREADS
//#define TEST_SCANKEYS

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

#include<cmath>  
#include <map>
#include <algorithm>

//Configuration Options
    // Global variables for configuration
    const bool isSender = true;  // Default to sender mode
    const int octaveNumber = 4;  // Default octave number
    
//Constants
    // Mute setting
    bool mute = false;

    // Display update interval
    const uint32_t interval = 100; 

    // Notes Constant
    const char notes [12][3] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

    // StepSizes Constant
    const float base_freq = 440; // Base Frequency of note A
    const float base_phase = base_freq * pow(2, 32) * 1/22000;
    const float semitone_ratio = 1.0594630943592952645618252949463; // Equal temperament

    const uint32_t stepSizes[12] = {
        u_int32_t(base_phase * pow(semitone_ratio, -9)), // C
        u_int32_t(base_phase * pow(semitone_ratio, -8)), // C#
        u_int32_t(base_phase * pow(semitone_ratio, -7)), // D
        u_int32_t(base_phase * pow(semitone_ratio, -6)), // D#
        u_int32_t(base_phase * pow(semitone_ratio, -5)), // E
        u_int32_t(base_phase * pow(semitone_ratio, -4)), // F
        u_int32_t(base_phase * pow(semitone_ratio, -3)), // F#
        u_int32_t(base_phase * pow(semitone_ratio, -2)), // G
        u_int32_t(base_phase * pow(semitone_ratio, -1)), // G#
        u_int32_t(base_phase),                           // A
        u_int32_t(base_phase * semitone_ratio),          // A#
        u_int32_t(base_phase * pow(semitone_ratio, 2))   // B
    };

//Pin definitions
    //Row select and enable
    const int RA0_PIN = D3;
    const int RA1_PIN = D6;
    const int RA2_PIN = D12;
    const int REN_PIN = A5;

    //Matrix input and output
    const int C0_PIN = A2;
    const int C1_PIN = D9;
    const int C2_PIN = A6;
    const int C3_PIN = D1;
    const int OUT_PIN = D11;

    //Audio analogue out
    const int OUTL_PIN = A4;
    const int OUTR_PIN = A3;

    //Joystick analogue in
    const int JOYY_PIN = A0;
    const int JOYX_PIN = A1;

    //Output multiplexer bits
    const int DEN_BIT = 3;
    const int DRST_BIT = 4;
    const int HKOW_BIT = 5;
    const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//--------------------------------------------------------------------------------------//
//                            Classes, Variables and Structs                            //
//--------------------------------------------------------------------------------------//

// Define the global struct to store system state.
// This is protected by a mutex as it is accessed by multiple threads.
struct {
    int volume = 8;
    std::bitset<32> inputs;
    SemaphoreHandle_t mutex;  // A mutex is ‘taken’ (locked) by a thread while it accesses the protected data or resource,
} sysState;                   // then ‘given’ (unlocked) when it is finished.

// Knob Class
class Knob {
    public: 
        std::bitset<3> prevKnob; // Meaning of bits from index 2 to 0 is "SBA"
        std::bitset<3> knob;     // where "S" = press state and "BA" = turning state.
        int knobRotation; // Previous turn direction of the knob
        bool *toggleState; // Pointer to the global variable/toggleState you are toggling on/off (e.g mute)

        // Constructor for each knob. Need to initiate what "toggleState" this knob is changing.
        Knob (bool &state) {
            toggleState = &state;
        }

        // Set the bits of the current knob iteration. Note index 2 = left bit, index 0 = right bit.
        void setKnob(int index, bool val) {
            knob[index] = val;
        }

        // Check for knob change between "prevKnov" and "knob"
        void checkKnobChange() {
            uint8_t k = knob.to_ulong() & 0x03;
            uint8_t pk = prevKnob.to_ulong() & 0x03;

            if (prevKnob[2] == 1 && knob[2] == 0) { // Check if the knob has been pressed/toggled
                *toggleState = !(*toggleState);
            } else if ((pk == 1 && k == 0 ) || (pk == 2 && k == 3)) { // Clockwise
                knobRotation = -1; 
                sysState.volume = std::max(0, sysState.volume - 1); 
            } else if ((pk == 0 && k == 1) || (pk == 3 && k == 2)) { // Anti-Clockwise
                knobRotation = 1; 
                sysState.volume = std::min(8, sysState.volume + 1);
            } else if ((pk == 3 && k == 0) || (pk == 0 && k == 3)) { // Missed State, assume same rotation as before
                sysState.volume = (knobRotation == 1) ? std::min(8, sysState.volume + 1) : std::max(0, sysState.volume - 1);
            }
            prevKnob = knob;
        }
};

// StepSizes Variable
// This is protected by atomic updates as it is accessed by multiple threads and interrupts.
volatile uint32_t currentStepSize[36] = {0}; // voltaile = variable that is likely to change at any time

// Queue Handler (In and Out)
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

// Global handler for Transmit Thread
SemaphoreHandle_t CAN_TX_Semaphore;

// Incoming Message Variable
uint32_t ID;
uint8_t RX_Message[8]; 





//--------------------------------------------------------------------------------------//
//                                   Helper Functions                                   //
//--------------------------------------------------------------------------------------//

// Set Row
void setRow(uint8_t rowIdx){
    digitalWrite(REN_PIN,LOW);
    digitalWrite(RA0_PIN, rowIdx & 0x01);
    digitalWrite(RA1_PIN, rowIdx & 0x02);
    digitalWrite(RA2_PIN, rowIdx & 0x04);
    digitalWrite(REN_PIN,HIGH);
}

// Read Cols
std::bitset<4> readCols() {
    std::bitset<4> result;
    result[0] = digitalRead(C0_PIN); // e.g All pins are HIGH(1). Pressing a pin like C0_PIN will set it LOW (0)
    result[1] = digitalRead(C1_PIN); // hence it will show 0111 = E (backwards) on the display
    result[2] = digitalRead(C2_PIN); 
    result[3] = digitalRead(C3_PIN); 
    return result;
}

// Timer Driven:
// Interrupt Service Routine to Sample Wave
void sampleISR() {
    if (!mute) {
        static uint32_t phaseAcc[10] = {0}; // Define the phase accumulator as a static local variable so that its stored between successive calls
        int32_t Vout = 0;
        int8_t phaseIndex = 0;

        // Find the first 10 notes/stepSizes that are pressed in currentStepSize (which is a 36 uint32_t sized 
        // array) and increment them into phaseAcc (which only holds 10 waveforms).
        for (int i = 0; i < sizeof(currentStepSize)/sizeof(uint32_t) && phaseIndex < 10; i++) {
            if (currentStepSize[i] != 0) {
                phaseAcc[phaseIndex] = phaseAcc[phaseIndex] + currentStepSize[i];
                Vout += (phaseAcc[phaseIndex] >> 23) - 256; // Shift each phaseAcc to range -256 to 255 and add them to Vout (which has range -2560 to 2550)
                phaseIndex++;
            }
        }

        // If phaseIndex < 10 then it means less than 10 notes being played. If so set the rest of the
        // waveforms in phaseAcc to 0 amplitude.
        while (phaseIndex < 10) {
            phaseAcc[phaseIndex] = 0;
            Vout -= 256;
            phaseIndex++;
        }

        // Shift Vout back to range (0 to 5110) and adjust it according to the volume set
        // (As noticed overflow will occur if "amplitude > 4095", or in other words more than 8 notes
        // played. To avoid this, we set a cap on the amplitude of 4095.)
        Vout = (Vout + 2560) >> (8 - sysState.volume); 
        Vout = (Vout <= 4095) ? Vout : 4095; 
        
        analogWriteResolution(12);
        analogWrite(OUTR_PIN, Vout); // Output the sawtooth wave (which has range 0 to 4095)
    } 

}

// Event Driven:
// Triggered when a new CAN message arrives at the CAN receiver module of the microcontroller, the 
// CAN hardware raises an interrupt signal.
void CAN_RX_ISR(void) {
    uint8_t RX_Message_ISR[8];
    uint32_t ID;
    CAN_RX(ID, RX_Message_ISR); // Receive message from CAN bus
    xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL); // Places data in the queue
}

// Event Driven:
// Triggered when the CAN module is ready to send a new message (i.e a transmit buffer is free), or 
// after a message has been successfully transmitted, the CAN hardware generates an interrupt.
void CAN_TX_ISR(void) {
    xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL); 
    // Release the semaphore back to CAN_TX_Task.
    // Typically signals or notifies a task that it's possible to send another message.
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
    digitalWrite(REN_PIN,LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);
    digitalWrite(OUT_PIN,value);
    digitalWrite(REN_PIN,HIGH);
    delayMicroseconds(2);
    digitalWrite(REN_PIN,LOW);
}


// Reading Everything
void read(Knob &knob3) {

    uint8_t TX_Message[8] = {0, octaveNumber, 0, 0, 0, 0, 0, 0};

    for (uint8_t row = 0; row <= 7; row++) {
        setRow(row); // Set Row
        delayMicroseconds(3); // Add a small delay
        std::bitset<4> inputCols = readCols(); // Read columns

        for (uint8_t col = 0; col <= 3; col++) {

            // read inputCols into inputs: "inputs[row*4 + col] = inputCols[col]"
            sysState.inputs.set(row*4 + col, inputCols[col]);

            // If a key is pressed (1) i.e row <= 2 and (2) i.e inputCols[col] is low
            if (row <= 2) {
                TX_Message[0] = !inputCols[col] ? 'P' : 'R';
                TX_Message[2] = row*4 + col;  
                xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            }

            // If knob 3 (most right-hand) is changing, update its values into "knob3"
            if (row == 3 && col <= 1) knob3.setKnob(col, inputCols[col]);

            // If knob 3 is pressed (most-right hand), toggle the state of mute
            if (row == 5 && col == 1) knob3.setKnob(2, inputCols[col]);
        }
    }
}

void processMessage(Knob &knob3) { // Function to encapsulate shared logic

    if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE) {
        read(knob3); // Read rows and cols of the inputs
        knob3.checkKnobChange(); // Check for volume change
        xSemaphoreGive(sysState.mutex); // Release the mutex
    }
}





//--------------------------------------------------------------------------------------//
//                                       Thread                                        //
//--------------------------------------------------------------------------------------//

// ---- scanKeysTask ---------------------------------------------------------------------

#ifdef TEST_SCANKEYS
void scanKeysTask() {
    Knob knob3;
    int knob3Rotation;

    for (uint8_t key = 0; key < 12; key++) {
        processMessage(knob3, key);
    }
}
#else
void scanKeysTask(void * pvParameters) {
    Knob knob3(mute);

    const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        processMessage(knob3);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
#endif

// ---- CAN_TX_Task ----------------------------------------------------------------------

void CAN_TX_Task(void *pvParameters) {
    uint8_t msgOut[8];
    while (1) {
        xQueueReceive(msgOutQ, msgOut, portMAX_DELAY); 
        // Waits for a message to be available in the outgoing message queue (msgOutQ)
        // If the queue is empty, it will block/pause the task until a message is posted to the queue.
        // When a message is available, it copies the message into the msgOut array.

        xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
        CAN_TX(0x123, msgOut); // Send message over CAN bus (message ID = 0x123)
    }
}

// ---- CAN_RX_Task ----------------------------------------------------------------------

void CAN_RX_Task(void * pvParameters) {
    while (1) {
        // Wait indefinitely for a message to be available in the queue
        if (xQueueReceive(msgInQ, RX_Message, portMAX_DELAY) == pdTRUE) {
            uint32_t localCurrentStepSize = RX_Message[0] == 'P' ? stepSizes[RX_Message[2]] << (RX_Message[1] - 4) : 0;
            __atomic_store_n(&currentStepSize[RX_Message[2]], localCurrentStepSize, __ATOMIC_RELAXED);
        }
    }
}

// ---- displayUpdateTask ----------------------------------------------------------------

void displayUpdateTask(void * pvParameters) {
    const TickType_t xFrequency = 20 / portTICK_PERIOD_MS; // Initiation interval of the task set to 100ms
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Will store the time (tick count) of the last initiation

    // Poll for received messages
    while (1) {
        if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE) {

            // Update display
            u8g2.clearBuffer(); // clear the internal memory
            u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

            u8g2.drawStr(2,10, "Volume:");  // Write something to the internal memory
            u8g2.setCursor(55,10);
            u8g2.print(sysState.volume);

            u8g2.setCursor(2,20);
            u8g2.print(sysState.inputs.to_ulong(), HEX);

            u8g2.setCursor(2,30);
            /*
            u8g2.print((char) RX_Message[0]);
            u8g2.print(RX_Message[1]);
            RX_Message[2] == 'N' ? u8g2.print((char) RX_Message[2]) : u8g2.print(RX_Message[2]);
            RX_Message[3] == 'N' ? u8g2.print((char) RX_Message[3]) : u8g2.print(RX_Message[3]);
            RX_Message[4] == 'N' ? u8g2.print((char) RX_Message[4]) : u8g2.print(RX_Message[4]);
            RX_Message[5] == 'N' ? u8g2.print((char) RX_Message[5]) : u8g2.print(RX_Message[5]);
            u8g2.print(RX_Message[6]);
            u8g2.print(RX_Message[7]);
            */

            u8g2.setCursor(70,30);

            u8g2.sendBuffer(); // transfer internal memory to the display

            // Toggle LED
            digitalToggle(LED_BUILTIN);

            // Release the mutex after accessing sysState
            xSemaphoreGive(sysState.mutex);
        }

        // Blocks execution of the thread until xFrequency ticks have happened since the last execution of the loop
        vTaskDelayUntil( &xLastWakeTime, xFrequency ); 
    }
}





//--------------------------------------------------------------------------------------//
//                                        Setup                                         //
//--------------------------------------------------------------------------------------//

void setup() { // put your setup code here, to run once:

    //Set pin directions
    pinMode(RA0_PIN, OUTPUT);
    pinMode(RA1_PIN, OUTPUT);
    pinMode(RA2_PIN, OUTPUT);
    pinMode(REN_PIN, OUTPUT);
    pinMode(OUT_PIN, OUTPUT);
    pinMode(OUTL_PIN, OUTPUT);
    pinMode(OUTR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(C0_PIN, INPUT);
    pinMode(C1_PIN, INPUT);
    pinMode(C2_PIN, INPUT);
    pinMode(C3_PIN, INPUT);
    pinMode(JOYX_PIN, INPUT);
    pinMode(JOYY_PIN, INPUT);

    //Initialise display
    setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

    //Initialise UART
    Serial.begin(9600);
    Serial.println("Hello World");

    // ---- Creating the Mutex ---------------------------------------------------------------

    sysState.mutex = xSemaphoreCreateMutex();

    // ---- Hardware Timer -------------------------------------------------------------------

    TIM_TypeDef *Instance = TIM1;
    HardwareTimer *sampleTimer = new HardwareTimer(Instance);

    // Configure the timer: set period, attach ISR, and start timer
    sampleTimer->setOverflow(22000, HERTZ_FORMAT); // Set Period
    sampleTimer->attachInterrupt(sampleISR); // attaching ISR/interrupt (which here is our sampleISR)
    sampleTimer->resume(); // Start timer

    // ---- Initialize CAN bus ---------------------------------------------------------------

    CAN_Init(true); // Enable loopback mode (receive and acknowledge its own messages)
    setCANFilter(0x123, 0x7ff); // Set up filter to receive messages with ID 0x123
    CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3); // 3 mailbox slots, will block transmit thread on 4th attempt until a semaphore is released by the ISR.

#ifndef DISABLE_THREADS
    CAN_RegisterRX_ISR(CAN_RX_ISR); // Register the ISR function for receiving CAN messages
    CAN_RegisterTX_ISR(CAN_TX_ISR); // Register the ISR function for transmitting CAN messages
#endif

    CAN_Start(); // Start the CAN bus

    // ---- Intialise Queue Handler ----------------------------------------------------------

#ifndef DISABLE_THREADS
    msgOutQ = xQueueCreate(36, 8); // Create a queue with capacity for 36 items of 8 bytes each
#else
    msgOutQ = xQueueCreate(384, 8); // 384 items of 8 bytes each (hence scankeystask runs 32 times * 12 messages = 384 items)
#endif

    msgInQ = xQueueCreate(36, 8);

    // ---- Task Scheduler (Threading) -------------------------------------------------------

#ifndef DISABLE_THREADS // Disable following threads "#define DISABLE_THREADS" is defined
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
        scanKeysTask,    /* Function that implements the task */
        "scanKeys",      /* Text name for the task */
        64,              /* Stack size in words, not bytes */
        NULL,            /* Parameter passed into the task */
        2,               /* Task priority */
        &scanKeysHandle  /* Pointer to store the task handle */
    );

    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(
        displayUpdateTask,   /* Function that implements the task */
        "displayUpdate",     /* Text name for the task */
        256,                 /* Stack size in words */
        NULL,                /* Parameter passed into the task */
        1,                   /* Task priority (lower than scanKeysTask) */
        &displayUpdateHandle /* Pointer to store the task handle */
    );
    
    TaskHandle_t decodeTaskHandle = NULL;
    xTaskCreate(
        CAN_RX_Task,      /* Function that implements the task */
        "CAN_RX_Task",    /* Text name for the task */
        1024,             /* Stack size in words, not bytes */
        NULL,             /* Parameter passed into the task */
        3,                /* Task priority */
        &decodeTaskHandle /* Pointer to store the task handle */
    );

    TaskHandle_t encodeTaskHandle = NULL;
    xTaskCreate(
        CAN_TX_Task,      /* Function that implements the task */
        "CAN_TX_Task",    /* Text name for the task */
        256,              /* Stack size in words */
        NULL,             /* Parameter passed into the task (if any) */
        1,                /* Task priority */
        &encodeTaskHandle /* Pointer to store the task handle (if needed) */
    );
#endif

    // ---- Start the FreeRTOS scheduler -----------------------------------------------------
    


    // ---- ifdef TEST_SCANKEYS --------------------------------------------------------------

#ifndef TEST_SCANKEYS
    vTaskStartScheduler();
#else 
    uint32_t startTime = micros(); // Record the start time
    for (int iter = 0; iter < 32; iter++) {
        scanKeysTask(); // Call the task-under-test
    }
    Serial.println(micros() - startTime); // Print the execution time difference
    while(1); // Infinite loop to halt execution after measurement
#endif

}





//--------------------------------------------------------------------------------------//
//                                         Loop                                         //
//--------------------------------------------------------------------------------------//

void loop() { } // Left Empty