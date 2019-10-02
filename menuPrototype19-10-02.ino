/*
 * Interfacing MENU with 2 Rotary Encoder with Arduino
 * Dated: 07-08-2019
 * Catherine Mathieu
 *
 * Setup:
 * Power LCD from the +5V pin of Arduino:
 * LCD RS -> pin 7
 * LCD EN -> pin 6
 * LCD D4 -> pin 11
 * LCD D5 -> pin 10
 * LCD D6 -> pin 9
 * LCD D7 -> pin 8
 * VSS -> GND
 * VDD -> 5V
 * VO  -> potentionmeter ( GND + 5V)
 * RW  -> GND
 * A   -> 220 ohm resistor -> 5V
 * K   -> GND
 *
 * Rotary encoder 1:
 * Encoder Output DT  -> pin 3
 * Encoder Switch SW  -> pin 4
 * Encoder Output CLK -> pin 2
 * + -> 5V
 * GND -> GND
 *
 * Rotary encoder 2:
 * Encoder Output DT  -> pin 22
 * Encoder Switch SW  -> pin 23
 * Encoder Output CLK -> pin 21
 * + -> 5V
 * GND -> GND
 */
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <LiquidCrystal.h>

const int interrupt2 = 2; //Interrupt 2 for clock 1
const int interrupt21 = 21; //Interrupt 21 for clock 2

// LCD pin connection
const int rs = 7, en = 6, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Rotary #1 pin connection
const int CLK1 = 2; //CLK->D2
const int DT1 = 3;  //DT ->D3
const int SW1 = 4;  //SW ->D4

// Rotary #2 pin connection
const int CLK2 = 21;
const int DT2 = 22;
const int SW2 = 23;

// Menu
const int menuChangeDelay = 500;
const int menuMain = 0;
const int menuChorus = 1;
const int menuFlanger = 2;
int menuState = menuMain;

// Clock
int lastClk1 = 0; //CLK initial value
int lastClk2 = 0; //CLK initial value

//Effect value
int chorusWet = 0;
int chorusNumVoices = 0;

int flangerWet = 0;
int flangerDepth = 0;
int flangerDelayRate = 0;
int flangerOffset = 0;

int delayWet = 0;
int delayTime = 0;

int reverbWet = 0;
int reverbTime = 0;

void initializeLcd();
void setupRotaryEncoder(const int sw, const int clk, const int dt, const int interrupt, void (*onClockChange)());
void createMainMenu();
void displayEffectValue(const int valueIndex, const int value);
void createFlangerMenu();
void createChorusMenu();
void updateEffectValue(const int clk, const int dt, int* lastClk, int* effectValue, void (*createMenu)());
void OnClock1Change();
void OnClock2Change();

void setup() {
    initializeLcd();
    setupRotaryEncoder(SW1, CLK1, DT1, interrupt2, OnClock1Change);
    setupRotaryEncoder(SW2, CLK2, DT2, interrupt21, OnClock2Change);
    createMainMenu();
}

void loop() {
    if (!digitalRead(SW1)) { //Read the 1rst button press
        menuState = menuState == menuMain ? menuChorus : menuMain;
        if (menuState != menuChorus) {
            createMainMenu();
        } else {
            createChorusMenu();
        }
        delay(menuChangeDelay);
    } else if (!digitalRead(SW2)) { //Read the 2nd button press
        menuState = menuState == menuMain ? menuFlanger : menuMain;
        if (menuState != menuFlanger) {
            createMainMenu();
        } else {
            createFlangerMenu();
        }
        delay(menuChangeDelay);
    }
    // Other button to be added...
}

void initializeLcd() {
    lcd.begin(16, 2); // 16 characters by 2 lines
}

void setupRotaryEncoder(const int sw, const int clk, const int dt, const int interrupt, void (*onClockChange)()) {
    pinMode(sw, INPUT);
    digitalWrite(sw, HIGH);
    pinMode(clk, INPUT);
    pinMode(dt, INPUT);
    attachInterrupt(interrupt, onClockChange, CHANGE); // Set the interrupt 0 handler, trigger level change
}

void displayEffectValue(int valueIndex, int value) {
    lcd.setCursor(valueIndex * 4, 1);
    lcd.print(value);
    lcd.print("%");
}

void createMainMenu() {
    menuState = menuMain;
    
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("Chr ");
    lcd.print("Flg ");
    lcd.print("Dly ");
    lcd.print("Rev ");
    
    displayEffectValue(0, chorusWet);
    displayEffectValue(1, flangerWet);
    displayEffectValue(2, delayWet);
    displayEffectValue(3, reverbWet);
}

void createChorusMenu() {
    menuState = menuChorus;

    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("Chr ");
    lcd.print("NVo ");

    displayEffectValue(0, chorusWet);
    displayEffectValue(1, chorusNumVoices);
}

void createFlangerMenu() {
    menuState = menuFlanger;

    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("Flg ");
    lcd.print("Dth ");
    lcd.print("DlR ");
    lcd.print("Off ");

    displayEffectValue(0, flangerWet);
    displayEffectValue(1, flangerDepth);
    displayEffectValue(2, flangerDelayRate);
    displayEffectValue(3, flangerOffset);
}

void updateEffectValue(const int clk, const int dt, int *lastClk, int* effectValue, void (*createMenu)()) {
    int clkValue = digitalRead(clk); // Read the CLK pin level
    int dtValue = digitalRead(dt); // Read the DT pin level
    if(*lastClk != clkValue) {
        *lastClk = clkValue;
        *effectValue += (clkValue != dtValue ? -1 : 1); // CLK and inconsistent DT - 1, otherwise + 1
    }
    createMenu();
}

//The interrupt handlers
void OnClock1Change() {
    switch(menuState) {
        case menuMain: updateEffectValue(CLK1, DT1, &lastClk1, &chorusWet, createMainMenu); break;
        case menuChorus: updateEffectValue(CLK1, DT1, &lastClk1, &chorusWet, createChorusMenu); break;
        case menuFlanger: updateEffectValue(CLK1, DT1, &lastClk1, &flangerWet, createFlangerMenu); break;
    }
}

void OnClock2Change() {
    switch(menuState) {
        case menuMain: updateEffectValue(CLK2, DT2, &lastClk2, &flangerWet, createMainMenu); break;
        case menuChorus: updateEffectValue(CLK2, DT2, &lastClk2, &chorusNumVoices, createChorusMenu); break;
        case menuFlanger: updateEffectValue(CLK2, DT2, &lastClk2, &flangerDepth, createFlangerMenu); break;
    }
}
