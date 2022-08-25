/****************************************************************** SOME SYSTEM MACROS *******************************************************************/
#define BLYNK_TEMPLATE_ID                                                                   "TMPLeC90ri1t"
#define BLYNK_DEVICE_NAME                                                                   "SmartHomeVer5"
#define BLYNK_AUTH_TOKEN                                                                    "xJZzlk0JzNCD1IF1o3lsIlO9jWjeo9uH"
#define BLYNK_FIRMWARE_VERSION                                                              "0.1.0"
#define BLYNK_PRINT                                                                         Serial

/****************************************************************** Some references *******************************************************************/

#ifdef REFERENCES
https://linuxhint.com/createchar-method-arduino/
https://www.arduino.cc/reference/en/language/functions/math/map/
https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
http://www.ocfreaks.com/basics-interfacing-dht11-dht22-humidity-temperature-sensor-mcu/
#endif

/****************************************************************** INCLUDE HEADER FILE *******************************************************************/

#include "BlynkEdgent.h"
#include <Servo.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Fingerprint.h>

/****************************************************************** MACRO DEFINITION *******************************************************************/
/* Other */
#define START_OF_TRANSMISSION()                                                                       
#define STARTING_VALUE                                                                      (0U)
#define LOGIC_HIGH                                                                          (1U)
#define LOGIC_LOW                                                                           (0U)
#define APP_DEBUG
#define WINDOW_CONTROL_PIN                                                                  (34U)
#define LCD_ADDRESS                                                                         (0x27U)
#define LCD_NUMBER_OF_COLUMN                                                                (16U)
#define LCD_NUMBER_OF_ROW                                                                   (2U)
#define USER_THRESHOLD                                                                      (4000U)
#define FINGER_PRINT_1                                                                      (1U)
/* Macro for pin definition */
#define BUZZ                                                                                (12U)
#define PIN_SERVO                                                                           (19U)
#define PIN_GAS                                                                             (36U)
#define MOTION_DETECTION_SENSOR_PIN                                                         (33U)
#define INFRARED_DETECTION_SENSOR_PIN                                                       (13U)
#define LED_IN_BATHROOM_CONTROL_PIN                                                         (34U)
#define PIN_LIGHT                                                                           (34U)
#define PIN_FANPWM                                                                          (14U)
#define PIN_LEDPWM                                                                          (23U)
#define PIN_STEP1                                                                           (5U)
#define WINDOW_STEP_MOTOR_DIRECTION_CONTROL_PIN                                             (18U)
#define PIN_STEP2                                                                           (4U)
#define CLOTHES_LINE_DIRECTION_CONTROL_PIN                                                  (15U)
#define DHT_PIN                                                                             (32U)
#define BUTTON_PIN                                                                          (35U)
/* Macro for some initial values */
#define ID_INITIAL_VALUE                                                                    (1U)
#define GENERAL_INITIAL_VALUE                                                               (0U)
#define TEMPERATURE_MAX_VALUE                                                               (42U)
#define MaxGasContent_VALUE                                                                 (60U)
#define UNRECOGNIZED_COUNT_INITIAL_VALUE                                                    (0U)
#define FAN_PWM_INITIAL_VALUE                                                               (90u)
#define FAN_PWM_1_INITIAL_VALUE                                                             (0U)
/* Macros for states of some actuators */
#define AUTO_WINDOW_ON                                                                      (1U)
#define AUTO_WINDOW_OFF                                                                     (0U)
#define WINDOW_CLOSE                                                                        (0U)
#define WINDOW_OPEN                                                                         (1U)
#define WINDOW_STOP                                                                         (2U)
#define CURRENT_STATE_WINDOW_OPEN                                                           (0U)
#define CURRENT_STATE_WINDOW_CLOSE                                                          (1U)
#define LIGHT_SENSOR_THRESHOLD_NIGHT                                                        (4000U)
#define LIGHT_SENSOR_THRESHOLD_DAY                                                          (3500U)
#define CLOTHES_LINE_CLOSE                                                                  (0U)
#define CLOTHES_LINE_OPEN                                                                   (1U)
#define CLOTHES_LINE_CONTROL_PIN                                                            (39U)
#define RAIN_SENSOR_THRESHOLD_RAINING                                                       (4000U)
#define RAIN_SENSOR_THRESHOLD_NOT_RAINING                                                   (100U)
#define CURRENT_STATE_CLOTHES_LINE_OPEN                                                     (0U)
#define CURRENT_STATE_CLOTHES_LINE_CLOSE                                                    (1U)
#define MOTION_DETECTED                                                                     (1U)
#define INFRARED_RADIATION_DETECTED                                                         (0U)
#define DEFAULT_VALUE                                                                       (0b00000000)
#define DOOR_LED_SHIFT_VALUE_ON                                                             (0b01000000)
#define DOOR_LED_SHIFT_VALUE_OFF                                                            (0b10111111)
#define LED_2_SHIFT_VALUE_ON                                                                (0b00100000)
#define LED_2_SHIFT_VALUE_OFF                                                               (0b11011111)
#define LED_3_SHIFT_VALUE_ON                                                                (0b00010000)
#define LED_3_SHIFT_VALUE_OFF                                                               (0b11101111)
#define FAN_TURN_ON                                                                         (0b10000000)
#define FAN_TURN_OFF                                                                        (0b01111111)
/* Macro for time delay */
#define DELAY_100_MS                                                                        (100U)
#define DELAY_50_MS                                                                         (50U)
#define DELAY_20_MS                                                                         (20U)
#define DELAY_25_MS                                                                         (25U)
#define DELAY_500_MS                                                                        (500U)
#define DELAY_1500_MS                                                                       (1500U)
#define DELAY_2000_MS                                                                       (2000U)
#define DELAY_1000_MS                                                                       (1000U)
#define PERIOD_1000_MS                                                                      (1000U)
/* Macro for Clothesline module*/
#define CLOTHES_LINE_PULSE_DELAY                                                            (1600U)
#define CLOTHES_LINE_MOTOR_STEP                                                             (110U)
/*Macro for some default baud rate for serial communication */
#define BAUD_RATE_9600                                                                      (9600U)
#define BAUD_RATE_57600                                                                     (57600U)
/* Macro for controlling window system */
#define WINDOW_MOTOR_STEP                                                                   (250U)
#define WINDOW_PULSE_DELAY                                                                  (1200U)
/* Macro for servo motor controlling the door */
#define SERVO_OPEN                                                                          (120U)
#define SERVO_CLOSE                                                                         (10U)
/* Light standart state */
#define STD_ON                                                                              (1U)
#define STD_OFF                                                                             (0U)
/* Macro for speaker */
#define SPEAKER_ONE_TIME_GENERATE_SOUND                                                     (1U)
#define SPEAKER_TWO_TIME_GENERATE_SOUND                                                     (2U)
#define SPEAKER_THREE_TIME_GENERATE_SOUND                                                   (3U)
/* Macro for boundary value */
#define LOWER_BOUND_GAS_CONTENT_VALUE                                                       (1600U)
#define UPPER_BOUND_GAS_CONTENT_VALUE                                                       (4095U)
#define LOWER_BOUND_GAS_CONTENT_OUTPUT                                                        (0U)
#define UPPER_BOUND_GAS_CONTENT_OUTPUT                                                      (100U)
#define LOWER_BOUND_FAN_PWM                                                                 (90U)
#define UPPER_BOUND_FAN_PWM                                                                 (255U)
#define LOWER_BOUND_FAN_PWM_OUTPUT                                                          (0U)
#define UPPER_BOUND_FAN_PWM_OUTPUT                                                          (100U)
#define UPPER_BOUND_LED                                                                     (4095U)
#define LOWER_BOUND_LED                                                                     (300U)
#define UPPER_BOUND_LED_OUTPUT                                                              (250U)
#define LOWER_BOUND_LED_OUTPUT                                                              (0U)
/* Macro for fingerprint module */
#define GET_FINGER_PRINT_ERROR                                                              (-1)
#define T2Z_FINGER_PRINT_ERROR                                                              (-2)
#define FINGER_PRINT_FAST_SEARCH_ERROR                                                      (-3)
#define USER_INDENTITY_RECOGNIZED                                                           (1U)
#define USER_INDENTITY_UNRECOGNIZED                                                         (-1)
#define MAX_UNRECOGNIZED_INDENDITY                                                          (2U)
#define USER_ERROR                                                                               (-2)
/* Macro for 74HC595 */
#define SHIFT_RESIGTER_CLOCK_PIN                                                            (25U)
#define SHIFT_RESIGTER_LATCH_PIN                                                            (26U)
#define SHIFT_RESIGTER_DATA_PIN                                                             (27U)
/* Macro for module sim */
#define STARTING_MESSAGE_MODULE_SIM                                                         "AT+CMGF=1"
#define PHONE_NUMBER_MESSAGE                                                                "AT+CMGS=\""+ my_Phone + "\""
#define MESSAGE_CONTENT_THEFT_WARNING                                                       "Canh bao trom"
#define MESSAGE_CONTENT_FIRE_WARNING                                                        "Fire Warning!"
#define END_OF_MESSAGE_SMS_CODE                                                             (0x1a)
/* Macro for data read from sever */
#define FAN_PWM_CHANNEL                                                                     (3U)
#define LED_PWM_CHANNEL                                                                     (2U)
#define BLYNK_DATA_FOR_WINDOW                                                               V7
#define BLYNK_DATA_AUTO_MODE_FOR_WINDOW                                                     V6
#define BLYNK_DATA_FAN_SPEED                                                                V5
#define BLYNK_DATA_LED_3                                                                    V4     
#define BLYNK_DATA_LED_2                                                                    V3
#define SEVER_TEMPERATURE_DATA                                                              V0
#define SEVER_HUMUDITY_DATA                                                                 V2
#define SERVER_GAS_CONTENT_DATA                                                             V1   
/* Other configuration */
#define DEFAULT_RESOLUTION                                                                  (8U)
#define FREQUENCY_PWM                                                                       (5000U)
/****************************************************************** VARIABLE DEFINITION *******************************************************************/
BlynkTimer timer;
LiquidCrystal_I2C lcd(LCD_ADDRESS,LCD_NUMBER_OF_COLUMN,LCD_NUMBER_OF_ROW);
DHT dht(DHT_PIN, DHT11);
Servo servo1;
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&Serial2);
int Indentification = ID_INITIAL_VALUE;
int ReadAnalog;
byte MaxTemperature = TEMPERATURE_MAX_VALUE;
byte MaxGasContent = MaxGasContent_VALUE;
byte Temperature = GENERAL_INITIAL_VALUE;
byte Humidity = GENERAL_INITIAL_VALUE;     
byte NumberOfUnrecognizedIndentity=UNRECOGNIZED_COUNT_INITIAL_VALUE;
byte FanPWM=FAN_PWM_INITIAL_VALUE;
byte Fan_Speed_Percentage=FAN_PWM_1_INITIAL_VALUE;
unsigned int GasContent;
int pwmled = 0;
boolean AutoMode=AUTO_WINDOW_OFF;
boolean CurrentStateWindow=WINDOW_CLOSE;
boolean CurrentStateOfClothesLine=0;
String my_Phone  = "+84949691914";
byte vuong[] = 
{
  B01110,
  B01010,
  B01110,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};
byte Value = DEFAULT_VALUE;
/****************************************************************** CODE IMPLEMENTATION *******************************************************************/
BLYNK_WRITE(BLYNK_DATA_LED_2)
{
  byte Lamp2 = param.asInt();
  if(Lamp2 == STD_ON)       
  {
    Value = LED_2_SHIFT_VALUE_ON | Value;
    Write(Value);
  }
  else if(Lamp2 == STD_OFF)  
  {
    Value = LED_2_SHIFT_VALUE_OFF & Value;
    Write(Value);
  }
}
BLYNK_WRITE(BLYNK_DATA_LED_3)
{
 byte Lamp3 = param.asInt();
  if(Lamp3 == STD_ON)       
  {
    Value = LED_3_SHIFT_VALUE_ON | Value;
    Write(Value);
  }
  else if(Lamp3 == STD_OFF)  
  {
    Value = LED_3_SHIFT_VALUE_OFF & Value;
    Write(Value);
  }
}
BLYNK_WRITE(BLYNK_DATA_FAN_SPEED)
{
 FanPWM = param.asInt();
 ledcWrite(FAN_PWM_CHANNEL, FanPWM);
}
BLYNK_WRITE(BLYNK_DATA_AUTO_MODE_FOR_WINDOW)
{
 AutoMode=param.asInt();
}
BLYNK_WRITE(BLYNK_DATA_FOR_WINDOW)
{
 byte WindowData = param.asInt();
 if(WindowData == WINDOW_CLOSE && CurrentStateWindow == CURRENT_STATE_WINDOW_OPEN)
 {
  Window_Control(WINDOW_CLOSE);
  CurrentStateWindow = CURRENT_STATE_WINDOW_OPEN;
 }
 else if(WindowData == WINDOW_OPEN && CurrentStateWindow == CURRENT_STATE_WINDOW_CLOSE)
 {
  Window_Control(WINDOW_OPEN);
  CurrentStateWindow = CURRENT_STATE_WINDOW_CLOSE;
 }
}
void setup() 
{
  Serial.begin(BAUD_RATE_9600);
  BlynkEdgent.begin();
  pinMode(BUZZ,OUTPUT); 
  pinMode(SHIFT_RESIGTER_LATCH_PIN, OUTPUT); 
  pinMode(SHIFT_RESIGTER_DATA_PIN, OUTPUT); 
  pinMode(SHIFT_RESIGTER_CLOCK_PIN, OUTPUT);
  Write(DEFAULT_VALUE);
  pinMode(PIN_GAS,INPUT); 
  pinMode(MOTION_DETECTION_SENSOR_PIN,INPUT); 
  pinMode(INFRARED_DETECTION_SENSOR_PIN,INPUT);
  pinMode(PIN_STEP1,OUTPUT); 
  pinMode(WINDOW_STEP_MOTOR_DIRECTION_CONTROL_PIN,OUTPUT); 
  pinMode(PIN_STEP2,OUTPUT); 
  pinMode(CLOTHES_LINE_DIRECTION_CONTROL_PIN,OUTPUT);
  lcd.init();
  lcd.backlight();
  lcd.print("Smart home V5");
  delay(DELAY_100_MS);
  lcd.setCursor(0,0); lcd.print("Device detect.....");
  finger.begin(BAUD_RATE_57600);
  if (finger.verifyPassword()) 
  {
  lcd.setCursor(0,1); lcd.print("      Done!     ");
  delay(DELAY_1000_MS);
  lcd.clear();
  } 
  else 
  {
    lcd.setCursor(0,1); lcd.print("   Device error !  ");
    while (1) 
    { 
      delay(DELAY_20_MS); 
    }
  }
  dht.begin();
  servo1.attach(PIN_SERVO);
  servo1.write(SERVO_CLOSE);
  ledcSetup(LED_PWM_CHANNEL, FREQUENCY_PWM, DEFAULT_RESOLUTION);
  ledcAttachPin(PIN_LEDPWM, LED_PWM_CHANNEL);
  ledcSetup(FAN_PWM_CHANNEL, FREQUENCY_PWM, DEFAULT_RESOLUTION);
  ledcAttachPin(PIN_FANPWM, FAN_PWM_CHANNEL);
  timer.setInterval(PERIOD_1000_MS,Blynk_Update_Data);
}
void Humidity_And_Temperature_Updater()
{
  START_OF_TRANSMISSION();
  Temperature = dht.readTemperature();
  Humidity = dht.readHumidity();
  return;
}
void Gas_Content_Updater()
{
  GasContent = map(analogRead(PIN_GAS),LOWER_BOUND_GAS_CONTENT_VALUE,UPPER_BOUND_GAS_CONTENT_VALUE,LOWER_BOUND_GAS_CONTENT_OUTPUT,UPPER_BOUND_GAS_CONTENT_OUTPUT);
  if(Temperature >= MaxTemperature || GasContent >= MaxGasContent)
  {
    digitalWrite(BUZZ,LOGIC_HIGH);
    Fire_Warning_SMS();
    while(Temperature >= MaxTemperature || GasContent >= MaxGasContent)
    {

    }
    digitalWrite(BUZZ,LOGIC_LOW);
  }
  return;
}
void Bathroom_Control_System()
{
  if(digitalRead(INFRARED_DETECTION_SENSOR_PIN) == INFRARED_RADIATION_DETECTED)
  {
    Value = FAN_TURN_ON | Value ;
    Write(Value);
    pwmled = map(analogRead(LED_IN_BATHROOM_CONTROL_PIN),LOWER_BOUND_LED,UPPER_BOUND_LED,LOWER_BOUND_LED_OUTPUT,UPPER_BOUND_LED_OUTPUT);
    ledcWrite(LED_PWM_CHANNEL, pwmled);
  }
  else 
  {
    Value = FAN_TURN_OFF & Value;
    Write(Value);
    ledcWrite(LED_PWM_CHANNEL,STD_OFF);
  }
}
void Is_New_FingerPrint()
{
  ReadAnalog = analogRead(BUTTON_PIN);
  if(ReadAnalog < USER_THRESHOLD)
  {
    delay(DELAY_25_MS);
    if(ReadAnalog < USER_THRESHOLD)
    {
      while(ReadAnalog < USER_THRESHOLD)  
      ReadAnalog = analogRead(BUTTON_PIN);
      Speaker_Control(SPEAKER_ONE_TIME_GENERATE_SOUND);
      while (Finger_Print_Sample(FINGER_PRINT_1) != USER_ERROR);
    }
  }
  return;
}
void Door_Control()
{
  if(digitalRead(MOTION_DETECTION_SENSOR_PIN) == MOTION_DETECTED)
  {
    Value = DOOR_LED_SHIFT_VALUE_ON | Value;
    Write(Value);
    servo1.write(SERVO_OPEN);
    delay(DELAY_2000_MS);
    Value = DOOR_LED_SHIFT_VALUE_OFF & Value;
    Write(Value);
    servo1.write(SERVO_CLOSE);
  }
  return;
}
void Bedroom_Fan_Speed_Updater()
{
  Fan_Speed_Percentage = map(FanPWM,LOWER_BOUND_FAN_PWM,UPPER_BOUND_FAN_PWM,LOWER_BOUND_FAN_PWM_OUTPUT,UPPER_BOUND_FAN_PWM_OUTPUT);
  General_Parameter_Updater();
  return;
}
void loop() 
{
  Humidity_And_Temperature_Updater();
  Gas_Content_Updater();
  Bathroom_Control_System();
  Door_Control();
  Bedroom_Fan_Speed_Updater();
  Auto_Window_Control();
  Auto_Clothes_Line_Control();
  Indentification_Check();
  /* If user pushes the button, it will enter learning new finger print mode */
  Is_New_FingerPrint();
  BlynkEdgent.run();
  timer.run();
}
void Speaker_Control(int Time)
{
for (int j = STARTING_VALUE ; j < Time ; j++)
{
  digitalWrite(BUZZ,LOGIC_HIGH);
  delay(DELAY_100_MS);
  digitalWrite(BUZZ,LOGIC_LOW);
  delay(DELAY_100_MS);
}
}
int Finger_Print_Sample(int Indentification)
{
  int ReturnValue = -1;
  /* Add finger print */
  while (ReturnValue != FINGERPRINT_OK) 
  {
  
    ReturnValue = finger.getImage();
    if (ReturnValue==FINGERPRINT_OK);
  }
  ReturnValue = finger.image2Tz(1);
  /* Save the first time */
  if (ReturnValue == FINGERPRINT_OK) 
  { 
    Speaker_Control(SPEAKER_ONE_TIME_GENERATE_SOUND);
  }
  delay(DELAY_1000_MS);
  ReturnValue = 0;
  while (ReturnValue != FINGERPRINT_NOFINGER) {
    ReturnValue = finger.getImage();
  }
  ReturnValue = -1;
  while (ReturnValue != FINGERPRINT_OK) {
    ReturnValue = finger.getImage();
    if (ReturnValue==FINGERPRINT_OK); 
  }
  ReturnValue = finger.image2Tz(2);
  /* Save the second time */
  if (ReturnValue==FINGERPRINT_OK)
  {
    Speaker_Control(SPEAKER_TWO_TIME_GENERATE_SOUND);
  }
  ReturnValue = finger.createModel();
  if (ReturnValue == FINGERPRINT_OK) 
  {
    Serial.println("Prints matched!");
  } 
  else if (ReturnValue == FINGERPRINT_PACKETRECIEVEERR) 
  {
    return ReturnValue;
  } 
  else if (ReturnValue == FINGERPRINT_ENROLLMISMATCH) 
  {
    return ReturnValue;
  } 
  else 
  {
    return ReturnValue;
  }      
  ReturnValue = finger.storeModel(Indentification);
  /* Save successfully */
  if (ReturnValue == FINGERPRINT_OK) {
    Speaker_Control(SPEAKER_THREE_TIME_GENERATE_SOUND);
  } 
  else if 
  (ReturnValue == FINGERPRINT_PACKETRECIEVEERR) 
  {
    return ReturnValue;
  } else if 
  (ReturnValue == FINGERPRINT_BADLOCATION) 
  {
    return ReturnValue;
  } 
  else if (ReturnValue == FINGERPRINT_FLASHERR) 
  {
    return ReturnValue;
  } 
  else 
  {
    return ReturnValue;
  }
  return USER_ERROR;
}

int Finger_Print_Reception() {
 
  uint8_t ReturnValue = finger.getImage();

  if (ReturnValue != FINGERPRINT_OK)
  {  
    return GET_FINGER_PRINT_ERROR;
  }

  ReturnValue = finger.image2Tz();
  if (ReturnValue != FINGERPRINT_OK)
  {
    return T2Z_FINGER_PRINT_ERROR;
  }

  ReturnValue = finger.fingerFastSearch();
  if (ReturnValue != FINGERPRINT_OK)
  {
    return FINGER_PRINT_FAST_SEARCH_ERROR;
  }
 
  return finger.fingerID; 
}

void Indentification_Check(void)
{
  Indentification = Finger_Print_Reception();
  if(Indentification == USER_INDENTITY_RECOGNIZED)
  {   
    Speaker_Control(SPEAKER_ONE_TIME_GENERATE_SOUND); //   Có vân tay 
    servo1.write(SERVO_OPEN);
    delay(DELAY_1500_MS);
    servo1.write(SERVO_CLOSE);                                   
  }
  else if(Indentification < USER_INDENTITY_UNRECOGNIZED) 
  {
    Speaker_Control(SPEAKER_THREE_TIME_GENERATE_SOUND);
    NumberOfUnrecognizedIndentity++;
    if(NumberOfUnrecognizedIndentity > MAX_UNRECOGNIZED_INDENDITY)
    {
      Illegal_Intrusion_Warning_SMS();
      NumberOfUnrecognizedIndentity = UNRECOGNIZED_COUNT_INITIAL_VALUE;
    }
  }
}
void General_Parameter_Updater()
{
  /* Temperature = dht.readTemperature();
  Humidity = dht.readHumidity();
  GasContent = map(analogRead(PIN_GAS),LOWER_BOUND_GAS_CONTENT_VALUE,UPPER_BOUND_GAS_CONTENT_VALUE,LOWER_BOUND_GAS_CONTENT_OUTPUT,UPPER_BOUND_GAS_CONTENT_OUTPUT); */
  lcd.setCursor(0,0);
  lcd.print("T: "); 
  lcd.print(Temperature);
  lcd.print(" "); 
  lcd.createChar(0,vuong);
  lcd.setCursor(5,0);
  lcd.write(0);
  lcd.print("C   ");
  lcd.setCursor(0,1);
  lcd.print("G: "); 
  lcd.print(GasContent);
  lcd.print(" %  "); 
  lcd.print("SF:"); 
  lcd.print(Fan_Speed_Percentage); 
  lcd.print("%     ");
  /* Motion detected */
  //WC detects human
}
void Blynk_Update_Data()
{
  Blynk.virtualWrite(SEVER_TEMPERATURE_DATA,Temperature);
  Blynk.virtualWrite(SEVER_HUMUDITY_DATA,Humidity);
  Blynk.virtualWrite(SERVER_GAS_CONTENT_DATA,GasContent);
}
void Fire_Warning_SMS()
{
  Serial.begin(BAUD_RATE_9600);
  delay(DELAY_50_MS);
  Serial.println(STARTING_MESSAGE_MODULE_SIM);  
  delay(DELAY_50_MS);
  /* Send message by using AT command */
  Serial.println(PHONE_NUMBER_MESSAGE);  
  delay(DELAY_50_MS);
  /* Content of the messgaes */
  Serial.println(MESSAGE_CONTENT_FIRE_WARNING); 
  delay(DELAY_50_MS);
  /* Send ending message */
  Serial.write(END_OF_MESSAGE_SMS_CODE);
  delay(DELAY_500_MS);   
  Serial.end();
}
void Illegal_Intrusion_Warning_SMS()
{
  Serial.begin(BAUD_RATE_9600);
  delay(DELAY_50_MS);
  Serial.println(STARTING_MESSAGE_MODULE_SIM);  
  delay(DELAY_50_MS);
  /* Send message by using AT command */
  Serial.println(PHONE_NUMBER_MESSAGE);  
  delay(DELAY_50_MS);
  /* Content of the messgaes */
  Serial.println(MESSAGE_CONTENT_THEFT_WARNING); 
  delay(DELAY_20_MS);
  /* Send ending message */
  Serial.write(END_OF_MESSAGE_SMS_CODE);
  delay(DELAY_500_MS);   
  Serial.end();
}
void Auto_Window_Control()
{
  if(analogRead(WINDOW_CONTROL_PIN) > LIGHT_SENSOR_THRESHOLD_NIGHT && CurrentStateWindow == CURRENT_STATE_WINDOW_OPEN && AutoMode == AUTO_WINDOW_ON)
  {
    Window_Control(WINDOW_CLOSE);
    CurrentStateWindow = CURRENT_STATE_WINDOW_CLOSE;
  }
  else if(analogRead(WINDOW_CONTROL_PIN) < LIGHT_SENSOR_THRESHOLD_DAY && CurrentStateWindow == CURRENT_STATE_WINDOW_CLOSE && AutoMode == AUTO_WINDOW_ON)
  {
    Window_Control(WINDOW_OPEN);
    CurrentStateWindow = CURRENT_STATE_WINDOW_OPEN;
  }
}
void Window_Control(byte WindowCommand)
{
  //n=1=>Open
  if(WindowCommand == WINDOW_OPEN)
  {
    digitalWrite(WINDOW_STEP_MOTOR_DIRECTION_CONTROL_PIN,WINDOW_OPEN);
    for(int i = STARTING_VALUE ; i < WINDOW_MOTOR_STEP ; i++)
    { 
      digitalWrite(PIN_STEP1,LOGIC_HIGH);
      delayMicroseconds(WINDOW_PULSE_DELAY);
      digitalWrite(PIN_STEP1,LOGIC_LOW);
      delayMicroseconds(WINDOW_PULSE_DELAY);
    } 
  }
  //n=0=>Close
  else if(WindowCommand == WINDOW_CLOSE)
  {
    digitalWrite(WINDOW_STEP_MOTOR_DIRECTION_CONTROL_PIN,WINDOW_CLOSE);
    for(int i = STARTING_VALUE ; i < WINDOW_MOTOR_STEP ; i++)
    {
      digitalWrite(PIN_STEP1,LOGIC_HIGH);
      delayMicroseconds(WINDOW_PULSE_DELAY);
      digitalWrite(PIN_STEP1,LOGIC_LOW);
      delayMicroseconds(WINDOW_PULSE_DELAY);
    }
  }
  else if(WindowCommand == WINDOW_STOP)
  { 
    digitalWrite(PIN_STEP1,LOGIC_LOW); 
  }
}
void Clothes_Line_Control(boolean CLothesLineCommand)
{
  if(CLothesLineCommand == CLOTHES_LINE_CLOSE)
  {
     //dong
    digitalWrite(CLOTHES_LINE_DIRECTION_CONTROL_PIN,0);
    for(int i = STARTING_VALUE ; i < CLOTHES_LINE_MOTOR_STEP ; i++)
    {
      digitalWrite(PIN_STEP2,LOGIC_HIGH);
      delayMicroseconds(CLOTHES_LINE_PULSE_DELAY);
      digitalWrite(PIN_STEP2,LOGIC_LOW);
      delayMicroseconds(CLOTHES_LINE_PULSE_DELAY);
    }
  }
  if(CLothesLineCommand == CLOTHES_LINE_OPEN)
  {
     //mo
    digitalWrite(CLOTHES_LINE_DIRECTION_CONTROL_PIN,1);
    for(int i = STARTING_VALUE ; i < CLOTHES_LINE_MOTOR_STEP ; i++)
    {
      digitalWrite(PIN_STEP2,LOGIC_HIGH);
      delayMicroseconds(CLOTHES_LINE_PULSE_DELAY);
      digitalWrite(PIN_STEP2,LOGIC_LOW);
      delayMicroseconds(CLOTHES_LINE_PULSE_DELAY);
    }
  }
}
void Auto_Clothes_Line_Control()
{
  //neu co mua
  if(analogRead(CLOTHES_LINE_CONTROL_PIN) < RAIN_SENSOR_THRESHOLD_RAINING && CurrentStateOfClothesLine == CURRENT_STATE_CLOTHES_LINE_OPEN)
  {
    Clothes_Line_Control(CLOTHES_LINE_CLOSE);
    CurrentStateOfClothesLine = CURRENT_STATE_CLOTHES_LINE_CLOSE;
  }
  else if(analogRead(CLOTHES_LINE_CONTROL_PIN) > RAIN_SENSOR_THRESHOLD_NOT_RAINING && CurrentStateOfClothesLine == CURRENT_STATE_CLOTHES_LINE_OPEN)
  {
    Clothes_Line_Control(CLOTHES_LINE_OPEN);
    CurrentStateOfClothesLine = CURRENT_STATE_CLOTHES_LINE_OPEN;
  }
}
void Write(byte ShiftValue)
{
  digitalWrite(SHIFT_RESIGTER_LATCH_PIN, LOGIC_LOW);
  shiftOut(SHIFT_RESIGTER_DATA_PIN, SHIFT_RESIGTER_CLOCK_PIN, LSBFIRST, ShiftValue);
  digitalWrite(SHIFT_RESIGTER_LATCH_PIN, LOGIC_HIGH);
}
/****************************************************************** END OF FILE *******************************************************************/
