#include "BluetoothSerial.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
//#include "include/reset.h"
//#include "include/sd_funk.h"

#define SD_CS		5
#define SPI_MOSI	23
#define SPI_MISO	19
#define SPI_SCK		18

#define ADC_0		36 //Volt Pin A
#define ADC_1		35 //Current Pin A
#define ADC_2		39 //Volt Pin B
#define ADC_3		32 //Current Pin B
#define ADC_4   34 //Volt Pin C
#define ADC_5   33 //Current Pin C
//#define ADC_6   15 //Temperature Pin

#define CORE_0 0
#define CORE_1 1

#define PRIORITY_0 0
#define PRIORITY_1 1
#define PRIORITY_2 2
#define PRIORITY_3 3

#define TIMER_0 0
#define TIMER_1 1
#define TIMER_2 2
#define TIMER_3 3

#define DATA_SIZE 12

File output_file;

// Settings
const char folder_name[] = "/ESPMonitor";		// Full path to output file folder
const char file_name[] = "output";		// Base name for the output file
const uint16_t sample_time = 5;			// Time (in us) between measurements, currently supports 5 or greater.
const uint32_t sample_set = 100 * DATA_SIZE;	// Data Points Included, must be multiple of data size.

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("RelayMonitor");
  
  // Utilized for debug, show what caused the last system reset
	Serial.println("CPU0 reset reason:");
	print_reset_reason(rtc_get_reset_reason(0));
	verbose_print_reset_reason(rtc_get_reset_reason(0));
	Serial.println("CPU1 reset reason:");
	print_reset_reason(rtc_get_reset_reason(1));
	verbose_print_reset_reason(rtc_get_reset_reason(1));
}

// Tasks and functions declarations
// TaskHandle_t SetupCardTask;
// void SetupCardTaskFunction(void* parameters);
TaskHandle_t SetupADTask;
void SetupADTaskFunction(void* parameters);
TaskHandle_t ReadADTask;
void ReadADTaskFunction(void* parameters);
TaskHandle_t StoreDataTask;
void StoreDataTaskFunction(void* parameters);

/* Potential for SD card saving?
void SetupCardTaskFunction(void* parameters) {
  // Setup SPI
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, LOW);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Setup SD
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    vTaskDelete(NULL);
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    vTaskDelete(NULL);
  }

  // Get card size
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Create output dir
  SD.mkdir(folder_name);

  // Check existing files
  File root = SD.open(folder_name);
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    vTaskDelete(NULL);
  }

  bool file_exist = false;
  uint16_t file_number = 0;
  char new_file_name[256];
  do {
    uint8_t pos = 0;
    // Folder
    for (uint8_t i = 0; folder_name[i] != '\0'; i++) {
      new_file_name[pos++] = folder_name[i];
    }
    if (new_file_name[pos] != '/')
      new_file_name[pos++] = '/';

    // File
    for (uint8_t i = 0; file_name[i] != '\0'; i++) {
      new_file_name[pos++] = file_name[i];
    }

    pos = appendNumberToString(new_file_name, pos, file_number);

    // End file name
    new_file_name[pos++] = '.';
    new_file_name[pos++] = 'o';
    new_file_name[pos++] = 'u';
    new_file_name[pos++] = 't';
    new_file_name[pos++] = '\0';

    // Check all files in output folder
    file_exist = false;
    File sub_file = root.openNextFile();
    while (sub_file) {
      if (!sub_file.isDirectory()) {
        if (compareStrings(sub_file.name(), new_file_name)) {
          file_exist = true;
          file_number++;
          for (uint8_t i = 0; i < pos; i++)
            new_file_name[i] = '\0';
          break;
        }
      }
      sub_file = root.openNextFile();
    }
  } while (file_exist);

  // Put sample time in file
  uint8_t pos = 0;
  char sample_time_header[256];
  sample_time_header[pos++] = 'S';
  sample_time_header[pos++] = 'T';
  sample_time_header[pos++] = ' ';
  pos = appendNumberToString(sample_time_header, pos, sample_time);
  sample_time_header[pos++] = 'u';
  sample_time_header[pos++] = 's';
  sample_time_header[pos++] = '\n';
  sample_time_header[pos] = '\0';

  // Create output file
  Serial.println(new_file_name);
  output_file = SD.open(new_file_name, FILE_WRITE);
  output_file.print(header_text);
  output_file.print(sample_time_header);

  // AD setup task start
  xTaskCreate(SetupADTaskFunction, "SetupADTask", 10000, NULL, 0, &SetupADTask);
  vTaskDelete(NULL);
}
*/

hw_timer_t * timerInt = NULL;
bool reading_ad = false;
void int1ms() {
	if(reading_ad)
		vTaskResume(ReadADTask);
}

void SetupADTaskFunction(void* parameters) {
	//Setup timer
	timerInt = timerBegin(TIMER_0, 80, true);
	timerAttachInterrupt(timerInt, &int1ms, true);
	timerAlarmWrite(timerInt, sample_time, true);
	timerAlarmEnable(timerInt);

	// Loop tasks start
	xTaskCreate(ReadADTaskFunction, "ReadADTask", 1000, NULL, 0, &ReadADTask);
	vTaskDelete(NULL);
}

uint8_t data_buffer[sample_set];
bool current_buffer = 0;
void ReadADTaskFunction(void* parameters) {
	reading_ad = true;

	while(reading_ad) {
		vTaskSuspend(NULL);
    int voltAadc = 0;
    int voltBadc = 0;
    int voltCadc = 0;
    int currentAadc = 0;
    int currentBadc = 0;
    int currentCadc = 0;
    int tempadc = 0;

    int voltValA = 0;
    int voltValB = 0;
    int voltValC = 0;
    int currentValA = 0;
    int currentValB = 0;
    int currentValC = 0;
    int tempVal = 0;    
    
		// Read values
		static uint16_t adc0val, adc1val, adc2val, adc3val, adc4val, adc5val, adc6val;
    int iter = 0;
    while (iter < 0):
		adc0val = analogRead(ADC_0);
		adc1val = analogRead(ADC_1);
		adc2val = analogRead(ADC_2);
		adc3val = analogRead(ADC_3);
    adc4val = analogRead(ADC_4);
    adc5val = analogRead(ADC_5);
    //adc6val = analogRead(ADC_6);    

		// Pass data to buffer
		static uint16_t position_count = 0;
		data_buffer[position_count++] = (uint8_t)((adc0val & 0x0ff0)>>4);
		data_buffer[position_count++] = (uint8_t)((adc0val & 0x000f)<<4) | (uint8_t)((adc1val & 0x0f00)>>8);
		data_buffer[position_count++] = (uint8_t)((adc1val & 0x00ff)<<0);
		data_buffer[position_count++] = (uint8_t)((adc2val & 0x0ff0)>>4);
		data_buffer[position_count++] = (uint8_t)((adc2val & 0x000f)<<4) | (uint8_t)((adc3val & 0x0f00)>>8);
		data_buffer[position_count++] = (uint8_t)((adc3val & 0x00ff)<<0);
	
		// Swap buffers, and flush the one filled
		if(position_count >= sample_set) {
			timerAlarmDisable(timerInt);
			reading_ad = false;
			vTaskResume(ReadADTask);
			xTaskCreate(StoreDataTaskFunction, "StoreDataTask", 5000, NULL, 0, &StoreDataTask);
		}
	}

	while(1)
		vTaskDelay(10/portTICK_PERIOD_MS);
	vTaskSuspend(NULL);
}

void StoreDataTaskFunction(void* parameters) {
	// Flush data in buffer
	output_file.write(data_buffer, sample_set);

	output_file.close();
	Serial.println("Done");

	while(1)
		vTaskDelay(10/portTICK_PERIOD_MS);
	vTaskSuspend(NULL);
}

void loop() {
  /* Significant rewrites, retain loop for operation - moving code to function calls.
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
  Serial.write(SerialBT.read());
  }
  i = 0;
  Va = 0;
  Vb = 0;
  Vc = 0;
  Aa = 0;
  Ab = 0;
  Ac = 0;
  //T = 0;
  while (i < 100):
    voltAadc = analogRead(voltPinA);
    voltBadc = analogRead(voltPinB);
    voltCadc = analogRead(voltPinC);
    currentAadc = analogRead(currentPinA);
    currentBadc = analogRead(currentPinB);
    currentCadc = analogRead(currentPinC);
    //tempadc = analogRead(tempPin);
    voltValA = floor(0.00080586*voltAadc);
    voltValB = floor(0.00080586*voltBadc);
    voltValC = floor(0.00080586*voltCadc);
    currentValA = floor(0.00080586*currentAadc);
    currentValB = floor(0.00080586*currentBadc);
    currentValC = floor(0.00080586*currentCadc);
    //tempVal = floor(0.00080586*tempadc);
    if (voltValA > Va):
      Va = voltValA;
    if (voltValB > Vb):
      Vb = voltValB;
    if (voltValC > Vc):
      Vc = voltValC;
    if (currentValA > Aa):
      Aa = currentValA;
    if (currentValB > Ab):
      Ab = currentValB;
    if (currentValC > Ac):
      Ac = currentValC;
    //if (tempVal > T):
    //  T = tempVal  
    i += 1;    
  delay(100);
  Serial.print("Phase A: voltage = ");
  Serial.print(Va);
  Serial.print(" current = ");
  Serial.print(Aa);
  Serial.print(" Phase B: voltage = ");
  Serial.print(Vb);
  Serial.print(" current = ");
  Serial.print(Ab);
  Serial.print(" Phase C: voltage = ");
  Serial.print(Vc);
  Serial.print(" current = ");
  Serial.print(Ac);
  //Serial.print("Temperature = ");
  //Serial.print(tempVal);
  */
}
