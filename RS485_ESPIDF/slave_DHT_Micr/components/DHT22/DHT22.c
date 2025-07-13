/*------------------------------------------------------------------------------
	DHT22 temperature & humidity sensor AM2302 (DHT22) driver for ESP32
	Jun 2017:	Ricardo Timmermann, new for DHT22  	
	Code Based on Adafruit Industries and Sam Johnston and Coffe & Beer. Please help
	to improve this code. 
	
	This example code is in the Public Domain (or CC0 licensed, at your option.)
	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.
	PLEASE KEEP THIS CODE IN LESS THAN 0XFF LINES. EACH LINE MAY CONTAIN ONE BUG !!!
---------------------------------------------------------------------------------*/

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

#include "DHT22.h"

// == global defines =============================================

static const char* TAG = "DHT";
 
// Internal function prototype
static int getSignalLevel( int gpio, int usTimeOut, bool state );

dht22_handle_t* dht22_init(int gpio) {
    dht22_handle_t* handle = (dht22_handle_t*) malloc(sizeof(dht22_handle_t));
    if (handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for DHT22 handle");
        return NULL;
    }
    handle->gpio = gpio;
    handle->humidity = 0.0f;
    handle->temperature = 0.0f;
    gpio_set_direction(handle->gpio, GPIO_MODE_INPUT_OUTPUT);
    return handle;
}

// == error handler ===============================================

void errorHandler(int response)
{
	switch(response) {
	
		case DHT_TIMEOUT_ERROR :
			ESP_LOGE( TAG, "Sensor Timeout\n" );
			break;

		case DHT_CHECKSUM_ERROR:
			ESP_LOGE( TAG, "CheckSum error\n" );
			break;

		case DHT_OK:
			break;

		default :
			ESP_LOGE( TAG, "Unknown error\n" );
	}
}

/*-------------------------------------------------------------------------------
;
;	get next state 
;
;	I don't like this logic. It needs some interrupt blocking / priority
;	to ensure it runs in realtime.
;
;--------------------------------------------------------------------------------*/

static int getSignalLevel( int gpio, int usTimeOut, bool state )
{

	int uSec = 0;
	while( gpio_get_level(gpio)==state ) {

		if( uSec > usTimeOut ) 
			return -1;
		
		++uSec;
		esp_rom_delay_us(1);		// uSec delay
	}
	
	return uSec;
}

/*----------------------------------------------------------------------------
;
;	read DHT22 sensor
copy/paste from AM2302/DHT22 Docu:
DATA: Hum = 16 bits, Temp = 16 Bits, check-sum = 8 Bits
Example: MCU has received 40 bits data from AM2302 as
0000 0010 1000 1100 0000 0001 0101 1111 1110 1110
16 bits RH data + 16 bits T data + check sum
1) we convert 16 bits RH data from binary system to decimal system, 0000 0010 1000 1100 → 652
Binary system Decimal system: RH=652/10=65.2%RH
2) we convert 16 bits T data from binary system to decimal system, 0000 0001 0101 1111 → 351
Binary system Decimal system: T=351/10=35.1°C
When highest bit of temperature is 1, it means the temperature is below 0 degree Celsius. 
Example: 1000 0000 0110 0101, T= minus 10.1°C: 16 bits T data
3) Check Sum=0000 0010+1000 1100+0000 0001+0101 1111=1110 1110 Check-sum=the last 8 bits of Sum=11101110
Signal & Timings:
The interval of whole process must be beyond 2 seconds.
To request data from DHT:
1) Sent low pulse for > 1~10 ms (MILI SEC)
2) Sent high pulse for > 20~40 us (Micros).
3) When DHT detects the start signal, it will pull low the bus 80us as response signal, 
   then the DHT pulls up 80us for preparation to send data.
4) When DHT is sending data to MCU, every bit's transmission begin with low-voltage-level that last 50us, 
   the following high-voltage-level signal's length decide the bit is "1" or "0".
	0: 26~28 us
	1: 70 us
;----------------------------------------------------------------------------*/

#define MAXdhtData 5	// to complete 40 = 5*8 Bits

int readDHT(dht22_handle_t* handle)
{
	if (handle == NULL) {
		ESP_LOGE(TAG, "DHT22 handle is NULL");
		return DHT_TIMEOUT_ERROR; // Or some other error
	}

	// -- We are in a critical section, we can't use ESP_LOG in this section --
	// -- Timing is critical, so we disable interrupts to prevent task switching.
	portMUX_TYPE dht_mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&dht_mux);

int uSec = 0;

uint8_t dhtData[MAXdhtData];
uint8_t byteInx = 0;
uint8_t bitInx = 7;

	for (int k = 0; k<MAXdhtData; k++) 
		dhtData[k] = 0;

	// == Send start signal to DHT sensor ===========

	gpio_set_direction( handle->gpio, GPIO_MODE_OUTPUT );

	// pull down for 3 ms for a smooth and nice wake up 
	gpio_set_level( handle->gpio, 0 );
	esp_rom_delay_us( 3000 );			

	// pull up for 25 us for a gentile asking for data
	gpio_set_level( handle->gpio, 1 );
	esp_rom_delay_us( 25 );

	gpio_set_direction( handle->gpio, GPIO_MODE_INPUT );		// change to input mode
  
	// == DHT will keep the line low for 80 us and then high for 80us ====

	uSec = getSignalLevel( handle->gpio, 85, 0 );
	if( uSec<0 ) {
		portEXIT_CRITICAL(&dht_mux);
		return DHT_TIMEOUT_ERROR;
	}

	// -- 80us up ------------------------

	uSec = getSignalLevel( handle->gpio, 85, 1 );
	if( uSec<0 ) {
		portEXIT_CRITICAL(&dht_mux);
		return DHT_TIMEOUT_ERROR;
	}

	// == No errors, read the 40 data bits ================
  
	for( int k = 0; k < 40; k++ ) {

		// -- starts new data transmission with >50us low signal

		uSec = getSignalLevel( handle->gpio, 56, 0 );
		if( uSec<0 ) {
			portEXIT_CRITICAL(&dht_mux);
			return DHT_TIMEOUT_ERROR;
		}

		// -- check to see if after >70us rx data is a 0 or a 1

		uSec = getSignalLevel( handle->gpio, 75, 1 );
		if( uSec<0 ) {
			portEXIT_CRITICAL(&dht_mux);
			return DHT_TIMEOUT_ERROR;
		}

		// add the current read to the output data
		// since all dhtData array where set to 0 at the start, 
		// only look for "1" (>28us us)
	
		if (uSec > 40) {
			dhtData[ byteInx ] |= (1 << bitInx);
			}
	
		// index to next byte

		if (bitInx == 0) { bitInx = 7; ++byteInx; }
		else bitInx--;
	}

	portEXIT_CRITICAL(&dht_mux);

	// == get humidity from Data[0] and Data[1] ==========================

	handle->humidity = dhtData[0];
	handle->humidity *= 0x100;					// >> 8
	handle->humidity += dhtData[1];
	handle->humidity /= 10;						// get the decimal

	// == get temp from Data[2] and Data[3]
	
	handle->temperature = dhtData[2] & 0x7F;	
	handle->temperature *= 0x100;				// >> 8
	handle->temperature += dhtData[3];
	handle->temperature /= 10;

	if( dhtData[2] & 0x80 ) 			// negative temp, brrr it's freezing
		handle->temperature *= -1;


	// == verify if checksum is ok ===========================================
	// Checksum is the sum of Data 8 bits masked out 0xFF
	
	if (dhtData[4] == ((dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3]) & 0xFF)) 
		return DHT_OK;

	else 
		return DHT_CHECKSUM_ERROR;
}

// == get temp & hum =============================================

float getHumidity(const dht22_handle_t* handle) { return handle ? handle->humidity : -1.0f; }
float getTemperature(const dht22_handle_t* handle) { return handle ? handle->temperature : -1.0f; }