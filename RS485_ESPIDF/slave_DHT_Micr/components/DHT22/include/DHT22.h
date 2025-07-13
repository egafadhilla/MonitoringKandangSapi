/* 
	DHT22 temperature sensor driver
*/

#ifndef DHT22_H_  
#define DHT22_H_

#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2

// Define a struct to hold sensor data and configuration
typedef struct {
    int gpio;
    float humidity;
    float temperature;
} dht22_handle_t;

// == function prototypes =======================================

dht22_handle_t* dht22_init(int gpio);
void 	errorHandler(int response);
int 	readDHT(dht22_handle_t* handle);
float 	getHumidity(const dht22_handle_t* handle);
float 	getTemperature(const dht22_handle_t* handle);
// This function is internal to the component, so we can remove it from the public header.
// int 	getSignalLevel( int usTimeOut, bool state );

#endif