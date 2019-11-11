/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UBLOX_GPS_H
#define __UBLOX_GPS_H

#ifdef __cplusplus
extern "C" {
#endif


//#include "stm32f4xx_hal_spi.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>


/*Definitions here*/
#define DEBUG 1				//use printf
#define TOTAL_BUFFER_LENGTH 600
#define NMEA_MAX_LENGTH 80
#define PKT_RESET 0
#define PKT_SET 1


/* USER CODE END Includes */

typedef enum {
  gpsNoFixState 		 = 0x00,			/**< \brief GPSS is in 'No Fix' state. */
  gpsFixState,								/**< \brief GPSS is in 'Fix' state. */
  gps2DFixState,							/**< \brief GPSS is in '2D Fix' state. */
  gps3DFixState,							/**< \brief GPSS is in '3D Fix' state. */
  gpsDiffFixState,							/**< \brief GPSS is in 'Differential Fix' state. */
}GPSS_StateVariableTypeDef;


enum nmea_sentence_id
{
    NMEA_INVALID = -1,
    NMEA_UNKNOWN = 0,
    NMEA_SENTENCE_RMC,
	NMEA_SENTENCE_GNS,
	NMEA_SENTENCE_GSV,
	NMEA_SENTENCE_GST,
	NMEA_SENTENCE_GBS,
	NMEA_SENTENCE_GSA,
//		NMEA_SENTENCE_GSA12,
//		NMEA_SENTENCE_GSA16,
		NMEA_SENTENCE_ZDA,
    NMEA_SENTENCE_GGA,
    NMEA_SENTENCE_GLL,
	NMEA_SENTENCE_VTG
};



struct nmea_float {
    int_least32_t value;
    int_least32_t scale;
};


struct nmea_date {
    uint8_t day;
    uint8_t month;
    uint8_t year;
};



struct nmea_time {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t milliseconds;
};


/**************************************RMC SENTENCE*******************************************************/
struct nmea_sentence_rmc {
    struct nmea_time time;   //UTC time
//    bool valid;
		uint8_t status;				//Status of data rcvd
    struct nmea_float latitude;
//		char lat_dir;
    struct nmea_float longitude;
//		char long_dir;
    struct nmea_float speed;   //speed over gnd in knots
    struct nmea_float course;  //track made good, degrees true
    struct nmea_date date;
//    struct nmea_float variation;  //magnetic variation not supported
		uint8_t mode;
		uint8_t nav_status;   //always 'V'
};


/**************************************GSV SENTENCE*******************************************************/
struct nmea_sat_info {
    uint8_t nr;   //satellite number
    uint8_t elevation;
    uint16_t azimuth;
    int16_t snr;
};

//modified
struct nmea_sentence_gsv {        //check for talker ID
    uint8_t total_msgs;
    uint8_t msg_nr;
    uint8_t sats_in_view;
    struct nmea_sat_info sats[4];
		uint8_t signal_id;   //always equal to 1
};



/**************************************GSA SENTENCE*******************************************************/

struct nmea_sentence_gsa {
    uint8_t mode;    //selection mode
    uint8_t fix_type;
    uint8_t sats[12];  //IDs of sats used for Fix - default value 12 satellites
    struct nmea_float pdop;
    struct nmea_float hdop;
    struct nmea_float vdop;
		uint8_t gnss_id;
};


/**************************************GGA SENTENCE*******************************************************/

struct nmea_sentence_gga {
    struct nmea_time time;
    struct nmea_float latitude;
//		char lat_dir;
    struct nmea_float longitude;
//		char long_dir;
    uint8_t fix_quality;   //from enum
    uint8_t satellites_tracked;
    struct nmea_float hdop;
    struct nmea_float altitude;   //antenna altitude
		uint8_t altitude_units;
    struct nmea_float height;   //geoid separation
		uint8_t height_units;
//    int dgps_age;   not supported in furuno
};

/**************************************GLL SENTENCE*******************************************************/

struct nmea_sentence_gll {
    struct nmea_float latitude;
//		char lat_dir;
    struct nmea_float longitude;
//		char long_dir;
    struct nmea_time time;
    uint8_t status;
    uint8_t mode;
};


/**************************************VTG SENTENCE*******************************************************/

struct nmea_sentence_vtg {
    struct nmea_float true_track_degrees;
//		char deg_true;
//    struct nmea_float magnetic_track_degrees;
		char deg_magnetic;
    struct nmea_float speed_knots;   //speed over ground
//		char knots;
    struct nmea_float speed_kph;		 //speed over ground
//		char kmph;
//    enum nmea_faa_mode faa_mode;
		uint8_t mode;  //from enum
};



/**************************************PARSED PACKET******************************************************/


typedef struct{
	uint8_t date;
	uint8_t month;
	uint8_t year;
}gpsDateStampTypeDef;

typedef struct{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint16_t millisecs;
}gpsTimeStampTypeDef;



typedef struct{
	gpsDateStampTypeDef date;
	gpsTimeStampTypeDef time;
}dateTimeStampParamTypeDef;


typedef struct{
//	uint16_t paramID;
	uint32_t value;
}longitudeParamTypeDef;


typedef struct{
//	uint16_t paramID;
	uint16_t value;
}altitudeParamTypeDef;


typedef struct{
//	uint16_t paramID;
	uint8_t value;
}satsInUseParamTypeDef;


typedef enum
{
  DATA_INVALID       = 0x00,				//from RMC
  DATA_VALID,
}fixValidityTypeDef;


typedef union {
	uint8_t fixParameter;
	struct {
		uint8_t fixValidity : 1;		//from RMC
	 	uint8_t fixQualityIndication : 3;		//from GGA
	 	uint8_t fixStatus : 2;			//fix mode from GSA
	 	uint8_t reserved : 2;
	 } fixBits;
} fixParamTypeDef;



typedef struct{
//	uint16_t paramID;
	uint8_t value;
}pDopParamTypeDef;


typedef struct{
//	uint16_t paramID;
	uint8_t value;
}hDopParamTypeDef;


typedef struct{
//	uint16_t paramID;
	uint8_t value;
}vDopParamTypeDef;

typedef struct{
//	uint16_t paramID;
	uint16_t value;
}headingParamTypeDef;

typedef struct{
//	uint16_t paramID;
	uint16_t value;
}speedParamTypeDef;



//final packet structure
typedef struct {
//	dateTimeStampParamTypeDef dateTime;			//key value pair of date time stamp
	float currentLatitude;
	float currentLongitude;
	float currentAltitude;
	fixParamTypeDef fixParamters;
	float pDOP;
	float hDOP;
	float vDOP;
	float magneticHeading;
	float groundSpeedKmph;
} gpsParsedPacketTypeDef;



/*Function prototypes*/
void gpsSelect();
void gpsDeselect();
void GPS_ReceiveRawPacket(SPI_HandleTypeDef * spiHandle);
gpsParsedPacketTypeDef * GPS_ProcessRawPacket(void);
void separatePackets(void);
void parseReceivedPackets(uint8_t * sentenceToParse);
static inline bool nmea_isfield(char c);
static int hex2int(char c);



/**
 * Calculate raw sentence checksum. Does not check sentence integrity.
 */
uint8_t nmea_checksum(const char *sentence);

uint16_t gpsStrCpyCh(uint8_t *StrSrc, uint8_t *StrDest, uint8_t Char);

/**
 * Check sentence validity and checksum. Returns true for valid sentences.
 */
bool nmea_check(const char *sentence, bool strict);

/**
 * Determine talker identifier.
 */
bool nmea_talker_id(char talker[3], const char *sentence);

/**
 * Determine sentence identifier.
 */
enum nmea_sentence_id nmea_sentence_id(const char *sentence, bool strict);

/**
 * Scanf-like processor for NMEA sentences. Supports the following formats:
 * c - single character (char *)
 * d - direction, returned as 1/-1, default 0 (int *)
 * f - fractional, returned as value + scale (int *, int *)
 * i - decimal, default zero (int *)
 * s - string (char *)
 * t - talker identifier and type (char *)
 * T - date/time stamp (int *, int *, int *)
 * Returns true on success. See library source code for details.
 */
bool nmea_scan(const char *sentence, const char *format, ...);

/*
 * Parse a specific type of sentence. Return true on success.
 */
bool nmea_parse_rmc(struct nmea_sentence_rmc *frame, const char *sentence);
bool nmea_parse_gsa(struct nmea_sentence_gsa *frame, const char *sentence);
bool nmea_parse_gga(struct nmea_sentence_gga *frame, const char *sentence);
bool nmea_parse_vtg(struct nmea_sentence_vtg *frame, const char *sentence);



























#ifdef __cplusplus
}
#endif

#endif

