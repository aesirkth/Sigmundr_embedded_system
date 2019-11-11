/*
 * ublox_gps.c
 *
 * Created on: Oct 7, 2019
 * Author: Sonal Shrivastava
 */

/*Includes here*/
//#include "../../../GPS_Testing/Inc/ublox_gps.h"
#include "ublox_gps.h"
#include <math.h>




/*Global variables*/
uint8_t copyBuffer[TOTAL_BUFFER_LENGTH] = {0};

uint8_t sentence1[NMEA_MAX_LENGTH] = {0};
uint8_t sentence2[NMEA_MAX_LENGTH] = {0};
uint8_t sentence3[NMEA_MAX_LENGTH] = {0};
uint8_t sentence4[NMEA_MAX_LENGTH] = {0};
uint8_t sentence5[NMEA_MAX_LENGTH] = {0};
uint8_t sentence6[NMEA_MAX_LENGTH] = {0};
uint8_t sentence7[NMEA_MAX_LENGTH] = {0};
uint8_t sentence8[NMEA_MAX_LENGTH] = {0};
uint8_t sentence9[NMEA_MAX_LENGTH] = {0};
uint8_t sentence10[NMEA_MAX_LENGTH] = {0};
uint8_t sentence11[NMEA_MAX_LENGTH] = {0};

struct nmea_sentence_rmc framermc;
struct nmea_sentence_gga framegga;
struct nmea_sentence_gsa framegsa;
struct nmea_sentence_vtg framevtg;

gpsParsedPacketTypeDef parsedPacketData;
float newLatitude;
float newLongitude;
float newAltitude;

//extern RTC_HandleTypeDef hrtc;
//RTC_DateTypeDef rtcDate;
//RTC_TimeTypeDef rtcTime;

/* USER CODE END Includes */


/* Variables -----------------------------------------------------------------*/
uint8_t value = 0;  		//made global for debugging

// data buffer
static uint8_t gpsbuf[TOTAL_BUFFER_LENGTH] = {0};


/*Function definitions*/
void gpsSelect()
{
	//Hold Chip select pin for GPS low to select the chip
	HAL_GPIO_WritePin(NSS_GPS_GPIO_Port, NSS_GPS_Pin, GPIO_PIN_RESET);
}


void gpsDeselect()
{
	//Hold Chip select pin for GPS low to select the chip
	HAL_GPIO_WritePin(NSS_GPS_GPIO_Port, NSS_GPS_Pin, GPIO_PIN_SET);
}


void GPS_ReceiveRawPacket(SPI_HandleTypeDef * spiHandle)
{
	if(HAL_SPI_TransmitReceive(spiHandle, 255, &gpsbuf, (uint16_t)TOTAL_BUFFER_LENGTH, 100) == HAL_OK)
	{
//		//all packets are received here
//		memset(copyBuffer, 0, sizeof(copyBuffer));
//		//now copy the buffer
//		for(int i = 0; i < TOTAL_BUFFER_LENGTH; i++)
//		{
//			copyBuffer[i] = gpsbuf[i];
//		}
//		//check first ten bytes to prevent parsing unnecessarily in case there is some garbage present in the buffer
//		for(int i = 0; i < 20; i++)
//		{
//			if(copyBuffer[i] == '$')
//			{
//				parsePacketsNow = true;
//				break;
//			}
//		}
//		if(parsePacketsNow == true)
//		{
//			uint8_t sizeOfParsedPacketStruct = sizeof(parsedPacketData);
//			//call this function after every chunk of packets
//			separatePackets();
//		}
	}
	else
	{
		printf("Error receiving bytes!");
	}
	//return &gpsbuf[0];
}



gpsParsedPacketTypeDef * GPS_ProcessRawPacket(void)
{
	uint8_t sizeOfParsedPacketStruct;
	uint8_t parsePacketsNow = false;

	//all packets are received here
	memset(copyBuffer, 0, sizeof(copyBuffer));
	//now copy the buffer
	for(int i = 0; i < TOTAL_BUFFER_LENGTH; i++)
	{
		copyBuffer[i] = gpsbuf[i];
	}
	//check first ten bytes to prevent parsing unnecessarily in case there is some garbage present in the buffer
	for(int i = 0; i < 20; i++)
	{
		if(copyBuffer[i] == '$')
		{
			parsePacketsNow = true;
			break;
		}
	}
	if(parsePacketsNow == true)
	{
		sizeOfParsedPacketStruct = sizeof(parsedPacketData);
		//call this function after every chunk of packets
		separatePackets();
	}
	return &parsedPacketData;
}




void separatePackets(void)
{
	uint8_t receivedPacketCount = 0;
	//clear the storage
	memset(&parsedPacketData, 0, sizeof(parsedPacketData));

	for(int i = 0; i < TOTAL_BUFFER_LENGTH; i++)
	{
		if(copyBuffer[i] == '$')
		{
			if(receivedPacketCount == 0)
			{
				//sentence1 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence1[0], '\r');
				//new packet started
				receivedPacketCount = 1;
				parseReceivedPackets(&sentence1[0]);
			}
			else if(receivedPacketCount == 1)
			{
//				sentence2 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence2[0], '\r');
				//new packet started
				receivedPacketCount = 2;
				parseReceivedPackets(&sentence2[0]);
			}
			else if(receivedPacketCount == 2)
			{
//				sentence3 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence3[0], '\r');
				//new packet started
				receivedPacketCount = 3;
				parseReceivedPackets(&sentence3[0]);
			}
			else if(receivedPacketCount == 3)
			{
//				sentence4 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence4[0], '\r');
				//new packet started
				receivedPacketCount = 4;
				parseReceivedPackets(&sentence4[0]);
			}
			else if(receivedPacketCount == 4)
			{
//				sentence5 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence5[0], '\r');
				//new packet started
				receivedPacketCount = 5;
				parseReceivedPackets(&sentence5[0]);
			}
			else if(receivedPacketCount == 5)
			{
//				sentence6 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence6[0], '\r');
				//new packet started
				receivedPacketCount = 6;
				parseReceivedPackets(&sentence6[0]);
			}
			else if(receivedPacketCount == 6)
			{
//				sentence7 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence7[0], '\r');
				//new packet started
				receivedPacketCount = 7;
				parseReceivedPackets(&sentence7[0]);
			}
			else if(receivedPacketCount == 7)
			{
//				sentence7 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence8[0], '\r');
				//new packet started
				receivedPacketCount = 8;
				parseReceivedPackets(&sentence8[0]);
			}
			else if(receivedPacketCount == 8)
			{
//				sentence7 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence9[0], '\r');
				//new packet started
				receivedPacketCount = 9;
				parseReceivedPackets(&sentence9[0]);
			}
			else if(receivedPacketCount == 9)
			{
//				sentence7 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence10[0], '\r');
				//new packet started
				receivedPacketCount = 10;
				parseReceivedPackets(&sentence10[0]);
			}
			else if(receivedPacketCount == 10)
			{
//				sentence7 = &copyBuffer[i];
				i += gpsStrCpyCh(&copyBuffer[i], &sentence11[0], '\r');
				//new packet started
				receivedPacketCount = 11;
				parseReceivedPackets(&sentence11[0]);
			}
			else
			{
				printf("separatePackets: Error or no packets remaining in the buffer!");
			}
		}
	}

	//now that we have separate packets, we can extract relevant info
	storeParsedData();

}


void storeParsedData(void)
{
//	/* Get the RTC current Time */
//	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
//	/* Get the RTC current Date */
//	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
//
//	//assigning date from RTC
//	parsedPacketData.dateTime.date.date = rtcDate.Date;
//	parsedPacketData.dateTime.date.month = rtcDate.Month;
//	parsedPacketData.dateTime.date.year = rtcDate.Year;
//	printf("RTC Date : %u.%u.%u", parsedPacketData.dateTime.date.date, parsedPacketData.dateTime.date.month, parsedPacketData.dateTime.date.year);
//
//	//assigning time from RTC
//	parsedPacketData.dateTime.time.hours = rtcTime.Hours;
//	parsedPacketData.dateTime.time.minutes = rtcTime.Minutes;
//	parsedPacketData.dateTime.time.seconds = rtcTime.Seconds;
//	//			  parsedPacketData.dateTime.value.time.millisecs = time.SubSeconds;
//	parsedPacketData.dateTime.time.millisecs = 999 - (rtcTime.SubSeconds * 999 / rtcTime.SecondFraction);
//	printf("RTC Time : %u : %u : %u.%u", parsedPacketData.dateTime.time.hours, parsedPacketData.dateTime.time.minutes, parsedPacketData.dateTime.time.seconds, parsedPacketData.dateTime.time.millisecs);

	//assigning Latitude from RMC
	float latValue = (float)framermc.latitude.value;
	float latScale = (float)framermc.latitude.scale;

	//stored in fix-point format
	//			  parsedPacketData.currentLatitude.value = roundf(100000 * (trunc(latValue/(latScale*100)) + ((((latValue/(latScale*100)) - (trunc(latValue/(latScale*100)))) * 100)/60)));
//	newLatitude = roundf(100000 * (trunc(latValue/(latScale*100)) + ((((latValue/(latScale*100)) - (trunc(latValue/(latScale*100)))) * 100)/60)));
	newLatitude = latValue/latScale;
	parsedPacketData.currentLatitude = newLatitude;

	//trunc(value/(scale*100)) + ((((value/(scale*100)) - (trunc(value/(scale*100)))) * 100)/60);
	printf("RMC Lat : %u / 100000", parsedPacketData.currentLatitude);

	//assigning Longitude from RMC
	float longValue = (float)framermc.longitude.value;
	float longScale = (float)framermc.longitude.scale;

	//stored in fix-point format
	//			  parsedPacketData.currentLongitude.value = roundf(100000 * (trunc(longValue/(longScale*100)) + ((((longValue/(longScale*100)) - (trunc(longValue/(longScale*100)))) * 100)/60)));
//	newLongitude = roundf(100000 * (trunc(longValue/(longScale*100)) + ((((longValue/(longScale*100)) - (trunc(longValue/(longScale*100)))) * 100)/60)));
	newLongitude = longValue/longScale;
	parsedPacketData.currentLongitude = newLongitude;

	printf("RMC Long : %u / 100000", parsedPacketData.currentLongitude);

	//from RMC
	if(framermc.status == 'V')
	{
		parsedPacketData.fixParamters.fixBits.fixValidity = DATA_INVALID;
		printf("Fix Parameters . Fix validity from RMC: %u", parsedPacketData.fixParamters.fixBits.fixValidity);

	}
	if(framermc.status == 'A')
	{
		parsedPacketData.fixParamters.fixBits.fixValidity = DATA_VALID;
		printf("Fix Parameters . Fix validity from RMC: %u", parsedPacketData.fixParamters.fixBits.fixValidity);

	}
	else
	{
		printf("RMC status wrong");  //when 'N' comes
	}

	//assigning altitude from GGA
	float altValue = (float)framegga.altitude.value;
	float altScale = (float)framegga.altitude.scale;

	//stored in fix-point format
//	newAltitude = roundf(100000 * (trunc(altValue/(altScale*100)) + ((((altValue/(altScale*100)) - (trunc(altValue/(altScale*100)))) * 100)/60)));
	newAltitude = altValue/altScale;
	parsedPacketData.currentAltitude = newAltitude;
	printf("GGA Altitude : %u / 100000", parsedPacketData.currentAltitude);

	//assigning GNSS fix quality indication from GGA
	parsedPacketData.fixParamters.fixBits.fixQualityIndication = framegga.fix_quality;
	printf("Fix Parameters . Fix quality indication from GGA: %u", parsedPacketData.fixParamters.fixBits.fixQualityIndication);

	//from GSA
	parsedPacketData.fixParamters.fixBits.fixStatus = framegsa.fix_type;
	printf("Fix Parameters . Fix Type from GSA: %u (%u)", parsedPacketData.fixParamters.fixBits.fixStatus);

	//assigning PDOP from GSA
	parsedPacketData.pDOP = (float)(framegsa.pdop.value)/(framegsa.pdop.scale);
	printf("PDOP: %u (%u)", parsedPacketData.pDOP);

	//assigning HDOP from GSA
	parsedPacketData.hDOP = (float)(framegsa.hdop.value)/(framegsa.hdop.scale);
	printf("HDOP: %u (%u)", parsedPacketData.hDOP);

	//assigning VDOP from GSA
	parsedPacketData.vDOP = (float)(framegsa.vdop.value)/(framegsa.vdop.scale);
	printf("VDOP: %u (%u)", parsedPacketData.vDOP);

	//assigning heading from VTG
	parsedPacketData.magneticHeading = (float)(framevtg.true_track_degrees.value)/(framevtg.true_track_degrees.scale);
	printf("Heading: %u (%u)", parsedPacketData.magneticHeading);

	//assigning ground speed in kmph from VTG
	parsedPacketData.groundSpeedKmph = (float)(framevtg.speed_kph.value)/(framevtg.speed_kph.scale);
	printf("Speed (Kmph): %u (%u)", parsedPacketData.groundSpeedKmph);
}




void parseReceivedPackets(uint8_t * sentenceToParse)
{
	switch(nmea_sentence_id(sentenceToParse, false))
	{
		case NMEA_SENTENCE_RMC:
		{
			printf("NMEA Packet: %s", sentenceToParse);
			if (nmea_parse_rmc(&framermc, sentenceToParse))
			{
				printf("RMC data stored");
			}
			else
			{
				printf("$xxRMC sentence is not parsed\n");
			}

		}
		break;

		case NMEA_SENTENCE_GGA:
		{
			if (nmea_parse_gga(&framegga, sentenceToParse))
			{
				printf("GGA data stored");
			}
			else
			{
				printf("$xxGGA sentence is not parsed\n");
			}
		}
		break;

		case NMEA_SENTENCE_VTG:
		{
			if (nmea_parse_vtg(&framevtg, sentenceToParse))
			{
				printf("VTG data stored");
			}
			else
			{
				printf("$xxVTG sentence is not parsed\n");
			}
		}
		break;

		case NMEA_SENTENCE_GSA:
		{
			if(nmea_parse_gsa(&framegsa, sentenceToParse))
			{
				printf("GSA data stored");
			}
			else
			{
				printf("$xxGSA sentence is not parsed\n");
			}
		}
		break;

		default:
			printf("Received some other unknown packet");
		break;
	}

//	if ((framegsa.fix_type == 2 || framegsa.fix_type == 3)
//			&& (framermc.status == 'A')
//			&& (framegga.fix_quality != 0)
////			&& (framegns.sat_mode != 'NNN')
//	)
//	{
//		printf("Got FIX %d, %c, %d, %s", framegsa.fix_type, framermc.status, framegga.fix_quality);
//	}
//	else
//	{
//		printf("waiting for fix :( ");
//	}
}




/*Copy the string until the char is encountered*/
/**
  * @brief  Copy the string until the char is encountered
  * @param  StrSrc Pointer to the source string
  * @param  StrDest Pointer to the destination string
  * @param  Char Character until which the characters are to be copied.
  * @note   This function copies the characters from Src String to Dest String, until character 'char' is found.
  * @retval Index count
  */
uint16_t gpsStrCpyCh(uint8_t *StrSrc, uint8_t *StrDest, uint8_t Char)
{
	uint8_t Index = 0;

	//copy into string until the character is encountered
	while(StrSrc[Index] != Char)
	{
		StrDest[Index] = StrSrc[Index];
		Index++;
	}
	return Index;
}



bool nmea_check(const char *sentence, bool strict)
{
	uint8_t checksum = 0x00;

	// Sequence length is limited.
	if (strlen(sentence) > NMEA_MAX_LENGTH + 3)
		return false;

	// A valid sentence starts with "$".
	if (*sentence++ != '$')
		return false;

	// The optional checksum is an XOR of all bytes between "$" and "*" & checks whether it is printable or not by isprint
	while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
		checksum ^= *sentence++;

	// If checksum is present...
	if (*sentence == '*')
	{
		// Extract checksum.
		sentence++;
		int upper = hex2int(*sentence++);    //upper byte of checksum
		if (upper == -1)
			return false;
		int lower = hex2int(*sentence++);    //lower byte of checksum
		if (lower == -1)
			return false;
		int expected = upper << 4 | lower;

		// Check for checksum mismatch.
		if (checksum != expected)
			return false;
	}
	else if (strict)
	{
		// Discard non-checksummed frames in strict mode.
		return false;
	}

	// The only stuff allowed at this point is a newline.
	if (*sentence && strcmp(sentence, "\n") && strcmp(sentence, "\r\n"))
		return false;

	return true;
}



bool nmea_talker_id(char talker[3], const char *sentence)  //talker ID either GP or GL or GN
{
	char type[6];
	if (!nmea_scan(sentence, "t", type))
		return false;

	talker[0] = type[0];
	talker[1] = type[1];
	talker[2] = '\0';

	return true;
}


static inline bool nmea_isfield(char c) {
	return isprint((unsigned char) c) && c != ',' && c != '*';
}



static int hex2int(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return -1;
}


//returns the ID of the nmea sentence
enum nmea_sentence_id nmea_sentence_id(const char *sentence, bool strict)
{
//	if (!nmea_check(sentence, strict))
//		return NMEA_INVALID;

	char type[6];
	if (!nmea_scan(sentence, "t", type))
		return NMEA_INVALID;

	if (!strcmp(type+2, "RMC"))
		return NMEA_SENTENCE_RMC;
	if (!strcmp(type+2, "GNS"))
		return NMEA_SENTENCE_GNS;
	if (!strcmp(type+2, "GSV"))
		return NMEA_SENTENCE_GSV;
	if (!strcmp(type+2, "GST"))
		return NMEA_SENTENCE_GST;
	if (!strcmp(type+2, "GBS"))
		return NMEA_SENTENCE_GBS;
	if (!strcmp(type+2, "GSA"))
		return NMEA_SENTENCE_GSA;
	if (!strcmp(type+2, "ZDA"))
		return NMEA_SENTENCE_ZDA;
	if (!strcmp(type+2, "GGA"))
		return NMEA_SENTENCE_GGA;
	if (!strcmp(type+2, "GLL"))
		return NMEA_SENTENCE_GLL;
	if (!strcmp(type+2, "VTG"))
		return NMEA_SENTENCE_VTG;

	return NMEA_UNKNOWN;   //for any other sentence
}



bool nmea_scan(const char *sentence, const char *format, ...)
{
	bool result = false;
	bool optional = false;
	va_list ap;   //variable list object
	va_start(ap, format);

	const char *field = sentence;
#define next_field() \
		do { \
			/* Progress to the next field. */ \
		while (nmea_isfield(*sentence)) \
		sentence++; \
		/* Make sure there is a field there. */ \
		if (*sentence == ',') { \
			sentence++; \
			field = sentence; \
		} else { \
			field = NULL; \
		} \
		} while (0)

	while (*format) {
		char type = *format++;

		if (type == ';') {
			// All further fields are optional.
			optional = true;
			continue;
		}

		if (!field && !optional) {
			// Field requested but we ran out of input. Bail out.
			goto parse_error;
		}

		switch (type) {
		case 'c': { // Single character field (char).
			//                char value = '\0';   //initialize the char value with null character
			value = 0;
			if (field && nmea_isfield(*field))
				value = *field;

			*va_arg(ap, char *) = value;
			//								*va_arg(ap, uint8_t *) = value;
		} break;

		case 'd': { // Single character direction field (int).
			int value = 0;
			if (field && nmea_isfield(*field))
			{
				switch (*field)
				{
				case 'N':
				case 'E': value = 1;
				break;

				case 'S':
				case 'W':	value = -1;
				break;

				default:	goto parse_error;
				}
			}

			*va_arg(ap, int *) = value;

		} break;

		case 'f': { // Fractional value with scale (refer struct nmea_float).
			int sign = 0;
			int_least32_t value = -1;
			int_least32_t scale = 0;

			if (field)
			{
				while (nmea_isfield(*field))
				{
					if (*field == '+' && !sign && value == -1)
					{
						sign = 1;
					}
					else if (*field == '-' && !sign && value == -1)
					{
						sign = -1;
					}
					else if (isdigit((unsigned char) *field))
					{
						int digit = *field - '0';
						if (value == -1)
							value = 0;
						if (value > (INT_LEAST32_MAX-digit) / 10)
						{
							/* we ran out of bits, what do we do? */
							if (scale)
							{
								/* truncate extra precision */
								break;
							}
							else
							{
								/* integer overflow. bail out. */
								goto parse_error;
							}
						}
						value = (10 * value) + digit;
						if (scale)
							scale *= 10;
					}
					else if (*field == '.' && scale == 0)
					{
						scale = 1;
					}
					else if (*field == ' ')
					{
						/* Allow spaces at the start of the field. Not NMEA
						 * conformant, but some modules do this. */
						if (sign != 0 || value != -1 || scale != 0)
							//	                            if (value != -1 || scale != 0)
							goto parse_error;
					}
					else
					{
						goto parse_error;
					}
					field++;
				}
			}

			if ((sign || scale) && value == -1)
				goto parse_error;

			if (value == -1) {
				/* No digits were scanned. */
				value = 0;
				scale = 0;
			} else if (scale == 0) {
				/* No decimal point. */
				scale = 1;
			}
			if (sign)
				value *= sign;

			*va_arg(ap, struct nmea_float *) = (struct nmea_float) {value, scale};
		} break;

		case 'i': { // Integer value, default 0 (int).
			int value = 0;

			if (field) {
				char *endptr;
				value = strtol(field, &endptr, 10);
				if (nmea_isfield(*endptr))
					goto parse_error;
			}

			*va_arg(ap, uint8_t *) = (uint8_t)value;
		} break;

		case 'y': { // year in ZDA, default 0 (int).
			int value = 0;

			if (field) {
				char *endptr;
				value = strtol(field, &endptr, 10);
				if (nmea_isfield(*endptr))
					goto parse_error;
			}

			*va_arg(ap, uint16_t *) = (uint16_t)value;
		} break;

		case 's': { // String value (char *).
			//                char *buf = va_arg(ap, char *);
			unsigned char *buf = va_arg(ap, uint8_t *);

			if (field) {
				while (nmea_isfield(*field))
					*buf++ = *field++;
			}

			*buf = '\0';  //last char, stored as string!
		} break;

		case 't': { // NMEA talker+sentence identifier (char *).
			// This field is always mandatory.
			if (!field)
				goto parse_error;

			if (field[0] != '$')
				goto parse_error;
			for (int f=0; f<5; f++)
				if (!nmea_isfield(field[1+f]))
					goto parse_error;

			char *buf = va_arg(ap, char *);  //created a pointer to that arg where it is to b stored, here it is 'type'
			memcpy(buf, field+1, 5);
			buf[5] = '\0';
		} break;

		case 'D': { // Date (int, int, int), -1 if empty.
			struct nmea_date *date = va_arg(ap, struct nmea_date *);

			int d = -1, m = -1, y = -1;

			if (field && nmea_isfield(*field)) {
				// Always six digits.
				for (int f=0; f<6; f++)
					if (!isdigit((unsigned char) field[f]))
						goto parse_error;

				char dArr[] = {field[0], field[1], '\0'};
				char mArr[] = {field[2], field[3], '\0'};
				char yArr[] = {field[4], field[5], '\0'};
				d = strtol(dArr, NULL, 10);
				m = strtol(mArr, NULL, 10);
				y = strtol(yArr, NULL, 10);
			}

			date->day = (uint8_t)d;
			date->month = (uint8_t)m;
			date->year = (uint8_t)y;
		} break;

		case 'T': { // Time (int, int, int, int), -1 if empty.
			struct nmea_time *time_ = va_arg(ap, struct nmea_time *);

			int h = -1, m = -1, s = -1, u = -1;

			if (field && nmea_isfield(*field)) {
				// Minimum required: integer time.
				for (int f=0; f<6; f++)
					if (!isdigit((unsigned char) field[f]))
						goto parse_error;

				char hArr[] = {field[0], field[1], '\0'};
				char mArr[] = {field[2], field[3], '\0'};
				char sArr[] = {field[4], field[5], '\0'};
				h = strtol(hArr, NULL, 10);
				m = strtol(mArr, NULL, 10);
				s = strtol(sArr, NULL, 10);
				field += 6;

				// Extra: fractional time. Saved as milliseconds.
				if (*field++ == '.') {
					int value = 0;
					//                        int scale = 1000000;  //since microsecs
					int scale = 1000;  //since millisecs
					while (isdigit((unsigned char) *field) && scale > 1) {
						value = (value * 10) + (*field++ - '0');
						scale /= 10;
					}
					//                        u = value * scale;
					u = value;
				} else {
					u = 0;
				}
			}

			//                time_->hours = h;
			//                time_->minutes = m;
			//                time_->seconds = s;
			//                time_->microseconds = u;
			time_->hours = (uint8_t)h;
			time_->minutes = (uint8_t)m;
			time_->seconds = (uint8_t)s;
			time_->milliseconds = (uint16_t)u;
		} break;

		case '_': { // Ignore the field.
		} break;

		default: { // Unknown.
			goto parse_error;
		} break;
		}

		next_field();
	}

	result = true;

	parse_error:
	va_end(ap);
	//    printf("Parse Error");
	return result;
}




bool nmea_parse_rmc(struct nmea_sentence_rmc *frame, const char *sentence)
{
	// $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62  -- with magnetic variation
	// $GPRMC,092406.800,A,3442.8211,N,13520.1148,E,0.01,353.80,230812,,,D,V*0A  -- Furuno supported rmc

	char type[6];
	//    char validity;
	int latitude_direction;
	int longitude_direction;
	//    int variation_direction;   -- for magnetic variation direction
	//    if (!nmea_scan(sentence, "tTcfdfdffDfd",  //last 'fd' is for magnetic variation

	if (!nmea_scan(sentence, "tTcfdfdffD__cc",
			type,
			&frame->time,
			//           &validity,
			&frame->status,
			&frame->latitude, &latitude_direction,
			&frame->longitude, &longitude_direction,
			&frame->speed,   //speed over gnd
			&frame->course,  //track made good, deg true
			&frame->date,
			&frame->mode,  //single char field
			&frame->nav_status))  //single char field
		//            &frame->variation, &variation_direction))  magnetic -- NOT SUPPORTED
	{
		printf("Error! RMC not parsed");
		return false;
	}
	if (strcmp(type+2, "RMC"))
	{
		printf("Not RMC sentence");
		return false;
	}

	//    frame->status = (status == 'A');
	frame->latitude.value *= latitude_direction;
	frame->longitude.value *= longitude_direction;
	//    frame->variation.value *= variation_direction;
	return true;
}



bool nmea_parse_gsa(struct nmea_sentence_gsa *frame, const char *sentence)
{
	// $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
	char type[6];

	//    if (!nmea_scan(sentence, "tciiiiiiiiiiiiifff",
	if (!nmea_scan(sentence, "tciiiiiiiiiiiiifffi",
			type,
			&frame->mode,
			&frame->fix_type,
			&frame->sats[0],				//at a time atmost 12 satellites can be tracked
			&frame->sats[1],
			&frame->sats[2],
			&frame->sats[3],
			&frame->sats[4],
			&frame->sats[5],
			&frame->sats[6],
			&frame->sats[7],
			&frame->sats[8],
			&frame->sats[9],
			&frame->sats[10],
			&frame->sats[11],
			&frame->pdop,
			&frame->hdop,
			&frame->vdop,
			&frame->gnss_id))
	{
		printf("Error! GSA not parsed");
		return false;
	}
	if (strcmp(type+2, "GSA"))
	{
		printf("Not GSA sentence");
		return false;
	}

	return true;
}


bool nmea_parse_gga(struct nmea_sentence_gga *frame, const char *sentence)
{
	// $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
	char type[6];
	int latitude_direction;
	int longitude_direction;

	//    if (!nmea_scan(sentence, "tTfdfdiiffcfci_",
	if (!nmea_scan(sentence, "tTfdfdiiffcfc",
			type,
			&frame->time,
			&frame->latitude, &latitude_direction,
			&frame->longitude, &longitude_direction,
			&frame->fix_quality,
			&frame->satellites_tracked,
			&frame->hdop,
			&frame->altitude, &frame->altitude_units,
			&frame->height, &frame->height_units))
		//            &frame->dgps_age))
	{
		printf("Error! GGA not parsed");
		return false;
	}
	if (strcmp(type+2, "GGA"))
	{
		printf("Not GGA sentence");
		return false;
	}
	frame->latitude.value *= latitude_direction;
	frame->longitude.value *= longitude_direction;

	return true;
}



bool nmea_parse_vtg(struct nmea_sentence_vtg *frame, const char *sentence)
{
	// $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
	// $GPVTG,156.1,T,140.9,M,0.0,N,0.0,K*41
	// $GPVTG,096.5,T,083.5,M,0.0,N,0.0,K,D*22
	// $GPVTG,188.36,T,,M,0.820,N,1.519,K,A*3F
	//The value can be A=autonomous, D=differential, E=Estimated, N=not valid, S=Simulator. Sometimes there can be a null value as well.
	//Only the A and D values will correspond to an Active and reliable Sentence.
	char type[6];
	char c_true, c_knots, c_magnetic, c_kph; // c_mode_indicator , ;

	//    if (!nmea_scan(sentence, "tfcfcfcfc;c",
	if (!nmea_scan(sentence, "tfc_cfcfcc",
			type,
			&frame->true_track_degrees,
			&c_true,
			//		&frame->magnetic_track_degrees,
			&c_magnetic,
			&frame->speed_knots,
			&c_knots,
			&frame->speed_kph,
			&c_kph,
			&frame->mode))
		//					&c_mode_indicator))
	{
		printf("Error! VTG not parsed");
		return false;
	}
	if (strcmp(type+2, "VTG"))
	{
		printf("Not VTG sentence");
		return false;
	}
	return true;
}



void GPS_ShutdownReceiver()
{
	//set power settings here
	gpsDeselect();
	//cut off power for gps chip
}


