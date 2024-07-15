#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <dirent.h>
#include <string.h>
#include <assert.h>

/* 0: use gyro and accelerometer, 1: use gyro only, 2: do not rotate craft display */
#define CRAFT_ORIENTATION_MODE 0

int debugMode = 6; /* "Gyro Scaled"; unfiltered gyro signal */
// int debugMode = 12; /* "ESC RPM"; motor frequence in dHz (decihertz) */

// Function declarations
void writeLogStartMarker( FILE* f );
void writeLogEndMarker( FILE* f );
char* unsignedVariableByte( uint32_t value );
int32_t zigZag( int32_t value );
char* signedVariableByte( int32_t value );
void writeData( FILE* f_out );

// Global variables
int iteration = 0;
int t_time = 0;
int axisP[ 3 ] = { 0, 0, 0 };
int axisI[ 3 ] = { 0, 0, 0 };
int axisD[ 2 ] = { 0, 0 };
int axisF[ 3 ] = { 0, 0, 0 };
int rcCommand[ 4 ] = { 0, 0, 0, 0 };
int setpoint[ 4 ] = { 0, 0, 0, 0 };
int vbatLatest = 0;
int amperageLatest = 0;
int rssi = 0;
int gyroADC[ 3 ] = { 0, 0, 0 };
int accSmooth[ 3 ] = { 0, 0, 0 };
int debug[ 4 ] = { 0, 0, 0, 0 };
int motor[ 4 ] = { 0, 0, 0, 0 };
int motorHz[ 4 ] = { 0, 0, 0, 0 };

void writeLogStartMarker( FILE* f )
{
	fprintf( f, "H Product:Blackbox flight data recorder by Nicholas Sherlock\n" );
	fprintf( f, "H Data version:2\n" );
	fprintf( f, "H I interval:1\n" );
	fprintf( f, "H P interval:1/1\n" );
	fprintf( f, "H Firmware type:Cleanflight\n" );
	fprintf( f, "H Firmware revision:Betaflight 4.0\n" );

	fprintf( f,
					 "H Field I "
					 "name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],"
					 "axisD[1],axisF[0],axisF[1],axisF[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3]"
					 ",setpoint[0],setpoint[1],setpoint[2],setpoint[3],vbatLatest,amperageLatest,rssi,"
					 "gyroADC[0],gyroADC[1],gyroADC[2],accSmooth[0],accSmooth[1],accSmooth[2],debug[0],debug["
					 "1],debug[2],debug[3],motor[0],motor[1],motor[2],motor[3]\n" );
	fprintf( f,
					 "H Field I "
					 "signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0\n" );
	// fprintf( f, "H Field I
	// predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0,0,11,5,5,5\n" );
	// fprintf( f, "H Field I
	// encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,3,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0\n" );
	fprintf( f,
					 "H Field I "
					 "signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0\n" );
	fprintf(
		f, "H Field I "
			 "predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n" );
	fprintf(
		f, "H Field I "
			 "encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1\n" );
	fprintf(
		f, "H Field P "
			 "predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,3,3,3\n" );
	fprintf(
		f, "H Field P "
			 "encoding:9,0,0,0,0,7,7,7,0,0,0,0,0,8,8,8,8,8,8,8,8,6,6,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n" );

	// fprintf( f, "H minthrottle:1000\n" )
	fprintf( f, "H maxthrottle:2000\n" ); // to make Plasmatree/PID-Analyzer happy
	fprintf( f, "H gyro_scale:0x3f800000\n" ); // Gyro scale 4 byte hex of IEEE float; 0x3e800000:
																						 // 1=0.25 deg/sec, 0x3f800000: 1=1 deg/sec
	fprintf( f, "H motorOutput:0,1000\n" ); // for motor range
	fprintf( f, "H acc_1G:2048\n" ); // for Accelerometer range
	// fprintf( f, "H vbat_scale:108\n" )
	fprintf( f, "H vbatcellvoltage:330,350,430\n" );
	fprintf( f, "H vbatref:420\n" ); // 1S
	// fprintf( f, "H currentSensor:0,400\n" )
	fprintf( f, "H looptime:250\n" ); // for FFT Hz scaling
	fprintf( f, "H pid_process_denom:2\n" ); // for FFT Hz scaling

	fprintf( f, "H rc_rates:213.4,213.4,213.4\n" ); // for setpoint scaling
	// fprintf(f, "H rc_expo:0,0,0\n");
	fprintf( f, "H rates:50,50,50\n" ); // for setpoint scaling
	// fprintf(f, "H rate_limits:1998,1998,1998\n");

	fprintf( f, "H rollPID:31.22,1,1\n" ); // Also to make Plasmatree/PID-Analyzer happy. Since we do
																				 // not know the
	fprintf( f, "H pitchPID:31.22,1,1\n" ); // equivalent Betaflight PID values, the response.png plot
																					// is of no use.
	fprintf( f, "H yawPID:31.22,1,0\n" ); // However the noise.png plot seems to be independent of
																				// those numbers.

	if( debugMode == 6 ) {
		fprintf( f, "H debug_mode:6\n" ); // 3 .. "Gyro Scaled"; -2000 .. 2000 deg/s
	}
	else if( debugMode == 12 ) {
		fprintf( f, "H debug_mode:12\n" ); // 12 .. "ESC RPM"; arbitrary values, but [1]..[4] use the
																			 // same scaling factor
	}
	else {
		assert( false && "debugMode has no valid value" );
	}
}

void writeLogEndMarker( FILE* f )
{
	// C equivilent of Python3 f.write( b'E\xffEnd of log\x00' )
	fprintf( f, "%s%c%s%c", "E", 0xFF, "End of log", 0x00 );
}

char* unsignedVariableByte( uint32_t value )
{
	char* encoded = malloc( 5 );
	int i = 0;
	while( value > 127 ) {
		encoded[ i++ ] = ( value | 0x80 ) & 0xFF;
		value >>= 7;
	}
	encoded[ i++ ] = value;
	encoded[ i ] = '\0';
	return encoded;
}

int32_t zigZag( int32_t value )
{
	int32_t encoded = value * 2;
	if( value < 0 ) {
		encoded = -encoded - 1;
	}
	return encoded;
}

char* signedVariableByte( int32_t value )
{
	return unsignedVariableByte( zigZag( value ) );
}

/*
	C equivilent function to Python struct.unpack that recognizes a variable length format
	string of characters h, H, i, I where all are assumed to be little-endian, copying from
	src to dest.  The format string is assumed to be null terminated, where
	h=int16_t, H=uint16_t, i=int32_t, I=uint32_t.  All other characters are ignored.
	* \param[in] dest pointer to destination buffer
	* \param[in] src pointer to source buffer
	* \param[in] format pointer to format string
	* \return void
*/
void structUnpack( unsigned char* dest, unsigned char* src, char* format )
{
	// check for null pointers
	if( dest == NULL || src == NULL || format == NULL ) {
		return;
	}

	int i = 0;
	while( format[ i ] != '\0' ) {
		switch( format[ i ] ) {
			case 'h':
				memcpy( dest, src, sizeof( int16_t ) );
				break;
			case 'H':
				memcpy( dest, src, sizeof( uint16_t ) );
				break;
			case 'i':
				memcpy( dest, src, sizeof( int32_t ) );
				break;
			case 'I':
				memcpy( dest, src, sizeof( uint32_t ) );
				break;
			default:
				// do nothing
				break;
		}
		i++;
	}
}

void writeData( FILE* f_out )
{
}

void parseFile( FILE* f_in, FILE* f_out )
{
	writeLogStartMarker( f_out );

	fseek( f_in, 0, SEEK_END ); // end
	long size = ftell( f_in );
	fseek( f_in, 0, SEEK_SET ); // beginning
	bool nextFramestartFound = false;
	iteration = 0;



	writeLogEndMarker( f_out );
}

/* C equivilent function to the following Python code:
if __name__ == '__main__':
	for filename in os.listdir( '.' ):
		if ( filename[ : 3 ] == 'LOG' and filename[ -4 : ] == '.TXT' ):
			print 'converting file "%s"' % filename
			f_in = open( filename, 'rb' )
			datePrefix = datetime.datetime.now().strftime( "%Y-%m-%d  %H'%M'%S  " )
			f_out = open( datePrefix + 'SLF4_' + filename[ 3 : -4 ] + '.bbl', 'wb' )
			parseFile( f_in, f_out )
			f_out.close()
			f_in.close()
			if not os.path.exists( 'trash' ):
				os.mkdir( 'trash' )
			os.rename( filename, os.path.join( 'trash', datePrefix + filename ) )
			print
	print 'all done'
	raw_input()
*/
int main( int argc, char const* argv[] )
{
	/* loop through all files in "." directory. */
	DIR* d;
	struct dirent* dir;
	d = opendir( "." );
	if( d ) {
		while( ( dir = readdir( d ) ) != NULL ) {
			// printf("%s\n", dir->d_name);
			if( strncmp( dir->d_name, "LOG", 3 ) == 0 &&
					strncmp( dir->d_name + strlen( dir->d_name ) - 4, ".TXT", 4 ) == 0 ) {
				printf( "converting file \"%s\"\n", dir->d_name );

				/* open input and output files */
				FILE* f_in = fopen( dir->d_name, "rb" );

				/* create output filename */
				char datePrefix[ 100 ];
				time_t t = time( NULL );
				struct tm tm = *localtime( &t );
				sprintf( datePrefix, "%d-%02d-%02d  %02d'%02d'%02d  ", tm.tm_year + 1900, tm.tm_mon + 1,
								 tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec );
				char* filename = malloc( strlen( datePrefix ) + 5 + strlen( dir->d_name ) - 3 - 4 + 1 );
				sprintf( filename, "%sSLF4_%s.bbl", datePrefix, dir->d_name + 3 );

				/* open output file */
				FILE* f_out = fopen( filename, "wb" );

				/* parse the input file and write bbl file contents to output file */
				parseFile( f_in, f_out );

				/* close files */
				fclose( f_out );
				fclose( f_in );

				/* free up the memmory associated with filename */
				free( filename );

				// if (!opendir("trash"))
				// {
				//   mkdir("trash");
				// }
				// char *trashFilename = malloc(strlen(datePrefix) + strlen(dir->d_name) + 1);
				// sprintf(trashFilename, "%s%s", datePrefix, dir->d_name);
				// rename(dir->d_name, trashFilename);
				// free(trashFilename);
				printf( "\n" );
			}
		}
		closedir( d );
	}

	printf( "all done\n" );

	return 0;
}