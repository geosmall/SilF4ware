#! python3.7

import os, struct, datetime
from struct import pack

craftOrientationMode = 0 # 0 .. use gyro and accelerometer, 1 .. use gyro only, 2 .. do not rotate craft display

debugMode = 6 # "Gyro Scaled"; unfiltered gyro signal
# debugMode = 12 # "ESC RPM"; motor frequence in dHz (decihertz)

def writeLogStartMarker( f ):
	f.write( b'H Product:Blackbox flight data recorder by Nicholas Sherlock\n' ) # log start marker
	f.write( b'H Data version:2\n' ) # required
	f.write( b'H I interval:1\n' )
	f.write( b'H P interval:1/1\n' )
	f.write( b'H Firmware type:Cleanflight\n' )
	f.write( b'H Firmware revision:Betaflight 4.0\n' ) # This changes for example the setpoint fields interpretation.

	f.write( b'H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisF[0],axisF[1],axisF[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],setpoint[0],setpoint[1],setpoint[2],setpoint[3],vbatLatest,amperageLatest,rssi,gyroADC[0],gyroADC[1],gyroADC[2],accSmooth[0],accSmooth[1],accSmooth[2],debug[0],debug[1],debug[2],debug[3],motor[0],motor[1],motor[2],motor[3]\n' )
	f.write( b'H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0\n' )
	# f.write( b'H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0,0,11,5,5,5\n' )
	# f.write( b'H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,3,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0\n' )
	f.write( b'H Field I signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0\n' )
	f.write( b'H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n' )
	f.write( b'H Field I encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1\n' )
	f.write( b'H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,3,3,3\n' )
	f.write( b'H Field P encoding:9,0,0,0,0,7,7,7,0,0,0,0,0,8,8,8,8,8,8,8,8,6,6,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n' )

	# f.write( b'H minthrottle:1000\n' )
	f.write( b'H maxthrottle:2000\n' ) # to make Plasmatree/PID-Analyzer happy
	f.write( b'H gyro_scale:0x3f800000\n' ) # Gyro scale 4 byte hex of IEEE float; 0x3e800000: 1=0.25 deg/sec, 0x3f800000: 1=1 deg/sec
	f.write( b'H motorOutput:0,1000\n' ) # for motor range
	f.write( b'H acc_1G:2048\n' ) # for Accelerometer range
	# f.write( b'H vbat_scale:108\n' )
	f.write( b'H vbatcellvoltage:330,350,430\n' )
	f.write( b'H vbatref:420\n' ) # 1S
	# f.write( b'H currentSensor:0,400\n' )
	f.write( b'H looptime:250\n' ) # for FFT Hz scaling
	f.write( b'H pid_process_denom:2\n' ) # for FFT Hz scaling

	f.write( b'H rc_rates:213.4,213.4,213.4\n' ) # for setpoint scaling
	# f.write( b'H rc_expo:0,0,0\n' )
	f.write( b'H rates:50,50,50\n' ) # for setpoint scaling
	# f.write( b'H rate_limits:1998,1998,1998\n' )

	f.write( b'H rollPID:31.22,1,1\n' )  # Also to make Plasmatree/PID-Analyzer happy. Since we do not know the
	f.write( b'H pitchPID:31.22,1,1\n' ) # equivalent Betaflight PID values, the response.png plot is of no use.
	f.write( b'H yawPID:31.22,1,0\n' )   # However the noise.png plot seems to be independent of those numbers.

	if debugMode == 6:
		f.write( b'H debug_mode:6\n' ) # 3 .. "Gyro Scaled"; -2000 .. 2000 deg/s
	elif debugMode == 12:
		f.write( b'H debug_mode:12\n' ) # 12 .. "ESC RPM"; arbitrary values, but [1]..[4] use the same scaling factor
	else:
		assert False, 'debugMode has no valid value'

def writeLogEndMarker( f ):
	f.write( b'E\xffEnd of log\x00' ) # optional log end marker

def unsignedVariableByte( value ): # encoder 1
	encoded = b''
	while value > 127:
		encoded += pack( 'B', ( value | 0x80 ) & 0xFF )
		value >>= 7
	encoded += pack( 'B', value )
	return encoded

def zigZag( value ):
	encoded = value * 2
	if value < 0:
		encoded = -encoded - 1
	return encoded

def signedVariableByte( value ): # encoder 0
	return unsignedVariableByte( zigZag( value ) )

iteration = 0
time = 0
axisP = [ 0, 0, 0 ]
axisI = [ 0, 0, 0 ]
axisD = [ 0, 0 ]
axisF = [ 0, 0, 0 ]
rcCommand = [ 0, 0, 0, 0 ]
setpoint = [ 0, 0, 0, 0 ]
vbatLatest = 0
amperageLatest = 0
rssi = 0
gyroADC = [ 0, 0, 0 ]
accSmooth = [ 0, 0, 0 ]
debug = [ 0, 0, 0, 0 ]
motor = [ 0, 0, 0, 0 ]
motorHz = [ 0, 0, 0, 0 ]

rssiLast = 0
rssiAvg = 0
rssiArray = [ 0 ] * int( 200 * 0.15 ) # 0.15 seconds
rssiIndex = 0

def writeData( f_out ):
	global iteration, time, axisP, axisI, axisD, axisF, rcCommand, setpoint, vbatLatest, amperageLatest, rssi, gyroADC, accSmooth, debug, motor, motorHz
	loopIteration = unsignedVariableByte( iteration )
	# time = unsignedVariableByte( 0 if iteration == 0 else time ) # micro seconds
	temp = time
	time = unsignedVariableByte( temp ) # micro seconds
	# PID, Feedforward
	axisP[0] = signedVariableByte( axisP[0] ) # -1000 .. 1000 -> -100.0% .. 100.0%
	axisP[1] = signedVariableByte( axisP[1] )
	axisP[2] = signedVariableByte( axisP[2] )
	axisI[0] = signedVariableByte( axisI[0] )
	axisI[1] = signedVariableByte( axisI[1] )
	axisI[2] = signedVariableByte( axisI[2] )
	axisD[0] = signedVariableByte( axisD[0] )
	axisD[1] = signedVariableByte( axisD[1] )
	axisF[0] = signedVariableByte( axisF[0] )
	axisF[1] = signedVariableByte( axisF[1] )
	axisF[2] = signedVariableByte( axisF[2] )
	# RC Command (sticks)
	rcCommand[0] = signedVariableByte( rcCommand[0] ) # -600 .. 600
	rcCommand[1] = signedVariableByte( rcCommand[1] ) # -600 .. 600
	rcCommand[2] = signedVariableByte( rcCommand[2] ) # -600 .. 600
	rcCommand[3] = unsignedVariableByte( rcCommand[3] ) # 1000 .. 2000
	# RC Rates
	setpoint[0] = signedVariableByte( setpoint[0] ) # -2000 .. 2000 deg/s
	setpoint[1] = signedVariableByte( setpoint[1] ) # -2000 .. 2000 deg/s
	setpoint[2] = signedVariableByte( setpoint[2] ) # -2000 .. 2000 deg/s
	setpoint[3] = signedVariableByte( setpoint[3] ) # 0 .. 1000 -> 0.0% .. 100.0%
	# Battery volt., Amperage, rssi
	vbatLatest = unsignedVariableByte( vbatLatest ) # 0.01 V/unit
	amperageLatest = signedVariableByte( amperageLatest ) # 0.01 A/unit
	# amperageLatest = signedVariableByte( rssi ) # 0.01 A/unit
	global rssiLast, rssiAvg, rssiArray, rssiIndex
	if iteration % 10 == 0:
		rssiAvg -= rssiArray[ rssiIndex ]
		delta = 1 if rssi != rssiLast else 0
		if rssi < rssiLast and rssi == 0 and rssiAvg == 0:
			delta = 0
		rssiLast = rssi
		rssiAvg += delta
		rssiArray[ rssiIndex ] = delta
		rssiIndex += 1
		rssiIndex %= len( rssiArray )
	rssi = unsignedVariableByte( int(1024 * rssiAvg / len( rssiArray ) ) ) # 0 .. 1024 -> 0% .. 100%
	# Gyros
	if 1: # Select between showing filtered (1) or unfiltered (0) gyro data
		gyroADC[0] = signedVariableByte( gyroADC[0] ) # -2000 .. 2000 deg/s
		gyroADC[1] = signedVariableByte( gyroADC[1] )
		gyroADC[2] = signedVariableByte( gyroADC[2] ) # (not represented in craft orientation display)
	else:
		gyroADC[0] = signedVariableByte( debug[0] ) # -2000 .. 2000 deg/s
		gyroADC[1] = signedVariableByte( debug[1] )
		gyroADC[2] = signedVariableByte( debug[2] ) # (not represented in craft orientation display)
	# Accelerometers
	if craftOrientationMode == 0:
		accSmooth[0] = signedVariableByte( accSmooth[0] ) # 2048 -> 1g
		accSmooth[1] = signedVariableByte( accSmooth[1] )
		accSmooth[2] = signedVariableByte( accSmooth[2] )
	elif craftOrientationMode == 1: # rely on gyro only
		accSmooth[0] = signedVariableByte( 0 )
		accSmooth[1] = signedVariableByte( 0 )
		accSmooth[2] = signedVariableByte( 2048 if iteration == 0 else 0 ) # 2048 -> 1g
	elif craftOrientationMode == 2: # do not rotate craft
		accSmooth[0] = signedVariableByte( 0 )
		accSmooth[1] = signedVariableByte( 0 )
		accSmooth[2] = signedVariableByte( 0 )
	# Debug
	if debugMode == 6:
		debug[0] = signedVariableByte( debug[0] )
		debug[1] = signedVariableByte( debug[1] )
		debug[2] = signedVariableByte( debug[2] )
		debug[3] = signedVariableByte( debug[3] )
	elif debugMode == 12:
		debug[0] = signedVariableByte( motorHz[0] )
		debug[1] = signedVariableByte( motorHz[1] )
		debug[2] = signedVariableByte( motorHz[2] )
		debug[3] = signedVariableByte( motorHz[3] )
	# Motors
	motor[0] = unsignedVariableByte( motor[0] ) # 0 .. 1000 -> 0% .. 100%
	motor[1] = unsignedVariableByte( motor[1] )
	motor[2] = unsignedVariableByte( motor[2] )
	motor[3] = unsignedVariableByte( motor[3] )
	f_out.write( b'I' + loopIteration + time +
		axisP[0] + axisP[1] + axisP[2] + axisI[0] + axisI[1] + axisI[2] + axisD[0] + axisD[1] + axisF[0] + axisF[1] + axisF[2] +
		rcCommand[0] + rcCommand[1] + rcCommand[2] + rcCommand[3] + setpoint[0] + setpoint[1] + setpoint[2] + setpoint[3] +
		vbatLatest + amperageLatest + rssi + gyroADC[0] + gyroADC[1] + gyroADC[2] + accSmooth[0] + accSmooth[1] + accSmooth[2] +
		debug[0] + debug[1] + debug[2] + debug[3] + motor[0] + motor[1] + motor[2] + motor[3]
	)
			 
def parseFile( f, f_out ):
	writeLogStartMarker( f_out )

	global iteration, time, axisP, axisI, axisD, axisF, rcCommand, setpoint, vbatLatest, amperageLatest, rssi, gyroADC, accSmooth, debug, motor, motorHz
	f.seek( 0, 2 ) # end
	size = f.tell()
	f.seek( 0, 0 ) # beginning
	nextFramestartFound = False
	iteration = 0
	while True:
		if ( nextFramestartFound or # try reading 'FRAME'
			f.read( 1 ) == b'F' and f.read( 1 ) == b'R' and f.read( 1 ) == b'A' and f.read( 1 ) == b'M' and f.read( 1 ) == b'E' ):
			try:
			# h .. int16_t, H .. uint16_t, i .. int32_t, I .. uint32_t
				iter, = struct.unpack( '<I', f.read( 4 ) )
				time, = struct.unpack( '<I', f.read( 4 ) )
				axisP = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				axisI = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				axisD = list( struct.unpack( '<hh', f.read( 4 ) ) )
				axisF = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				rcCommand = list( struct.unpack( '<hhhH', f.read( 8 ) ) )
				setpoint = list( struct.unpack( '<hhhh', f.read( 8 ) ) )
				vbatLatest, = struct.unpack( '<H', f.read( 2 ) )
				amperageLatest, = struct.unpack( '<h', f.read( 2 ) )
				rssi, = struct.unpack( '<H', f.read( 2 ) )
				gyroADC = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				accSmooth = list( struct.unpack( '<hhh', f.read( 6 ) ) )
				debug = list( struct.unpack( '<hhhh', f.read( 8 ) ) )
				motor = list( struct.unpack( '<HHHH', f.read( 8 ) ) )
				motorHz = list( struct.unpack( '<hhhh', f.read( 8 ) ) )
				# 93 bytes read
			except struct.error as message:
				if f.tell() == size:
					print('  [' + '#' * 50 + ']', end=' ')
					print('100%  ')
					print('file ended with partial frame')
					break
				else:
					print('\nstruct.error:', message)
			if f.read( 5 ) == b'FRAME':
				if iter < iteration:
					print('\n  new logging session')
					writeLogEndMarker( f_out )
					writeLogStartMarker( f_out )
				iteration = iter
				writeData( f_out )
				nextFramestartFound = True
			else:
				if f.tell() == size:
					writeData( f_out )
					print('  [' + '#' * 50 + ']', end=' ')
					print('100%  ')
					print('file ended with complete frame')
					break
				else:
					print('\n  bad frame')
					nextFramestartFound = False
					f.seek( -4, 1 ) # 4 .. len( 'FRAME' ) - 1, 1 .. relative to the current position
			if iteration % 2000 == 0:
				percent = f.tell() / float( size ) * 100
				print('  [' + '#' * int( percent / 2 ) + '.' * ( 50 - int( percent / 2 ) ) + ']', end=' ')
				print('%4.1f%%\r' % percent, end=' ')
		else:
			if f.tell() == size:
				print('no FRAME found')
				break

	writeLogEndMarker( f_out )

if __name__ == '__main__':
	for filename in os.listdir( '.' ):
		if ( filename[ : 3 ] == 'LOG' and filename[ -4 : ] == '.TXT' ):
			print('converting file "%s"' % filename)
			f_in = open( filename, 'rb' )
			datePrefix = datetime.datetime.now().strftime( "%Y-%m-%d  %H'%M'%S  " )
			f_out = open( datePrefix + 'SLF4_' + filename[ 3 : -4 ] + '.bbl', 'wb' )
			parseFile( f_in, f_out )
			f_out.close()
			f_in.close()
			# if not os.path.exists( 'trash' ):
			# 	os.mkdir( 'trash' )
			# os.rename( filename, os.path.join( 'trash', datePrefix + filename ) )
			print()
	print('all done')
	# input()

