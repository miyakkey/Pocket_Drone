
# normal library
import time
import threading
import sys
#import readchar
import numpy as np
import socket

# Blynk
# https://github.com/vshymanskyy/blynk-library-python
# use Blynk :: https://www.blynk.cc/
#import BlynkLib

# Adafruit BBIO library
# https://github.com/adafruit/adafruit-beaglebone-io-python
import Adafruit_BBIO.PWM as pwm
import Adafruit_BBIO.ADC as adc
import Adafruit_BBIO.SPI as SPI

# my local library
import cpphelper_calc


###### NOTE ###### NOTE ###### NOTE ###### NOTE ###### NOTE ######
#to do
# replace "readchar"
# add git readme and license
# Blynk is not stable, so replace socket connecting to my local pc

##gain
#d-0.005, p-0.013 : d is a little large?
#d-0.004, p-0.015 : d is a little large? batt 7.8v
#d-0.003~0.0025, p-0.015 : d is a bit large?
###### NOTE ###### NOTE ###### NOTE ###### NOTE ###### NOTE ######


#connstant
PIN_PWM = ( "P1_33" , "P1_36" , "P2_1" , "P2_3" )
PIN_BATT = "P1_25"
K_ADC = 1.80 * 10.65
#THRESHOLD_BATT = 7.4
THRESHOLD_BATT = 10.8
PWM_FREQUENCY = 1000
#K_YPR_P = np.asarray([ 0.005, 0.015, 0.015 ], dtype = np.float32)
#K_YPR_D = np.asarray([ 0, 0.0025, 0.0025 ], dtype = np.float32)
K_YPR_P = np.asarray([ 0.003, 0.003, 0.003 ], dtype = np.float32)
K_YPR_D = np.asarray([ 0.0, 0.0, 0.0 ], dtype = np.float32)
K_YPR_I = np.asarray([ 0.0, 0.0, 0.0 ], dtype = np.float32)
#M_OFFSET = np.asarray([0, 0, -0.04, 0], dtype = np.float16)
READ_FLAG = 0x80
READ_ALL = ( 0x3B | READ_FLAG, 0x3C | READ_FLAG, 0x3D | READ_FLAG, 0x3E | READ_FLAG, 0x3F | READ_FLAG, 0x40 | READ_FLAG,
             0x41 | READ_FLAG, 0x42 | READ_FLAG,
             0x43 | READ_FLAG, 0x44 | READ_FLAG, 0x45 | READ_FLAG, 0x46 | READ_FLAG, 0x47 | READ_FLAG, 0x48 | READ_FLAG, 0x00 )
MPUREG_WHOAMI = 0x75
MPUREG_INT_STATUS = 0x3A
#BLYNK_AUTH = '7af31f3ac6dc44a596eb0b414b3b2e09'

#hadler
spi = SPI.SPI(0,0)
calc = cpphelper_calc.CalcHelper()
#blynk = BlynkLib.Blynk(BLYNK_AUTH)

#global varient
target_ypr = np.asarray([ 0, 0, 0 ], dtype = np.float32)
throttle = 0.0

#function
def get_batt() :
    """
    batt check for threading
    @param  -> none
    @return -> none
    """
    global PIN_BATT, K_ADC
    while ( True ) :
        global flag_main
        _batt = adc.read(PIN_BATT) * K_ADC
        if ( _batt < THRESHOLD_BATT ) :
            print ( "batt is {:.3f}V".format(_batt) )
            print ( "batt is going down. Charge now" )
            flag_main = False
            while ( True ) :
                time.sleep(100)
        if ( _batt < THRESHOLD_BATT + 0.1 ) :
            print ( "Warning::batt level is low, {:.3f}V".format(_batt) )
        time.sleep(10)

def controler() :
    """
    controler for threading
    @param  -> none
    @return -> none
    """
    global target_ypr, throttle, calc, spi
    response = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    ret = [0, 0]
    mortor_power = np.asarray([0,0,0,0], dtype = np.float16)
    #angle_ypr_c = np.asarray([0,0,0], dtype = np.float32)
    while( True ) :
        ret = spi.xfer2( [ MPUREG_INT_STATUS | READ_FLAG , 0x00 ] )
        if ( ret[1]&0x01 ) :
            response = spi.xfer2( list( READ_ALL ) )
            calc.update(*response)
            #angle_ypr_c[0] = calc.get_ypr_y()
            #angle_ypr_c[1] = calc.get_ypr_p()
            #angle_ypr_c[2] = calc.get_ypr_r()
            #print(angle_ypr_c)
            #calculate controling
            calc.control(throttle, *target_ypr)
            mortor_power[0] = calc.get_m_power(0) + throttle
            mortor_power[1] = calc.get_m_power(1) + throttle
            mortor_power[2] = calc.get_m_power(2) + throttle
            mortor_power[3] = calc.get_m_power(3) + throttle
            move(mortor_power)
            #print(mortor_power)
        time.sleep(0.002)

def move(args) :
    """
    mortor safety access function 
    @param  -> numpy array (4) # mortor(i) raise this power. 0.0 - 1.0
    @return -> none
    """
    global PIN_BATT#, M_OFFSET
    #args = args + M_OFFSET
    args = np.clip(args, 0.0, 1.0)
    #args = args * 12.5 + 12.5 #this is true One-shot 125, for STM32 controler ESC
    args = args * 100 # normal PWM
    for i in range (4) :
        pwm.set_duty_cycle(PIN_PWM[i], args[i])

def init_mpu(spi) :
    """
    initialize MPU6500
    @param  -> SPI handler ( =Adafruit_BBIO.SPI.SPI(x,x) )
    @return1-> numpy array (3) # acceleration sensor bias data
    @return2-> numpy array (3) # gyro sensor bias data
    """
    accel_bias = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    gyro_bias  = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    MPUREG_WHOAMI = 0x75 #return 0x70 or 112
    MPUREG_USER_CTRL = 0x6A
    MPUREG_PWR_MGMT_1 = 0x6B
    MPUREG_PWR_MGMT_2 = 0x6C
    MPUREG_SMPLRT_DIV = 0x19
    MPUREG_CONFIG = 0x1A
    MPUREG_GYRO_CONFIG = 0x1B
    MPUREG_ACCEL_CONFIG = 0x1C
    MPUREG_ACCEL_CONFIG_2 = 0x1D
    MPUREG_SMPLRT_DIV = 0x19
    MPUREG_INT_PIN_CFG = 0x37
    MPUREG_INT_ENABLE = 0x38
    MPUREG_INT_STATUS = 0x3A
    MPUREG_FIFO_EN = 0x23
    MPUREG_I2C_MST_CTRL = 0x24
    READ_FLAG = 0x80
    READ_ALL = ( 0x3B | READ_FLAG, 0x3C | READ_FLAG, 0x3D | READ_FLAG, 0x3E | READ_FLAG, 0x3F | READ_FLAG, 0x40 | READ_FLAG,
             0x41 | READ_FLAG, 0x42 | READ_FLAG,
             0x43 | READ_FLAG, 0x44 | READ_FLAG, 0x45 | READ_FLAG, 0x46 | READ_FLAG, 0x47 | READ_FLAG, 0x48 | READ_FLAG, 0x00 )
    ##### boot and who am I #####
    print ( "###### Start MPU Initialize ######" )
    ret = spi.xfer2( [ MPUREG_WHOAMI | READ_FLAG , 0x00 ] )
    if ( ret[1] != 0x70 ) :
        sys.exit("Faile MPU connection")
    print ( "Success MPU connection" )
    
    # power off
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_1, 0x80 ] )
    time.sleep(0.1)
    # power on
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_1, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_2, 0x00 ] )
    time.sleep(0.2)
    
    ##### self test #####
    print ( "Start MPU Self Test..." )
    ## mpu6500 self test ##
    # ( https://github.com/NordicPlayground/nrf52-quadcopter/blob/master/Firmware/drivers/mpu6500.c )
    mpu6500StTb = (
        2620,2646,2672,2699,2726,2753,2781,2808,2837,2865,2894,2923,2952,2981,3011,3041, #15
        3072,3102,3133,3165,3196,3228,3261,3293,3326,3359,3393,3427,3461,3496,3531,3566, #31
        3602,3638,3674,3711,3748,3786,3823,3862,3900,3939,3979,4019,4059,4099,4140,4182, #47
        4224,4266,4308,4352,4395,4439,4483,4528,4574,4619,4665,4712,4759,4807,4855,4903, #63
        4953,5002,5052,5103,5154,5205,5257,5310,5363,5417,5471,5525,5581,5636,5693,5750, #79
        5807,5865,5924,5983,6043,6104,6165,6226,6289,6351,6415,6479,6544,6609,6675,6742, #95
        6810,6878,6946,7016,7086,7157,7229,7301,7374,7448,7522,7597,7673,7750,7828,7906, #111
        7985,8065,8145,8227,8309,8392,8476,8561,8647,8733,8820,8909,8998,9088,9178,9270,
        9363,9457,9551,9647,9743,9841,9939,10038,10139,10240,10343,10446,10550,10656,10763,10870,
        10979,11089,11200,11312,11425,11539,11654,11771,11889,12008,12128,12249,12371,12495,12620,12746,
        12874,13002,13132,13264,13396,13530,13666,13802,13940,14080,14221,14363,14506,14652,14798,14946,
        15096,15247,15399,15553,15709,15866,16024,16184,16346,16510,16675,16842,17010,17180,17352,17526,
        17701,17878,18057,18237,18420,18604,18790,18978,19167,19359,19553,19748,19946,20145,20347,20550,
        20756,20963,21173,21385,21598,21814,22033,22253,22475,22700,22927,23156,23388,23622,23858,24097,
        24338,24581,24827,25075,25326,25579,25835,26093,26354,26618,26884,27153,27424,27699,27976,28255,
        28538,28823,29112,29403,29697,29994,30294,30597,30903,31212,31524,31839,32157,32479,32804,33132 )
    # Write test configuration
    ret = spi.xfer2( [ MPUREG_SMPLRT_DIV, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_CONFIG, 0x02 ] )
    ret = spi.xfer2( [ MPUREG_GYRO_CONFIG, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG_2, 0x02 ] )
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG, 0x00 ] )
    # get average current values of gyro and acclerometer
    i = 0
    t_accel = [0, 0, 0]
    t_gyro = [0, 0, 0]
    self_test_accel_avg = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    self_test_gyro_avg = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    while ( i < 200 ) :
        i = i + 1
        response = spi.xfer2( [ 0x3B | READ_FLAG, 0x3C | READ_FLAG, 0x3D | READ_FLAG, 0x3E | READ_FLAG, 0x3F | READ_FLAG, 0x40 | READ_FLAG, 0x00 ] )
        t_accel[0] = ( response[1] << 8 ) | response[2]
        t_accel[1] = ( response[3] << 8 ) | response[4]
        t_accel[2] = ( response[5] << 8 ) | response[6]
        response = spi.xfer2( [ 0x43 | READ_FLAG, 0x44 | READ_FLAG, 0x45 | READ_FLAG, 0x46 | READ_FLAG, 0x47 | READ_FLAG, 0x48 | READ_FLAG, 0x00 ] )
        t_gyro[0] = ( response[1] << 8 ) | response[2]
        t_gyro[1] = ( response[3] << 8 ) | response[4]
        t_gyro[2] = ( response[5] << 8 ) | response[6]
        for count in range (3) :
            self_test_accel_avg[count] = self_test_accel_avg[count] + ( -(t_accel[count] & 0b1000000000000000) | (t_accel[count] & 0b0111111111111111) )
            self_test_gyro_avg[count]  = self_test_gyro_avg[count]  + ( -(t_gyro[count]  & 0b1000000000000000) | (t_gyro[count]  & 0b0111111111111111) )
        time.sleep(0.001)
    self_test_accel_avg = self_test_accel_avg / 200
    self_test_gyro_avg = self_test_gyro_avg / 200
    print ( " accel selftest : normal avg= %s" % (self_test_accel_avg) )
    print ( " gyro  selftest : normal avg= %s" % (self_test_gyro_avg) )
    # Configure the accelerometer for self-test
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG, 0xE0 ] )
    ret = spi.xfer2( [ MPUREG_GYRO_CONFIG, 0xE0 ] )
    time.sleep(0.025)
    # get average self-test values of gyro and acclerometer
    i = 0
    t_accel = [0, 0, 0]
    t_gyro = [0, 0, 0]
    self_test_accel_on = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    self_test_gyro_on = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    while ( i < 200 ) :
        i = i + 1
        response = spi.xfer2( [ 0x3B | READ_FLAG, 0x3C | READ_FLAG, 0x3D | READ_FLAG, 0x3E | READ_FLAG, 0x3F | READ_FLAG, 0x40 | READ_FLAG, 0x00 ] )
        t_accel[0] = ( response[1] << 8 ) | response[2]
        t_accel[1] = ( response[3] << 8 ) | response[4]
        t_accel[2] = ( response[5] << 8 ) | response[6]
        response = spi.xfer2( [ 0x43 | READ_FLAG, 0x44 | READ_FLAG, 0x45 | READ_FLAG, 0x46 | READ_FLAG, 0x47 | READ_FLAG, 0x48 | READ_FLAG, 0x00 ] )
        t_gyro[0] = ( response[1] << 8 ) | response[2]
        t_gyro[1] = ( response[3] << 8 ) | response[4]
        t_gyro[2] = ( response[5] << 8 ) | response[6]
        for count in range (3) :
            self_test_accel_on[count] = self_test_accel_on[count] + ( -(t_accel[count] & 0b1000000000000000) | (t_accel[count] & 0b0111111111111111) )
            self_test_gyro_on[count]  = self_test_gyro_on[count]  + ( -(t_gyro[count]  & 0b1000000000000000) | (t_gyro[count]  & 0b0111111111111111) )
        time.sleep(0.001)
    self_test_accel_on = self_test_accel_on / 200
    self_test_gyro_on = self_test_gyro_on / 200
    print ( " accel selftest : selftest-mode avg= %s" % (self_test_accel_on) )
    print ( " gyro  selftest : selftest-mode avg= %s" % (self_test_gyro_on) )
    
    selftest_g = spi.xfer2( [ 0x00 | READ_FLAG , 0x01 | READ_FLAG , 0x02 | READ_FLAG , 0x00 ] )
    selftest_a = spi.xfer2( [ 0x0D | READ_FLAG , 0x0E | READ_FLAG , 0x0F | READ_FLAG , 0x00 ] )
    # Configure the gyro and accelerometer for normal operation
    ret = spi.xfer2( [ MPUREG_GYRO_CONFIG, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG, 0x00 ] )
    time.sleep(0.05)
    # Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    factory_trim_a = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    factory_trim_g = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    i = 0
    for i in range ( 3 ) :
        if ( selftest_a[i+1] != 0 ) :
            factory_trim_a[i] = mpu6500StTb[selftest_a[i+1] - 1]
        else :
            factory_trim_a[i] = 0
        if ( selftest_g[i+1] != 0 ) :
            factory_trim_g[i] = mpu6500StTb[selftest_g[i+1] - 1]
        else :
            factory_trim_g[i] = 0
    #Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    gyro_diff = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    accel_diff = np.asarray([ 0, 0, 0 ], dtype=np.float32)
    
    accel_diff = 100.0*( ( self_test_accel_on - self_test_accel_avg ) - factory_trim_a )/factory_trim_a
    gyro_diff = 100.0*( ( self_test_gyro_on - self_test_gyro_avg ) - factory_trim_g )/factory_trim_g
    
    print ( " accel selftest = {0} %".format(accel_diff) )
    print ( " gyro  selftest = {0} %".format(gyro_diff) )
    for i in range ( 3 ) :
        if ( accel_diff[i] > 14.0 or accel_diff[i] < -14.0 ) :
            print ( "Factory Value is +-14%" )
            print ( "Accel %s is Out of range" % i )
            sys.exit( "MPU Self Test Error" )
        if ( gyro_diff[i] > 14.0 or gyro_diff[i] < -14.0 ) :
            print ( "Factory Value is +-14%" )
            print ( "Gyro %s is Out of range" % i )
            sys.exit( "MPU Self Test Error" )
    print ("Finish Self Test" )
    
    
    ##### Calibrate #####
    print ( "Start MPU Calibrate..." )
    #reset
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_1 , 0x80 ] )
    time.sleep(0.1)
    #power on
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_1, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_2, 0x00 ] )
    #ret = spi.xfer2( [ MPUREG_USER_CTRL, 0x10 ] ) # maybe need this command
    time.sleep(0.2)
    ret = spi.xfer2( [ MPUREG_INT_ENABLE, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_FIFO_EN, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_1, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_I2C_MST_CTRL, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_USER_CTRL, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_USER_CTRL, 0x0C ] )
    time.sleep(0.015)
    
    ret = spi.xfer2( [ MPUREG_CONFIG, 0x01 ] )
    ret = spi.xfer2( [ MPUREG_SMPLRT_DIV, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_GYRO_CONFIG, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG, 0x00 ] )
    
    ret = spi.xfer2( [ MPUREG_INT_PIN_CFG, 0x20 ] )
    ret = spi.xfer2( [ MPUREG_INT_ENABLE, 0x01 ] )
    
    BIAS_ACCEL_SENSITIVITY = 32768.0/2.0
    BIAS_GYRO_SENSITIVITY = 32768.0/250.0
    
    i = 0
    t_accel = [0, 0, 0]
    t_gyro = [0, 0, 0]
    while ( i < 80 ) :
        ret = spi.xfer2( [ MPUREG_INT_STATUS | READ_FLAG , 0x00 ] )
        if ( ret[1]&0x01 ) :
            i = i + 1
            #response = spi.xfer2( list(READ_SIGNAL) )
            response = spi.xfer2( [ 0x3B | READ_FLAG, 0x3C | READ_FLAG, 0x3D | READ_FLAG, 0x3E | READ_FLAG, 0x3F | READ_FLAG, 0x40 | READ_FLAG, 0x00 ] )
            t_accel[0] = ( response[1] << 8 ) | response[2]
            t_accel[1] = ( response[3] << 8 ) | response[4]
            t_accel[2] = ( response[5] << 8 ) | response[6]
            response = spi.xfer2( [ 0x43 | READ_FLAG, 0x44 | READ_FLAG, 0x45 | READ_FLAG, 0x46 | READ_FLAG, 0x47 | READ_FLAG, 0x48 | READ_FLAG, 0x00 ] )
            t_gyro[0] = ( response[1] << 8 ) | response[2]
            t_gyro[1] = ( response[3] << 8 ) | response[4]
            t_gyro[2] = ( response[5] << 8 ) | response[6]
            for count in range (3) :
                accel_bias[count] = accel_bias[count] + ( -(t_accel[count] & 0b1000000000000000) | (t_accel[count] & 0b0111111111111111) )
                gyro_bias[count]  = gyro_bias[count]  + ( -(t_gyro[count]  & 0b1000000000000000) | (t_gyro[count]  & 0b0111111111111111) )
    
    accel_bias = accel_bias / 80
    gyro_bias = gyro_bias / 80
    print ("CALIBRATE RAW DATA")
    print ( "accel-x:%s, accel-y:%s, accel-z:%s" % (accel_bias[0], accel_bias[1], accel_bias[2]) )
    print ( "gyro-x:%s, gyro-y:%s, gyro-z:%s" % (gyro_bias[0], gyro_bias[1], gyro_bias[2]) )
    if ( accel_bias[2] > 0 ) :
        accel_bias[2] = accel_bias[2] - BIAS_ACCEL_SENSITIVITY
    else :
        accel_bias[2] = accel_bias[2] + BIAS_ACCEL_SENSITIVITY
    accel_bias = accel_bias / BIAS_ACCEL_SENSITIVITY
    gyro_bias = gyro_bias / BIAS_GYRO_SENSITIVITY
    print ("CALIBRATE DATA")
    print ( "accel-x:%s, accel-y:%s, accel-z:%s" % (accel_bias[0], accel_bias[1], accel_bias[2]) )
    print ( "gyro-x:%s, gyro-y:%s, gyro-z:%s" % (gyro_bias[0], gyro_bias[1], gyro_bias[2]) )
    
    
    ##### Initialize for Sampling Data #####
    print ( "MPU initialized for active data mode..." )
    #power off
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_1 , 0x80 ] )
    time.sleep(0.1)
    #power on
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_1, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_2, 0x00 ] )
    ret = spi.xfer2( [ MPUREG_USER_CTRL, 0x10 ] )
    time.sleep(0.1)
    ret = spi.xfer2( [ MPUREG_PWR_MGMT_1, 0x01 ] ) # set clock source to PLL with x-axis ( more reliable than default )
    #sampling frequency and Digital LPF
    ret = spi.xfer2( [ MPUREG_SMPLRT_DIV, 0x01 ] ) # set sampling frequenby (0 is 1kHz, fastest) (1 is 500Hz ) Attention::python fastest result is 1.64ms to get and calculate 1 data
    ret = spi.xfer2( [ MPUREG_CONFIG, 0x01 ] ) # set gyro LPF (0 is 250Hz, 0.97ms:delay)
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG_2, 0x00 ] ) # set accel LPF (0b100 is 1.13kHz(default), 0.75ms:delay) (0 is 460Hz, 1.97ms:delay)
    #gyro scope range
    ret = spi.xfer2( [ MPUREG_GYRO_CONFIG | READ_FLAG , 0x00] )
    print ( ret )
    c = ret[1]
    ret = spi.xfer2( [ MPUREG_GYRO_CONFIG, c & ~0xE0 ] )
    ret = spi.xfer2( [ MPUREG_GYRO_CONFIG, c & ~0x18 ] )
    ret = spi.xfer2( [ MPUREG_GYRO_CONFIG, c | 0 << 3 ] ) # 0 is 250DPS
    #accel scope range
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG | READ_FLAG , 0x00] )
    print ( ret )
    c = ret[1]
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG, c & ~0xE0 ] )
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG, c & ~0x18 ] )
    ret = spi.xfer2( [ MPUREG_ACCEL_CONFIG, c | 0 << 3 ] ) # 0 is 2G
    #Configure Interrupts and Bypass Enable
    ret = spi.xfer2( [ MPUREG_INT_PIN_CFG, 0x20 ] ) # interrupt pin set to 'active high, push-pull and clear on read of INT_STATUS'
    ret = spi.xfer2( [ MPUREG_INT_ENABLE, 0x01 ] ) # enable data ready intterupt
    print ( "###### Finish MPU initialize! ######" )
    print ( "" )
    return accel_bias, gyro_bias

def send_ack() :
    while ( True ) :
        server.sendto(b'ACK', (send_addr, int(send_port)))
        time.sleep(5)

def receive_data() :
    global throttle, target_ypr
    while ( True ) :
        data, (addr, port) = server.recvfrom(512)
        if ( addr == send_addr ) :
            s_data = data.decode('utf-8')
            ss_data = s_data.split('@')
            if ( ss_data[0] == 't' ) :
                throttle = float(ss_data[1]) / 100.0
            elif ( ss_data[0] == 'p' ) :
                target_ypr[1] = float(ss_data[1]) / 25.0
            elif ( ss_data[0] == 'r' ) :
                target_ypr[2] = float(ss_data[1]) / 25.0
        time.sleep(0.005)
    
'''
@blynk.VIRTUAL_WRITE(0)
def b_input0(val) :
    global throttle
    throttle = float(val) / 150.0 # max, 0.6

@blynk.VIRTUAL_WRITE(1)
def b_input1(val) :
    global target_ypr
    target_ypr[2] = float(val) / 25.0 #max, 4degree

@blynk.VIRTUAL_WRITE(2)
def b_input2(val) :
    global target_ypr
    target_ypr[1] = float(val) / 20.0

def blynk_handler():
    blynk.run()
'''
#################################################  main  #################################################
###### setup ######
print ( "Check Python version" )
print ( sys.version )
print ( "" )
### MPU initialize ###
accel_bias = np.asarray([ 0, 0, 0 ], dtype=np.float32)
gyro_bias  = np.asarray([ 0, 0, 0 ], dtype=np.float32)
spi.mode = 0
spi.msh = 1000000
accel_bias, gyro_bias = init_mpu(spi)
ret = spi.xfer2( [ MPUREG_WHOAMI | READ_FLAG , 0x00 ] )
if ( ret[1] != 0x70 ) :
    print ( ret )
    sys.exit("Faile MPU connection")
print ( "Success in connecting to MPU, change communication speed to 20MHz" )
spi.msh = 20000000
calc.set_bias_a(*accel_bias)
calc.set_bias_g(*gyro_bias)

### battery check ###
print ( "Checking Battery ..." )
adc.setup()
batt = adc.read(PIN_BATT) * K_ADC
if ( batt > ( THRESHOLD_BATT + 0.1 ) ) :
    print ( "batt is {:.3f}V, OK".format(batt) )
else :
    print ( "batt is {:.3f}V".format(batt) )
    sys.exit("batt is going down. Charge now")
t_batt = threading.Thread( target=get_batt )
t_batt.setDaemon(True)
t_batt.start()
'''
### Blynk setup ###
print ( "Start Blynk Setup ..." )
t_blynk = threading.Thread( target=blynk_handler )
t_blynk.setDaemon(True)
t_blynk.start()
time.sleep(3) # wait blynk connection
'''
'''
### Socket setup ###
#check 'server.close()' at the end of this program
print ( 'Starting UDP Setup ...' )
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 10002
server.bind(('', port))
send_addr = ''
send_port = ''
flag_connection = False
while ( flag_connection == False ) :
    print ( 'Waiting Connection. Please send message from controler' )
    data, (send_addr, send_port) = server.recvfrom(1024)
    print ( 'Receive data, {0}'.format(data) )
    print ( 'from ::address={0}, port={1}'.format(send_addr, send_port) )
    print ( 'is this your controler? y/n' )
    char = input('>')
    if ( char == 'y' ) :
        flag_connection = True
server.sendto(b'Hello from Server', (send_addr, int(send_port)))
print ( 'Server send test message.' )
#t_sendack = threading.Thread( target=send_ack )
#t_sendack.setDaemon(True)
#t_sendack.start()
t_receive = threading.Thread( target=receive_data )
t_receive.setDaemon(True)
t_receive.start()
'''
### ESC setup ###
print ( "Start ESC Setup ..." )
for i in range (4) :
    try :
        pwm.start(PIN_PWM[i], 0.0, PWM_FREQUENCY)
    except RuntimeError :
        print ( "...try to boot PWM-%s again" % i )
        time.sleep(1)
        try :
            pwm.start(PIN_PWM[i], 0.0, PWM_FREQUENCY)
        except :
            print(sys.exc_info())
            sys.exit("Error in booting ESC")
print ( "Success in PWM Setup " )
print ( "Connect Battery to ESC, press 'y' to continue..." )
char = input(">")
if ( char != 'y' ) :
    sys.exit("User Interrupt")
temp = np.asarray([0.0,0.0,0.0,0.0], dtype = np.float16)
temp.fill(0.05)
move(temp)
char = input(">")
if ( char != 'y' ) :
    sys.exit("User Interrupt")

temp = np.asarray([0.0,0.0,0.0,0.0], dtype = np.float16)
move(temp)
char = input(">")
if ( char != 'y' ) :
    sys.exit("User Interrupt")

print ( "Start" )


###### raise program ######
flag_main = True

calc.set_kp(*K_YPR_P)
calc.set_kd(*K_YPR_D)
calc.set_ki(*K_YPR_I)

t_controler = threading.Thread( target=controler )
t_controler.setDaemon(True)
t_controler.start()

time_delta = 0
time_old = time.time()
#time_start = time_old
count = 0

while ( flag_main ) :
    print ( target_ypr )
    print ( throttle )
    char = input(">")
    if ( char == 's' ) :
        target_ypr[0] = 0
        target_ypr[1] = 0
        target_ypr[2] = 0
        throttle = 0
    elif ( char == 'w' ) :
        target_ypr[1] = target_ypr[1] + 0.5
    elif ( char == 'x' ) :
        target_ypr[1] = target_ypr[1] - 0.5
    elif ( char == 'a' ) :
        target_ypr[2] = target_ypr[2] + 0.5
    elif ( char == 'd' ) :
        target_ypr[2] = target_ypr[2] - 0.5
    elif ( char == 'o' ) :
        throttle = throttle + 0.025
    elif ( char == 'l' ) :
        throttle = throttle - 0.025
    elif ( char == 'k' ) :
        flag_main = False

###### cleanup process ######
#end = time.time() - time_start
#print ( "Time,{0}".format(end) )
#server.close()
pwm.cleanup()
print ( "End Process" )