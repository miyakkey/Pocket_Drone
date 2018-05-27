#include <stdio.h>
#include <stdlib.h>
// Math library required for ‘sqrt’
#include <math.h>
// for SPI
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <string>
#include <iostream>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/time.h>


// System constants
//#define deltat 0.001f
// sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)
// gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta ( sqrt(3.0f / 4.0f) * gyroMeasError )
// compute beta


// Global system variables
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions

static uint8_t mode = 0 ;
static uint8_t bits = 8 ;
static uint32_t speed = 1000000 ;

struct timeval s, e, n, o ;

int fd ;

int spiWriteRead(unsigned char *send, unsigned char *receive, int length){
    //struct spi_ioc_transfer spi[length] ;
    struct spi_ioc_transfer *spi ;
    int i = 0 ;
    int ret = -1 ;
    int e_num;
     /*
    spi.tx_buf = (unsigned long) send ;
    spi.rx_buf = (unsigned long) receive ;
    spi.len = length ;
    spi.speed_hz = speed ;
    spi.bits_per_word = bits ;
    //spi.delay_usecs = 0 ;
    spi.cs_change = 0 ;
    */
    
    for ( i = 0 ; i < length ; i++ ){
        spi[i].tx_buf   = (unsigned long) &send[i] ;
        spi[i].rx_buf   = (unsigned long) &receive[i] ;
        spi[i].len      = 1;
        spi[i].delay_usecs = 0 ;
        spi[i].speed_hz    = speed ;
        spi[i].bits_per_word = bits ;
        spi[i].cs_change = 0 ;
    }
    
    errno = 0 ;
    ret = ioctl(fd, SPI_IOC_MESSAGE(length), spi) ;
    if ( ret < 0 ){
        e_num = errno ;
        printf("ERR CODE::%d\r\n", e_num) ;
        perror("ERRNO::") ;
        printf("W/R Error\r\n") ;
        //printf("ERRNO::%s\r\n",strerror(num)) ;
        return -1 ;
    }
    return 0 ;
}

void filterUpdate( )
{
    // Local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;
    float deltat = 0 ;
    
    gettimeofday(&n, NULL) ;
    deltat = (n.tv_usec - o.tv_usec)*1.0E-6 ;
    gettimeofday(&o, NULL) ;

    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
}

int main( ){ //accel[3], gyro[3], q[4]
    int ret ;
    float angle_ypr[3] ;
    /*
    fd = open("/dev/spidev0.0", O_RDWR) ;
    if ( fd < 0 ){
        printf("Can't open file\r\n") ;
        return 0 ;
    }
	
	printf("READ\r\n") ;*/
	a_x = 0.002 ;
    a_y = 0.001 ;
    a_z = 0.001 ;
    w_x = 0.001 ;
    w_y = 0.001 ;
    w_z = 0.001 ;
	
	gettimeofday(&s, NULL) ;
	gettimeofday(&o, NULL) ;
	
	for ( int i = 0 ; i < 500 ; i++ ) {
	    /*
	    unsigned char response[15] ;
        unsigned char data[15] = { 0x3B | 0x80, 0x3C | 0x80 , 0x3D | 0x80 , 0x3E | 0x80 , 0x3F | 0x80 , 0x40 | 0x80 , 0x41 | 0x80 , 
                                     0x42 | 0x80, 0x43 | 0x80 , 0x44 | 0x80 , 0x45 | 0x80 , 0x46 | 0x80 , 0x47 | 0x80 , 0x48 | 0x80 , 0x00 } ;
        
    	ret = spiWriteRead(data, response, 15) ;
	    if ( ret < 0 ){
	        printf("Error in IO\r\n") ;
    	    return 0 ;
	    }
	
    	a_x = (int16_t)(( (int16_t)response[1] << 8 ) | response[2] ) ;
        a_y = (int16_t)(( (int16_t)response[3] << 8 ) | response[4] ) ;
        a_z = (int16_t)(( (int16_t)response[5] << 8 ) | response[6] ) ;
        w_x = (int16_t)(( (int16_t)response[9] << 8 ) | response[10] ) ;
        w_y = (int16_t)(( (int16_t)response[11] << 8 ) | response[12] ) ;
        w_z = (int16_t)(( (int16_t)response[13] << 8 ) | response[14] ) ;*/
        
        a_x = (float)a_x * 2.f/32768.f +(float)i/100000.f;
        a_y = (float)a_y * 2.f/32768.f +(float)i/100000.f;
        a_z = (float)a_z * 2.f/32768.f +(float)i/100000.f;
        w_x = ( (float)w_x * 250.f/32768.f ) * 3.141592f/180.f +(float)i/100000.f;
        w_y = ( (float)w_y * 250.f/32768.f ) * 3.141592f/180.f +(float)i/100000.f;
        w_z = ( (float)w_z * 250.f/32768.f ) * 3.141592f/180.f +(float)i/100000.f;
        //printf("accel is %f, %f, %f\r\n", a_x, a_y, a_z) ;
        //printf(" gyro is %f, %f, %f\r\n", w_x, w_y, w_z) ;
	
        filterUpdate() ;
        angle_ypr[0] = atan2(2.0f * (SEq_2 * SEq_3 + SEq_1 * SEq_4), SEq_1 * SEq_1 + SEq_2 * SEq_2 - SEq_3 * SEq_3 - SEq_4 * SEq_4) ;
        angle_ypr[1] = asin(2.0f * (SEq_1 * SEq_3 - SEq_2 * SEq_4)) ;
        angle_ypr[2] = atan2(2.0f * (SEq_1 * SEq_2 + SEq_3 * SEq_4), SEq_1 * SEq_1 - SEq_2 * SEq_2 - SEq_3 * SEq_3 + SEq_4 * SEq_4) ;
        
        //if ( i % 100 == 0 ) {
        //    printf("YPR(%d): %f, %f, %f\r\n",i , angle_ypr[0], angle_ypr[1], angle_ypr[2]) ;
        //}
	}
	//close(fd) ;
	//sleep(1) ;
	gettimeofday(&e, NULL) ;
	
	printf("time = %lf\n", (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6) ;
	
	return 0 ;
}
// 1cycle ... 200us (wwww)