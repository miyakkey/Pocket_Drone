#include <math.h>
#include <sys/time.h>
#include <syslog.h>
#include <stdio.h>

#include "cpphelper_calc_.h"

namespace n_cpphelper_calc{

cpphelper_calc::cpphelper_calc():beta( sqrt(3.0f / 4.0f) * M_PI * 5.0f / 180.0f ){
    SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;
    a_x = 0.0f , a_y = 0.0f , a_z = 0.0f ;
    w_x = 0.0f , w_y = 0.0f , w_z = 0.0f ;
    height = 0.0 , v_z = 0.0 ;
    gravity = 0.0f ;
    for ( int i = 0 ; i < 3 ; i++ ){
        angle_ypr[i] = 0.0f ;
        bias_a[i] = 0.0f ;
        bias_g[i] = 0.0f ;
        kp_ypr[i] = 0.0f ;
        kd_ypr[i] = 0.0f ;
        ki_ypr[i] = 0.0f ;
        delta_ypr[i] = 0.0f ;
        delta_ypr_delta[i] = 0.0f ;
        delta_ypr_old[i] = 0.0f ;
        delta_ypr_integrate[i] = 0.0f ;
        control_ypr[i] = 0.0f ;
    }
    for ( int i = 0 ; i < 4 ; i++ ){
        mortor_power[i] = 0.0f ;
    }
    flag_first = 0 ;
    flag_first_m = 0 ;
    //debug for syslog
    openlog("syslog_cpphelper",LOG_CONS | LOG_PID, LOG_LOCAL1) ;
    syslog(LOG_DEBUG, "cpphelper initialize\r\n") ;
    return ;
}

void cpphelper_calc::update(    unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4, unsigned char d5, unsigned char d6,
                                unsigned char d7, unsigned char d8,
                                unsigned char d9, unsigned char d10, unsigned char d11, unsigned char d12, unsigned char d13, unsigned char d14 ) {
    // ### Assemble Data ###
    int16_t dest_a[3] ;
    int16_t dest_g[3] ;
    dest_a[0] = int16_t( int16_t(d1<<8) | d2 ) ;
    dest_a[1] = int16_t( int16_t(d3<<8) | d4 ) ;
    dest_a[2] = int16_t( int16_t(d5<<8) | d6 ) ;
    dest_g[0] = int16_t( int16_t(d9<<8) | d10 ) ;
    dest_g[1] = int16_t( int16_t(d11<<8) | d12 ) ;
    dest_g[2] = int16_t( int16_t(d13<<8) | d14 ) ;
    a_x = float(dest_a[0]) * 2.0f/32768.0f - bias_a[0] ;
    w_x = ( float(dest_g[0]) * 250.0f/32768.0f- bias_g[0] ) * M_PI/180.0f ;
    a_y = float(dest_a[1]) * 2.0f/32768.0f - bias_a[1] ;
    w_y = ( float(dest_g[1]) * 250.0f/32768.0f- bias_g[1] ) * M_PI/180.0f ;
    a_z = float(dest_a[2]) * 2.0f/32768.0f - bias_a[2] ;
    w_z = ( float(dest_g[2]) * 250.0f/32768.0f- bias_g[2] ) * M_PI/180.0f ;
    
    // ### Madgwick Update ###
    // ## Estimate Attitude angle using Madgwick filter ##
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
    float deltat;
    
    if ( flag_first == 0 ){
        flag_first = 1 ;
        gettimeofday(&o, NULL) ;
    }
    
    gettimeofday(&n, NULL) ;
    deltat = (n.tv_sec - o.tv_sec) + (n.tv_usec - o.tv_usec)/1000000.0f ;
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
    SEqHatDot_3 = J_12or23 * f_2 - J_13or22 * f_1 - J_33 * f_3 ;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 =  halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 =  halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 =  halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // Compute then integrate the estimated quaternion derrivative
    SEq_1 += ((SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat);
    SEq_2 += ((SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat);
    SEq_3 += ((SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat);
    SEq_4 += ((SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat);
    // Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
    
    angle_ypr[0] = atan2(2.0f * (SEq_2 * SEq_3 + SEq_1 * SEq_4), SEq_1 * SEq_1 + SEq_2 * SEq_2 - SEq_3 * SEq_3 - SEq_4 * SEq_4) * 180.0f / M_PI ;
    angle_ypr[1] = asin(2.0f * (SEq_1 * SEq_3 - SEq_2 * SEq_4))  * 180.0f / M_PI ;
    angle_ypr[2] = atan2(2.0f * (SEq_1 * SEq_2 + SEq_3 * SEq_4), SEq_1 * SEq_1 - SEq_2 * SEq_2 - SEq_3 * SEq_3 + SEq_4 * SEq_4) * 180.0f / M_PI ;
    
    // ### Update height ###
    // simple integrate version
    v_z = v_z + (double)( a_z - gravity ) * (double)deltat ;
    height = height + v_z * (double)deltat ;
    
    return ;
}

void cpphelper_calc::control(float _throttle, float _target_y, float _target_p, float _target_r){
    float target_ypr[3] = { _target_y, _target_p, _target_r } ;
    float time_delta ;
    
    for ( int i = 0 ; i < 3 ; i++ ) {
        delta_ypr[i] = angle_ypr[i] - target_ypr[i] ;
        delta_ypr_delta[i] = delta_ypr[i] - delta_ypr_old[i] ;
        delta_ypr_old[i] = delta_ypr[i] ;
        delta_ypr_integrate[i] += delta_ypr[i] ;
    }
    
    if ( flag_first_m == 0 ){
        flag_first_m = 1 ;
        gettimeofday(&o_m, NULL) ;
    }
    
    gettimeofday(&n_m, NULL) ;
    time_delta = (n_m.tv_sec - o_m.tv_sec) + (n_m.tv_usec - o_m.tv_usec)/1000000.0f ;
    gettimeofday(&o_m, NULL) ;
    
    for ( int i = 0 ; i < 3 ; i++ ){
        control_ypr[i] = kp_ypr[i] * delta_ypr[i] + kd_ypr[i] * delta_ypr_delta[i] / time_delta + ki_ypr[i] * delta_ypr_integrate[i] * time_delta ;
    }
    mortor_power[0] = _throttle - control_ypr[0] + control_ypr[1] + control_ypr[2] ; //right - flont
    mortor_power[1] = _throttle + control_ypr[0] + control_ypr[1] - control_ypr[2] ; // left - flont
    mortor_power[2] = _throttle - control_ypr[0] - control_ypr[1] - control_ypr[2] ; // left - back
    mortor_power[3] = _throttle + control_ypr[0] - control_ypr[1] + control_ypr[2] ; //right - back
}

float cpphelper_calc::get_m_power(int _i){
    if ( ( _i >= 0 ) && ( _i < 4 ) ) {
        return mortor_power[_i] ;
    }
    return 0 ;
}

void cpphelper_calc::set_bias_a(float _bx, float _by, float _bz){
    bias_a[0] = _bx ;
    bias_a[1] = _by ;
    bias_a[2] = _bz ;
    return ;
}

void cpphelper_calc::set_bias_g(float _bx, float _by, float _bz){
    bias_g[0] = _bx ;
    bias_g[1] = _by ;
    bias_g[2] = _bz ;
    return ;
}

float cpphelper_calc::get_q(int _qi){
    if ( _qi == 1 ) {
        return SEq_1 ;
    } else if ( _qi == 2 ) {
        return SEq_2 ;
    } else if ( _qi == 3 ) {
        return SEq_3 ;
    } else if ( _qi == 4 ) {
        return SEq_4 ;
    } else {
        return 0 ;
    }
}

float cpphelper_calc::get_ypr(int _i){
    if ( ( _i >= 0 ) && ( _i < 3 ) ){
        return angle_ypr[_i] ;
    }
    return 0 ;
}

float cpphelper_calc::get_ypr_y(){
    return angle_ypr[0] ;
}

float cpphelper_calc::get_ypr_p(){
    return angle_ypr[1] ;
}

float cpphelper_calc::get_ypr_r(){
    return angle_ypr[2] ;
}

float cpphelper_calc::get_a(int _i){
    if ( _i == 0 ) {
        return a_x ;
    } else if ( _i == 1 ) {
        return a_y ;
    } else if ( _i == 2 ) {
        return a_z ;
    } else {
        return 0 ;
    }
}

float cpphelper_calc::get_g(int _i){
    if ( _i == 0 ) {
        return w_x ;
    } else if ( _i == 1 ) {
        return w_y ;
    } else if ( _i == 2 ) {
        return w_z ;
    } else {
        return 0 ;
    }
}

void cpphelper_calc::set_kp(float _y, float _p, float _r){
    kp_ypr[0] = _y ;
    kp_ypr[1] = _p ;
    kp_ypr[2] = _r ;
    return ;
}
void cpphelper_calc::set_kd(float _y, float _p, float _r){
    kd_ypr[0] = _y ;
    kd_ypr[1] = _p ;
    kd_ypr[2] = _r ;
    return ;
}
void cpphelper_calc::set_ki(float _y, float _p, float _r){
    ki_ypr[0] = _y ;
    ki_ypr[1] = _p ;
    ki_ypr[2] = _r ;
    return ;
}

void cpphelper_calc::set_gravity(float _g){
    gravity = _g ;
    return ;
}

double cpphelper_calc::get_height(){
    return height ;
    //return v_z ;
}

} //namespace n_cpphelper_calc