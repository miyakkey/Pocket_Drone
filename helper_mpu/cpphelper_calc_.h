#ifndef CPPHELPER_CALC_H
#define CPPHELPER_CALC_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <syslog.h>

//#define GYROMEASERROR 3.14159265358979f * (5.0f / 180.0f)
//#define BETA ( sqrt(3.0f / 4.0f) * GYROMEASERROR )

// after change .h, .cpp, .pyx file in this directory ... 
// (1) run 'python3 setup.py build_ext --inplace' and compile
// (2) copy '~.so' file to your working directory
// (3) rename 'example.cython-version.so' to 'example.so'

namespace n_cpphelper_calc{
    class cpphelper_calc{
    public:
        cpphelper_calc() ;
        ~cpphelper_calc(){
            syslog(LOG_DEBUG, "cpphelper terminated\r\n") ;
            closelog() ;
        }
        void update( unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4, unsigned char d5, unsigned char d6,
                     unsigned char d7, unsigned char d8, unsigned char d9, unsigned char d10, unsigned char d11, unsigned char d12, unsigned char d13, unsigned char d14  ) ;
        void control(float _throttle, float _target_y, float _target_p, float _target_r) ;//new
        void set_bias_a(float _bx, float _by, float _bz) ;
        void set_bias_g(float _bx, float _by, float _bz) ;
        float get_q(int _qi) ;
        float get_a(int _i) ;
        float get_g(int _i) ;
        float get_ypr(int _i) ;
        float get_m_power(int _i) ;
        void set_kp(float _y, float _p, float _r) ;
        void set_kd(float _y, float _p, float _r) ;
        void set_ki(float _y, float _p, float _r) ;
        float get_ypr_y() ;
        float get_ypr_p() ;
        float get_ypr_r() ;
        void set_gravity(float _g) ;//new
        double get_height() ;//new
        
    private :
        float a_x, a_y, a_z; // accelerometer measurements
        float w_x, w_y, w_z; // gyroscope measurements in rad/s
        float SEq_1, SEq_2, SEq_3, SEq_4; 
        float angle_ypr[3] ;
        float bias_a[3], bias_g[3] ;
        float kp_ypr[3], kd_ypr[3], ki_ypr[3] ;
        bool flag_first, flag_first_m;
        struct timeval n, o, n_m, o_m ;
        float mortor_power[4], control_ypr[3] ;
        float delta_ypr[3], delta_ypr_delta[3], delta_ypr_old[3], delta_ypr_integrate[3] ;
        const float beta ;
        float gravity ;
        double height, v_z ;
    };
}

#endif