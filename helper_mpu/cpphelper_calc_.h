#ifndef CPPHELPER_CALC_H
#define CPPHELPER_CALC_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <syslog.h>

//#define GYROMEASERROR 3.14159265358979f * (5.0f / 180.0f)
//#define BETA ( sqrt(3.0f / 4.0f) * GYROMEASERROR )

namespace n_cpphelper_calc{
    class cpphelper_calc{
    public:
        cpphelper_calc() ;
        ~cpphelper_calc(){
            syslog(LOG_INFO, "cpphelper terminated\r\n") ;
            closelog() ;
        }
        void update( unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4, unsigned char d5, unsigned char d6,
                     unsigned char d7, unsigned char d8, unsigned char d9, unsigned char d10, unsigned char d11, unsigned char d12, unsigned char d13, unsigned char d14  ) ;
        void set_bias_a(float _bx, float _by, float _bz) ;
        void set_bias_g(float _bx, float _by, float _bz) ;
        float get_q(int _qi) ;
        float get_a(int _i) ;
        float get_g(int _i) ;
        float get_ypr_y() ;
        float get_ypr_p() ;
        float get_ypr_r() ;
        float get_deltat() ;
        //future release
        //void set_kp(float, float, float) ;
        //void set_kd(float, float, float) ;
        //void set_ki(float, float, float) ;
        
    private :
        float a_x, a_y, a_z; // accelerometer measurements
        float w_x, w_y, w_z; // gyroscope measurements in rad/s
        float SEq_1, SEq_2, SEq_3, SEq_4; 
        float angle_ypr[3] ;
        float bias_a[3], bias_g[3] ;
        bool flag_first;
        struct timeval n, o ;
        float deltat ;
        const float beta ;
    };
}

#endif