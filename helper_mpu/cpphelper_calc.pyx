
# distutils: language=c++
# distutils: sources=cpphelper_calc_.cpp

cdef extern from "cpphelper_calc_.h" namespace "n_cpphelper_calc" :
    cdef cppclass cpphelper_calc:
        cpphelper_calc() except +
        void update(    unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4, unsigned char d5, unsigned char d6, 
                        unsigned char d7, unsigned char d8,
                        unsigned char d9, unsigned char d10, unsigned char d11, unsigned char d12, unsigned char d13, unsigned char d14  )
        void control(float _throttle, float _target_y, float _target_p, float _target_r)
        void set_bias_a(float _bx, float _by, float _bz)
        void set_bias_g(float _bx, float _by, float _bz)
        float get_q(int _qi)
        float get_ypr_y()
        float get_ypr_p()
        float get_ypr_r()
        float get_a(int _i)
        float get_g(int _i)
        float get_ypr(int _i)
        float get_m_power(int _i)
        double get_height()
        void set_kp(float _y, float _p, float _r)
        void set_kd(float _y, float _p, float _r)
        void set_ki(float _y, float _p, float _r)
        void set_gravity(float _g)


cdef class CalcHelper(object) :
    cdef cpphelper_calc* thisptr
    
    def __cinit__(self):
        self.thisptr = new cpphelper_calc()
    
    def __dealloc__(self):
        del self.thisptr
    
    def update(self, unsigned char d0, unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4, unsigned char d5, unsigned char d6, unsigned char d7, unsigned char d8,
                 unsigned char d9, unsigned char d10, unsigned char d11, unsigned char d12, unsigned char d13, unsigned char d14) :
        self.thisptr.update(d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14)
    
    def control(self, float _throttle, float _target_y, float _target_p, float _target_r) :
        self.thisptr.control(_throttle, _target_y, _target_p, _target_r)
    
    def set_bias_a(self, float _bx, float _by, float _bz):
        self.thisptr.set_bias_a(_bx, _by, _bz)
    
    def set_bias_g(self, float _bx, float _by, float _bz):
        self.thisptr.set_bias_g(_bx, _by, _bz)
    
    def get_q(self, int _qi):
        return self.thisptr.get_q(_qi)
    
    def get_ypr_y(self):
        return self.thisptr.get_ypr_y()
    
    def get_ypr_p(self):
        return self.thisptr.get_ypr_p()
    
    def get_ypr_r(self):
        return self.thisptr.get_ypr_r()
    
    def get_a(self, int _i):
        return self.thisptr.get_a(_i)
    
    def get_g(self, int _i):
        return self.thisptr.get_g(_i)
    
    def get_ypr(self, int _i):
        return self.thisptr.get_ypr(_i)
    
    def get_m_power(self, int _i):
        return self.thisptr.get_m_power(_i)
    
    def get_height(self):
        return self.thisptr.get_height()
    
    def set_kp(self, float _y, float _p, float _r):
        self.thisptr.set_kp(_y, _p, _r)
    
    def set_kd(self, float _y, float _p, float _r):
        self.thisptr.set_kd(_y, _p, _r)
    
    def set_ki(self, float _y, float _p, float _r):
        self.thisptr.set_ki(_y, _p, _r)
    
    def set_gravity(self, float _g):
        self.thisptr.set_gravity(_g)