
# distutils: language=c++
# distutils: sources=cpphelper_calc_.cpp

cdef extern from "cpphelper_calc_.h" namespace "n_cpphelper_calc" :
    cdef cppclass cpphelper_calc:
        cpphelper_calc() except +
        void update(    unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4, unsigned char d5, unsigned char d6, 
                        unsigned char d7, unsigned char d8,
                        unsigned char d9, unsigned char d10, unsigned char d11, unsigned char d12, unsigned char d13, unsigned char d14  )
        void set_bias_a(float _bx, float _by, float _bz)
        void set_bias_g(float _bx, float _by, float _bz)
        float get_q(int _qi)
        float get_ypr_y()
        float get_ypr_p()
        float get_ypr_r()
        float get_a(int _i)
        float get_g(int _i)
        float get_deltat()


cdef class CalcHelper(object) :
    cdef cpphelper_calc* thisptr
    
    def __cinit__(self):
        self.thisptr = new cpphelper_calc()
    
    def __dealloc__(self):
        del self.thisptr
    
    def update(self, unsigned char d0, unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4, unsigned char d5, unsigned char d6, unsigned char d7, unsigned char d8,
                 unsigned char d9, unsigned char d10, unsigned char d11, unsigned char d12, unsigned char d13, unsigned char d14) :
        self.thisptr.update(d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14)
    
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
    
    def get_deltat(self) :
        return self.thisptr.get_deltat()