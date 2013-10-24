#ifndef PROFILERTOOL_H_INCLUDED
#define PROFILERTOOL_H_INCLUDED
#include <ctime>

class ProfilerTool {
 private:
     clock_t _tic=0,_toc=0,_t;

 public:
    ProfilerTool()
    {
        _t = 0;
    }

    void tic()
    {
        _tic = clock();
    }

    void toc()
    {
        _toc = clock();
        _t += _toc - _tic;
    }

    void reset()
    {
        _t = 0;
    }

    clock_t n_clocks()
    {
        return _t;
    }

    double ms()
    {
        double ms = (double)n_clocks()/CLOCKS_PER_SEC*1000.0;
        return ms;
    }
};

#endif // PROFILERTOOL_H_INCLUDED
