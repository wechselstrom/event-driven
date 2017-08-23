#include "iCub/eventdriven/vtsHelper.h"

namespace ev {

#ifdef TIME32BIT
    //long int vtsHelper::max_stamp = 2147483647; //2^31
    long int vtsHelper::max_stamp = 0x1FFFFFF; //2^25
#else
    long int vtsHelper::max_stamp = 16777215; //2^24
#endif

#ifdef TENBITCODEC
    double vtsHelper::tsscaler = 0.0000001;
#else
    double vtsHelper::tsscaler = 0.000000128;
#endif

#ifdef TENBITCODEC
    double vtsHelper::vtsscaler = 10000000;
#else
    double vtsHelper::vtsscaler = 7812500;
#endif

}
