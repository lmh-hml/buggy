#ifndef UTIL_H
#define UTIL_H

#include <cstdlib>
#include <cstdio> 

#define ABS(n) ( (n<0)? -n:n )
#define PI     3.14159265359
#define signOf(n) ( (n<0) ? -1.0:1.0 )
#define ifNotZeroGetSign(n) ( (n==0) ? 0 : signOf(n) )




inline double degToRadians( double deg)
{
	return (PI *deg) /180.0;
}

inline double radsToDegrees( double rads )
{
	return (180.0*rads) /PI;
}


#endif
