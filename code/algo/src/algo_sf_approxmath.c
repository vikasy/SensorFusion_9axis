
/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "algo_sf_approxmath.h"

static float cos_1stquad(float x_rad);

static float sin_1stquad(float x_rad);

static float atan_below15(float x);


// Input: 0 <= x_rad <= Pi/2
// Output: 0.0 <= cosx <= 1.0
static float cos_1stquad(float x_rad)  
{
#define c1  0.999999953464
#define c2 -0.4999999053455
#define c3  0.0416635846769
#define c4 -0.0013853704264
#define c5  0.000023233
//#define c1  0.99999999999925182
//#define c2 -0.49999999997024012
//#define c3  0.041666666473384543
//#define c4 -0.001388888418000423
//#define c5  0.0000248010406484558
//#define c6  0.0000002752469638432
//#define c7  0.0000000019907856854

    float y, x2; 
 
    x2 = x_rad*x_rad;
    y = (c1+x2*(c2+x2*(c3+x2*(c4+c5*x2))));
	
	return y;
	
#undef c1
#undef c2
#undef c3
#undef c4
#undef c5
} 

 
// Input: 0 <= x_rad <= Pi/2
// Output: 0.0 <= sinx <= 1.0 
static float sin_1stquad(float x_rad)  
{	
    float y;
	y = cos_1stquad(x_rad);
    
	return sqrt(1 - y*y);

} 


// Input: -Inf < x_rad < Inf
// Output: -1.0 <= sinx, cosx <= 1.0
void sincos_approx( float x_rad, float *psinx, float *pcosx )
{
	
	float xin;
	float sign_c, sign_s;
	
	sign_c = 1.0;
	sign_s = 1.0;
	
	// adjust input angle to bring it within 1st quadrant
	xin = fmodf(x_rad, PI_TIMES_2);
	if (xin < 0) {
		xin = -xin;
		sign_s = -1.0;
	}
    if( xin >= PI_OVER_2 && xin < PI ) {
		xin = PI - xin;
		sign_c = -1.0*sign_c;
	}
    else if( xin >= PI && xin < (PI + PI_OVER_2) ) {
		xin = xin - PI;
		sign_c = -1.0*sign_c; 
		sign_s = -1.0*sign_s;
	}
    else if( xin >= (PI + PI_OVER_2) && xin < PI_TIMES_2 ) {
		xin = PI_TIMES_2 - xin;
		sign_s = -1.0*sign_s;
	}
    
	*pcosx = sign_c*cos_1stquad( xin );
	*psinx = sign_s*sin_1stquad( xin );
	
}


// Input: -TAN15DEG <= x <= TAN15DEG
// Output: -15 <= atanx <= 15 deg
// Max error of about 2e-7 deg
static float atan_below15(float x)  
{
#define c1  -0.3333333333297
#define c2   0.1999999963100
#define c3  -0.1428565386578
#define c4   0.1110747892853
#define c5  -0.0899143448436
 	
    float y, x2; 
 
    x2 = x*x;
    y = x*(1 + x2*(c1 + x2*(c2 + x2*(c3 + x2*(c4 + x2*c5)))));
	
	return y;
	
#undef c1
#undef c2
#undef c3
#undef c4
#undef c5
}


// Input: -1.0 <= x <= 1.0
// Output: -90 <= asinx <= 90 deg
float asin_approx( float x )
{
	float asinx_deg = 0.0;

	if (x >= 1.0) 
		asinx_deg = 90.0;
	else if (x <= -1.0) 
		asinx_deg = -90.0;
	else
		asinx_deg = atan_approx( x/sqrt(1.0 - x*x) );

	return asinx_deg;
	
}


// Input: -1.0 <= x <= 1.0
// Output: 0 <= acosx <= 180 deg
float acos_approx( float x )
{	
	float acosx_deg = 0.0;

	if (x == 0.0) {
		acosx_deg = 90.0;
	}
	else if (x >= 1.0) {
		acosx_deg = 0.0;
	}
	else if (x <= -1.0) {
		acosx_deg = 180.0;
	}
	else if (x > 0.0) {
		acosx_deg = atan_approx( sqrt(1.0 - x*x)/x );
	}
	else {
		acosx_deg = atan_approx( sqrt(1.0 - x*x)/x ) + 180.0;
	}

	return acosx_deg;

}


// Input: -Inf < x < Inf
// Output: -90 < atanx < 90 deg
float atan_approx( float x )
{
	float xin;
	float sign;
	float shift1, shift2;
	float y, atanx_deg;
	
	atanx_deg = 0.0;
	xin = x;
	sign = 1.0;
	shift1 = 0.0;
	shift2 = 0.0;
	
	if( xin == 0.0 ) {
		atanx_deg = 0.0;
		return atanx_deg;
	}
	else if( xin < 0.0 ) {
	    xin = -xin;
		sign = -1.0;
	}
	
	if( xin > 1.0 ) {
		xin = 1.0/xin;
		shift1 = sign*PI_OVER_2;
		sign = -1.0*sign;		
	}
	
	if( xin > TAN15DEG ) {
		xin = (xin - TAN30DEG)/(1.0 + TAN30DEG*x);
		shift2 = PI_OVER_6;
	}
	y = atan_below15( xin );
	y = sign*(y + shift2) + shift1;
	atanx_deg = y*RAD2DEG;

	return atanx_deg;
	
}


// Input: -Inf < x, y < Inf
// Output: -180 < atan2x <= 180 deg
float atan2_approx( float y, float x )
{
	float atan2_deg = 0.0;

	if( x > 0.0 ) {
		atan2_deg = atan_approx(y/x);
	}
	else if( x == 0.0 ) {
		if( y < 0.0 ) {
		    atan2_deg = -90.0;
		}
		else if( y > 0.0 ) {
		    atan2_deg = 90.0;
	    }
		else {
		    atan2_deg = 0.0;
		} 			
	}
	else if( y >= 0.0 ) {
		atan2_deg = atan_approx(y/x) + 180.0;
	}
	else if( y < 0.0 ) {
		atan2_deg = atan_approx(y/x) - 180.0;
	}

	return atan2_deg;
	
}


