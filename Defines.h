#pragma once

//#if defined(GeometryCore_EXPORTS) 
//#   define  GeometryCore_API  __declspec(dllexport)
//#else 
//#   define  GeometryCore_API  __declspec(dllimport)
//#endif  


#define _MFR_PI_		3.1415926535897932384626433832795
#define		M_PI		3.1415926535897932384626433832795

#define _MFR_TOL_		0.0000001
#define _MFR_LOWER_TOL_ 0.0000001

#define _MIN3_(a,b,c)		(a>b? (b>c? c:b):(a>c? c:a))
#define _MAX3_(a,b,c)		(a>b? (a>c? a:c):(b>c? b:c))
#define _MIN_(a,b)			(a>b? b:a)
#define _MAX_(a,b)			(a>b? a:b)
#define _MV_IS_POSITIVE_(a)	(a > -0.0000001) 

#define _MV_ZERO_			 0.0000001

#define _D_V3D_TOL_ Default::instance().tol(Default::VECTOR3D)

#define AREA(p1, p2, p3) fabs(0.5*((p1 - p2)^(p2 - p3)).magnitude())
#define SIGNED_AREA(V1, V2, V3) ((V2.x()-V1.x())*(V3.y()-V1.y()) - (V3.x()-V1.x())*(V2.y()-V1.y()))/2.0
#define TOL 0.0000001
