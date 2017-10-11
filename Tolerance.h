#ifndef TOLERANCE_H
#define TOLERANCE_H

#include "Defines.h"
#include<cmath>
//-------------------------------------------------------------------------------------------------------------------
/**
 * @file 		Tolerance.h
 * @brief 		It provides functionalities related to value tolerences.
 * @details		It provides functionalities related to value tolerences. 
 *
 */
//-------------------------------------------------------------------------------------------------------------------

namespace GeometryCore
{
	class /*GeometryCore_API*/  Tolerance
	{
	public:
		
		Tolerance(double        inTol = 0.0000001,
                                 double        inZero = 0.0000000000001,
                                 size_t        inHugeUInt = 0xF4240,//1000000
                                 size_t        inInfUInt = 0xFFFFFFFF)
                                 : mUseTol(false),
                                       mTol(inTol),
                                       mZero(inZero),
                                       mHuge(10.0/mTol),
                                       mInf(10.0/mZero),
                                       mHugeUInt(inHugeUInt),
                                       mInfUInt(inInfUInt){}
		~Tolerance(void){};
	
		//-------------------- Data Set/Get Function ---------------------//
				void		setTolFlag	( bool inFalg){mUseTol = inFalg;}
		inline	bool		isValid		( )const{return mHuge>mTol && mInf>mZero;}	
		inline	bool&		useTol		( ){return mUseTol;}
		inline	bool		useTol		( )const{return mUseTol;}
		inline	double&		tol			( ){return mTol;}
		inline	double		tol			( )const{return mTol;}
		inline	double&		zero		( ){return mZero;}
		inline	double		zero		( )const{return mZero;}
		inline	double&		huge		( ){return mHuge;}
		inline	double		huge		( )const{return mHuge;}
		inline	double&		inf			( ){return mInf;}
		inline	double		inf			( )const{return mInf;}
		inline	size_t&		hugeUInt	( ){return mHugeUInt;}
		inline	size_t		hugeUInt	( )const{return mHugeUInt;}
		inline	size_t&		infUInt		( ){return mInfUInt;}
		inline	size_t		infUInt		( )const{return mInfUInt;}
		inline	bool		isZero		( double	val)const{return fabs(val) < lowest();}
		inline	bool		isPositiveSqr	( double	val)const{return val > (lowest()*lowest());}
		inline	bool		isPositive	( double	val)const{return val > lowest();}
		inline	bool		isNegative	( double	val)const{return val < -lowest();}
		inline	bool		isLess		( double	lf,
										  double	rt)const{return (lf + lowest())<rt;}
		inline	bool		isGreater	( double	lf,
										  double	rt)const{return (lf - lowest())>rt;}
		inline	bool		isEqaul		( double	lf,
										  double	rt)const{return fabs(rt-lf) < lowest();}
		inline	double		lowest		( )const{return mUseTol?mTol:mZero;}
		inline	double		highest		( )const{return mUseTol?mHuge:mInf;}
		inline	size_t		highestUInt	( )const{return mUseTol?mHugeUInt:mInfUInt;}
				//----------------------------------------------------------------//
	private:
		bool		mUseTol;
		double		mTol;
		double		mZero;
		double		mHuge;
		double		mInf;
		size_t		mHugeUInt;
		size_t		mInfUInt;
	};
}
#endif //TOLERANCE_H
