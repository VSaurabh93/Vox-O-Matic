#ifndef VECTOR3D_H
#define VECTOR3D_H

#include "Tolerance.h"
//#include "Matrix4x4.h"
//-------------------------------------------------------------------------------------------------------------------
/**
 * @file 		Vector.h
 * @brief 		It provides functionalities related to Vector in 3D space.
 * @details		It provides functionalities related to Vector in 3D space. 
 *
 */
//-------------------------------------------------------------------------------------------------------------------

namespace GeometryCore
{ 
	
	class Matrix4x4;

	class /*GeometryCore_API*/ Vector3D
	{
	public:
		//------------------ constructor & destructor --------------------//
		Vector3D( double _x = 0,double _y = 0,double _z = 0);						//!< constructor
		Vector3D( double _position[]);									//!< constructor
		Vector3D( const Vector3D& inVector3D);	//!< constructor
		~Vector3D() {}													//!< destructor
		//----------------------------------------------------------------//

		//-------------------- Data Set/Get Function -----------------------//
		//////////////////////////////////////////////////////////////////////
		// THESE FUNC. ARE KEPT INLINE intentionally TO ENHANCE PERFORMANCE //
		//////////////////////////////////////////////////////////////////////

		inline	void	set					( double _x,
											  double _y,
											  double _z)	
		{mPosition[0]=_x;mPosition[1]=_y;mPosition[2]=_z;}	//!< sets vector positon								
		/*inline*/	double	x					( )	const		
				{
					return mPosition[0];
				}						//!< return i componant of vector
		/*inline*/	double&	x					( )				
				{
					return mPosition[0];
				}						//!< return reference of i componant of vector
		inline	double	y					( )	const		
				{return mPosition[1];}						//!< return j componant of vector
		inline	double&	y					( )				
				{return mPosition[1];}						//!< return reference of j componant of vector
		inline	double	z					( )	const		
				{return mPosition[2];}						//!< return k componant of vector
		inline	double&	z					( )				
				{return mPosition[2];}						//!< return reference of k componant of vector
		inline	double*	array				( )				
				{return mPosition;}							//!< return vector position
				//----------------------------------------------------------------//

				//------------------------- Operators  ---------------------------//
		inline	double		operator[]			( int i)const	{return mPosition[i];}			
		inline	double&		operator[]			( int i)		{return mPosition[i];}	
				double		operator*			( const Vector3D&	inVector)		const;	//!< inner dot product
				Vector3D	operator*			( const int			val)			const;	//!< Scalar Multiplication with int
				Vector3D	operator*			( const double		val)			const;	//!< Scalar Multiplication with double
				Vector3D	operator/			( const int			val)			const;	//!< Scalar Division with int
				Vector3D	operator/			( const double		val)			const;	//!< Scalar Division with double
				Vector3D	operator-			( )									const;	//!< unary minus
				Vector3D	operator+			( const Vector3D&	inVector)		const;	//!< vector add
				Vector3D	operator-			( const Vector3D&	inVector)		const;	//!< vector subtract
				Vector3D	operator^			( const Vector3D&	inVector)		const;	//!< 3D exterior cross product
				Vector3D&	operator=			( const Vector3D&	inVector);				//!< vector assignment
				Vector3D&	operator=			( const double		point[3]);				//!< vector assignment
				Vector3D&	operator*=			( const int			val);					//!< vector scalar mult
				Vector3D&	operator*=			( const double		val);					//!< vector scalar mult
				Vector3D&	operator/=			( const int			val);					//!< vector scalar div
				Vector3D&	operator/=			( const double		val);					//!< vector scalar div
				Vector3D&	operator+=			( const Vector3D&	inVector);				//!< vector increment
				Vector3D&	operator-=			( const Vector3D&	inVector);				//!< vector decrement
				Vector3D&	operator^=			( const Vector3D&	inVector);				//!< 3D exterior cross product
				bool		operator==			( const Vector3D&	inVector)		const;	//!< Relational Operations
				bool		operator!=			( const Vector3D&	inVector)		const;	//!< Relational Operations
				//----------------------------------------------------------------//

				//-------------------- Tolerant Methods --------------------------//
				bool		isEqual				( const Vector3D&	inVector3D,
												  const Tolerance&  inTol = Tolerance())	const;	//!< Relational Operations

				bool		isNotEqual			( const Vector3D&	inVector3D,
												  const Tolerance&  inTol = Tolerance())	const;	//!< Relational Operations

				double      signedAngle		    ( const Vector3D&   inVector3D)    const;
				//----------------------------------------------------------------//
				
				//------------------------- Base Methods  ------------------------//
		inline	double		magnitude			( )											const	//!< vector length
				{return sqrt(magnitudeSqr( ));}			
				//----------------------------------------------------------------//
			
				//------------------ Self Modification Methods  ------------------//
				bool		normalize			( const Tolerance&		inTol = Tolerance());	//!< make base vector a unit vector
				//----------------------------------------------------------------//

				//--------------------- Performance Methods  ---------------------//
		inline	double		magnitudeSqr		( )									const	//!< vector length squared (faster)
				{return (mPosition[0]*mPosition[0] + mPosition[1]*mPosition[1] + mPosition[2]*mPosition[2]);}
				void		doubleArray			( double				outVector[3])	const;	//!< provide static array of double 
				//----------------------------------------------------------------//
		inline	double		distance			( const Vector3D & inPoint)			const
				{return (*this - inPoint).magnitude();}

		    	//Vector3D		transform       ( const Matrix4x4 & mat)			const;

	private:
		double	mPosition[3];
	};

	inline Vector3D	operator*	( const int		val,	
								  const Vector3D&	inVector)				//!< Scalar Multiplication with Int
	{return inVector*val;}

	inline Vector3D	operator*	( const double	val,
								  const Vector3D&	inVector)				//!< Scalar Multiplication with double
	{return inVector*val;}

	
}

#endif /*VECTOR3D_H*/
