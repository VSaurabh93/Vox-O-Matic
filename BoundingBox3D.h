#ifndef CCT_BOUNDINGBOX3D_H
#define CCT_BOUNDINGBOX3D_H

#include "Vector3D.h"

namespace GeometryCore
{
	class /*GeometryCore_API*/ BoundingBox3D
	{
	public:
		
		typedef enum _CornerType
		{
			C_LEFT_TOP_FRONT,		// LEFT		= max(X)	RIGHT	= min(X)
			C_LEFT_TOP_BACK,		// TOP		= max(Y)	BOTTOM	= min(Y)
			C_LEFT_BOTTOM_FRONT,	// FRONT	= max(Z)	BACK	= min(Z)
			C_LEFT_BOTTOM_BACK,
			C_RIGHT_TOP_FRONT,		
			C_RIGHT_TOP_BACK,
			C_RIGHT_BOTTOM_FRONT,
			C_RIGHT_BOTTOM_BACK,
		}CornerType;
	public:
		//------------------ Constructor & Distroctor --------------------//
		BoundingBox3D	( void ):mEmpty(true){};
		BoundingBox3D	( const Vector3D&		inPoint1,
						  const Vector3D&		inPoint2);
		BoundingBox3D	( const Vector3D&		inPoint);
		BoundingBox3D	( Vector3D				inPoint[]);
		~BoundingBox3D	( void ){};
		//----------------------------------------------------------------//
		//-------------------- Data Set/Get Function ---------------------//
		inline	bool				isEmpty		( )const{return mEmpty;}
		inline	const Vector3D&		lower		( )const{return mCorner[0];}
		inline	const Vector3D&		upper		( )const{return mCorner[1];}
				//----------------------------------------------------------------//
				//------------------------- Operators  ---------------------------//
		inline	const Vector3D&		operator[]	( int					i)const{return mCorner[i];}
				BoundingBox3D		operator+	( const BoundingBox3D&	inBox)const;
		inline	BoundingBox3D&		operator+=	( const BoundingBox3D&	inBox){return *this=*this+inBox;};
		inline	bool				operator==	( const BoundingBox3D&	inBox)const	//!< Relational Operations
				{return (mCorner[0] == inBox.mCorner[0]) && (mCorner[1] == inBox.mCorner[1]);}
		inline	bool				operator!=	( const BoundingBox3D&	inBox)const	//!< Relational Operations
				{return (mCorner[0] != inBox.mCorner[0]) || (mCorner[1] != inBox.mCorner[1]);}
				//----------------------------------------------------------------//

				//-------------------- Tolerant Methods --------------------------//
		inline	bool				isEqual		( const BoundingBox3D&	inBox,
												  const Tolerance&		inTol = Tolerance())const	//!< Relational Operations
				{return mCorner[0].isEqual(inBox.mCorner[0],inTol) && mCorner[1].isEqual(inBox.mCorner[1],inTol);}
		inline	bool				isNotEqual	( const BoundingBox3D&	inBox,
												  const Tolerance&		inTol = Tolerance())const	//!< Relational Operations
				{return mCorner[0].isNotEqual(inBox.mCorner[0],inTol) || mCorner[1].isNotEqual(inBox.mCorner[1],inTol);}
				//----------------------------------------------------------------//
				
				//------------------------- Base Methods  ------------------------//
				bool				isInClash		( const Vector3D				inPoint[],
													  const Tolerance&			inTol = Tolerance())const;
				bool				isInClash		( const Vector3D&			inPoint1,
													  const Vector3D&			inPoint2,
													  const Tolerance&			inTol = Tolerance())const;
				bool				isInClash		( const BoundingBox3D&		inBoundingBox3D,
													  const Tolerance&			inTol = Tolerance())const;
				bool				contains		( const Vector3D&			inPoint,
													  const Tolerance&			inTol = Tolerance())const;

		inline	Vector3D			diagonal		( )const{return mCorner[1]- mCorner[0];}
		inline	Vector3D			vector			( CornerType				from,
													  CornerType				to)const
				{return corner(to)-corner(from);};
		inline	Vector3D			centroid		( )const{return (mCorner[0]+mCorner[1])*0.5;}
				Vector3D			corner			( CornerType				inType)const;
				double				length			( )const{return fabs((mCorner[0] - mCorner[1]).x());}
				double				width			( )const{return fabs((mCorner[0] - mCorner[1]).y());}
				double				height			( )const{return fabs((mCorner[0] - mCorner[1]).z());}
				//----------------------------------------------------------------//

				//------------------ Self Modification Methods  ------------------//
		inline	void				add				( const BoundingBox3D&		inBox){*this+=inBox;}
				void				add				( const Vector3D&			inPoint);
				//----------------------------------------------------------------//
	private:
		Vector3D		mCorner[2];
		bool			mEmpty;
	};
}
#endif /*CCT_BOUNDINGBOX3D_H*/
