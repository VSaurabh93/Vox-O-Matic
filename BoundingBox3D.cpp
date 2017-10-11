//#include "StdAfx.h"
#include "BoundingBox3D.h"
#include "Tolerance.h"

using namespace GeometryCore;

BoundingBox3D::BoundingBox3D	( const Vector3D&		inPoint1,
								  const Vector3D&		inPoint2)
	:mEmpty(false)
{
	mCorner[0] = Vector3D(	_MIN_(inPoint1.x(),inPoint2.x()),
							_MIN_(inPoint1.y(),inPoint2.y()),
							_MIN_(inPoint1.z(),inPoint2.z()));
	mCorner[1] = Vector3D(	_MAX_(inPoint1.x(),inPoint2.x()),
							_MAX_(inPoint1.y(),inPoint2.y()),
							_MAX_(inPoint1.z(),inPoint2.z()));
}
BoundingBox3D::BoundingBox3D	( const Vector3D&		inPoint)
	:mEmpty(false){mCorner[0] = mCorner[1] = inPoint;}
BoundingBox3D::BoundingBox3D	( Vector3D				inPoint[])
	:mEmpty(false)
{
	mCorner[0] = Vector3D(	_MIN_(inPoint[0].x(),inPoint[1].x()),
							_MIN_(inPoint[0].y(),inPoint[1].y()),
							_MIN_(inPoint[0].z(),inPoint[1].z()));
	mCorner[1] = Vector3D(	_MAX_(inPoint[0].x(),inPoint[1].x()),
							_MAX_(inPoint[0].y(),inPoint[1].y()),
							_MAX_(inPoint[0].z(),inPoint[1].z()));
};
BoundingBox3D		BoundingBox3D::operator+		( const BoundingBox3D&	inBox)const
{
	if(mEmpty)			return inBox;
	if(inBox.mEmpty)	return *this;

	Vector3D minCorner = Vector3D(_MIN_(mCorner[0].x(),inBox.mCorner[0].x()),
								_MIN_(mCorner[0].y(),inBox.mCorner[0].y()),
								_MIN_(mCorner[0].z(),inBox.mCorner[0].z()));

	Vector3D maxCorner = Vector3D(_MAX_(mCorner[1].x(),inBox.mCorner[1].x()),
								_MAX_(mCorner[1].y(),inBox.mCorner[1].y()),
								_MAX_(mCorner[1].z(),inBox.mCorner[1].z()));
	return BoundingBox3D(minCorner,maxCorner);
}
bool				BoundingBox3D::isInClash		( const Vector3D&		inPoint1,
													  const Vector3D&		inPoint2,
													  const Tolerance&		inTol)const
{
	return inTol.isGreater(mCorner[0].x(),inPoint2.x())
			||inTol.isGreater(mCorner[0].y(),inPoint2.y())
			||inTol.isGreater(mCorner[0].z(),inPoint2.z())
			||inTol.isLess(mCorner[1].x(),inPoint1.x())
			||inTol.isLess(mCorner[1].y(),inPoint1.y())
			||inTol.isLess(mCorner[1].z(),inPoint1.z())?false:true;
}
bool				BoundingBox3D::isInClash		( const Vector3D			inPoint[],
													  const Tolerance&		inTol)const
{
	return inTol.isGreater(mCorner[0].x(),inPoint[1].x())
			||inTol.isGreater(mCorner[0].y(),inPoint[1].y())
			||inTol.isGreater(mCorner[0].z(),inPoint[1].z())
			||inTol.isLess(mCorner[1].x(),inPoint[0].x())
			||inTol.isLess(mCorner[1].y(),inPoint[0].y())
			||inTol.isLess(mCorner[1].z(),inPoint[0].z())?false:true;
}
bool				BoundingBox3D::isInClash		( const BoundingBox3D&	inBox,
													  const Tolerance&		inTol)const
{
	return inTol.isGreater(mCorner[0].x(),inBox.mCorner[1].x())
			||inTol.isGreater(mCorner[0].y(),inBox.mCorner[1].y())
			||inTol.isGreater(mCorner[0].z(),inBox.mCorner[1].z())
			||inTol.isLess(mCorner[1].x(),inBox.mCorner[0].x())
			||inTol.isLess(mCorner[1].y(),inBox.mCorner[0].y())
			||inTol.isLess(mCorner[1].z(),inBox.mCorner[0].z())?false:true;
}
bool				BoundingBox3D::contains			( const Vector3D&			inPoint,
													  const Tolerance&			inTol   )const
{
	return inTol.isGreater(mCorner[0].x(),inPoint.x())
			||inTol.isGreater(mCorner[0].y(),inPoint.y())
			||inTol.isGreater(mCorner[0].z(),inPoint.z())
			||inTol.isLess(mCorner[1].x(),inPoint.x())
			||inTol.isLess(mCorner[1].y(),inPoint.y())
			||inTol.isLess(mCorner[1].z(),inPoint.z())?false:true;
 
}
Vector3D				BoundingBox3D::corner			( CornerType			inType)const
{
	switch(inType)
	{
	case C_LEFT_TOP_FRONT:		return	mCorner[1];	
	case C_LEFT_TOP_BACK:		return	Vector3D(mCorner[1].x(),mCorner[1].y(),mCorner[0].z());	
	case C_LEFT_BOTTOM_FRONT:	return	Vector3D(mCorner[1].x(),mCorner[0].y(),mCorner[1].z());
	case C_LEFT_BOTTOM_BACK:	return	Vector3D(mCorner[1].x(),mCorner[0].y(),mCorner[0].z());
	case C_RIGHT_TOP_FRONT:		return	Vector3D(mCorner[0].x(),mCorner[1].y(),mCorner[1].z());
	case C_RIGHT_TOP_BACK:		return	Vector3D(mCorner[0].x(),mCorner[1].y(),mCorner[0].z());
	case C_RIGHT_BOTTOM_FRONT:	return	Vector3D(mCorner[0].x(),mCorner[0].y(),mCorner[1].z());
	case C_RIGHT_BOTTOM_BACK:	return	mCorner[0];
	}
	return Vector3D();
}

void				BoundingBox3D::add				( const Vector3D&		inPoint)
{
	if(mEmpty)
	{
		mCorner[0] = mCorner[1] = inPoint;
		mEmpty = false;
		return;
	}
	mCorner[0]	= Vector3D(	_MIN_(mCorner[0].x(),inPoint.x()),
							_MIN_(mCorner[0].y(),inPoint.y()),
							_MIN_(mCorner[0].z(),inPoint.z()));

	mCorner[1]	= Vector3D(	_MAX_(mCorner[1].x(),inPoint.x()),
							_MAX_(mCorner[1].y(),inPoint.y()),
							_MAX_(mCorner[1].z(),inPoint.z()));
}
