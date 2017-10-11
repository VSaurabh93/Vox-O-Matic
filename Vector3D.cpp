#include "Tolerance.h"
#include "Vector3D.h"

using namespace GeometryCore;

Vector3D::Vector3D(double _x, double _y, double _z)						//!< constructor
{
	mPosition[0] = _x;
	mPosition[1] = _y;
	mPosition[2] = _z;
}
Vector3D::Vector3D(double _position[])									//!< constructor
{
	mPosition[0] = _position[0];
	mPosition[1] = _position[1];
	mPosition[2] = _position[2];
}
Vector3D::Vector3D(const Vector3D& inVector3D)	//!< constructor
{
	mPosition[0] = inVector3D.mPosition[0];
	mPosition[1] = inVector3D.mPosition[1];
	mPosition[2] = inVector3D.mPosition[2];
}
double		Vector3D::operator*				(const	Vector3D&	inVector)		const
{
	return (mPosition[0] * inVector.mPosition[0] +
		mPosition[1] * inVector.mPosition[1] +
		mPosition[2] * inVector.mPosition[2]);
}
Vector3D	Vector3D::operator*				(const	int		val)			const
{
	return Vector3D(val * mPosition[0],
		val * mPosition[1],
		val * mPosition[2]);
}
Vector3D	Vector3D::operator*				(const double	val)			const
{
	return Vector3D(val * mPosition[0],
		val * mPosition[1],
		val * mPosition[2]);
}
Vector3D	Vector3D::operator/				(const int		val)			const
{
	return Vector3D(mPosition[0] / val,
		mPosition[1] / val,
		mPosition[2] / val);
}
Vector3D	Vector3D::operator/				(const double	val)			const
{
	return Vector3D(mPosition[0] / val,
		mPosition[1] / val,
		mPosition[2] / val);
}
Vector3D	Vector3D::operator-				()									const
{
	return Vector3D(-mPosition[0], -mPosition[1], -mPosition[2]);
}
Vector3D	Vector3D::operator+				(const Vector3D&	inVector)		const
{
	return Vector3D(mPosition[0] + inVector.mPosition[0],
		mPosition[1] + inVector.mPosition[1],
		mPosition[2] + inVector.mPosition[2]);
}
Vector3D	Vector3D::operator-				(const Vector3D&	inVector)		const
{
	return Vector3D(mPosition[0] - inVector.mPosition[0],
		mPosition[1] - inVector.mPosition[1],
		mPosition[2] - inVector.mPosition[2]);
}
Vector3D	Vector3D::operator^				(const Vector3D&	inVector)		const
{
	return 	Vector3D(mPosition[1] * inVector.z() - mPosition[2] * inVector.y(),
		mPosition[2] * inVector.x() - mPosition[0] * inVector.z(),
		mPosition[0] * inVector.y() - mPosition[1] * inVector.x());
}
Vector3D&	Vector3D::operator=				(const Vector3D&	inVector)
{
	mPosition[0] = inVector.mPosition[0];
	mPosition[1] = inVector.mPosition[1];
	mPosition[2] = inVector.mPosition[2];
	return *this;
}
Vector3D&	Vector3D::operator=				(const double	point[3])
{
	mPosition[0] = point[0];
	mPosition[1] = point[1];
	mPosition[2] = point[2];
	return *this;
}
Vector3D&	Vector3D::operator*=			(const int		val)
{
	mPosition[0] *= val;
	mPosition[1] *= val;
	mPosition[2] *= val;
	return *this;
}
Vector3D&	Vector3D::operator*=			(const double	val)
{
	mPosition[0] *= val;
	mPosition[1] *= val;
	mPosition[2] *= val;
	return *this;
}
Vector3D&	Vector3D::operator/=			(const int		val)
{
	mPosition[0] /= val;
	mPosition[1] /= val;
	mPosition[2] /= val;
	return *this;
}
Vector3D&	Vector3D::operator/=			(const double	val)
{
	mPosition[0] /= val;
	mPosition[1] /= val;
	mPosition[2] /= val;
	return *this;
}
Vector3D&	Vector3D::operator+=			(const Vector3D&	inVector)
{
	mPosition[0] += inVector.x();
	mPosition[1] += inVector.y();
	mPosition[2] += inVector.z();
	return *this;
}
Vector3D&	Vector3D::operator-=			(const Vector3D&	inVector)
{
	mPosition[0] -= inVector.x();
	mPosition[1] -= inVector.y();
	mPosition[2] -= inVector.z();
	return *this;
}
Vector3D&	Vector3D::operator^=			(const Vector3D&	inVector)
{
	double val1 = mPosition[1] * inVector.z() - mPosition[2] * inVector.y();
	double val2 = mPosition[2] * inVector.x() - mPosition[0] * inVector.z();
	double val3 = mPosition[0] * inVector.y() - mPosition[1] * inVector.x();
	mPosition[0] = val1;
	mPosition[1] = val2;
	mPosition[2] = val3;
	return *this;
}
bool		Vector3D::operator==			(const Vector3D&	inVector)		const
{

	return	fabs(mPosition[0] - inVector.x()) < _MFR_TOL_ &&
		fabs(mPosition[1] - inVector.y()) < _MFR_TOL_ &&
		fabs(mPosition[2] - inVector.z()) < _MFR_TOL_;
}

bool		Vector3D::operator!=			(const Vector3D&	inVector)		const
{
	return !(*this == inVector);
}

bool		Vector3D::isEqual(const Vector3D&	inVector3D,
	const Tolerance&  inTol)		const
{
	return	inTol.isEqaul(mPosition[0], inVector3D.x()) ?
		inTol.isEqaul(mPosition[1], inVector3D.y()) ?
		inTol.isEqaul(mPosition[2], inVector3D.z()) ?
		true : false : false : false;
}
bool		Vector3D::isNotEqual(const Vector3D&	inVector3D,
	const Tolerance&  inTol)		const
{
	return	inTol.isEqaul(mPosition[0], inVector3D.x()) ?
		inTol.isEqaul(mPosition[1], inVector3D.y()) ?
		inTol.isEqaul(mPosition[2], inVector3D.z()) ?
		false : true : true : true;
}
double      Vector3D::signedAngle(const Vector3D&   inVector3D)    const
{
	Vector3D va(this->x(), this->y(), this->z());
	Vector3D vb(inVector3D.x(), inVector3D.y(), inVector3D.z());

	va.normalize();
	vb.normalize();

	return atan2((va^vb).magnitude(), va*vb);
}
bool		Vector3D::normalize(const Tolerance&  inTol)
{
	double ln = magnitude();
	if (inTol.isZero(ln))		return false;
	mPosition[0] /= ln;	mPosition[1] /= ln;	mPosition[2] /= ln;
	return true;
}
void		Vector3D::doubleArray(double				outVector[3])	const
{
	outVector[0] = mPosition[0];
	outVector[1] = mPosition[1];
	outVector[2] = mPosition[2];
}
//Vector3D    Vector3D::transform(const Matrix4x4 & inMat)const
//{
//	double pos[4];
//	for (int i = 0; i < 4; i++)
//		pos[i] = mPosition[0] * inMat(i, 0) + mPosition[1] * inMat(i, 1) + mPosition[2] * inMat(i, 2) + 1.0 * inMat(i, 3);
//	return Vector3D(pos[0] / pos[3], pos[1] / pos[3], pos[2] / pos[3]);
//}
