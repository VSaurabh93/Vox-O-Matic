#pragma once
#include<unordered_set>
#include<unordered_map>
#include<tuple>

#include"BoundingBox3D.h"

using namespace GeometryCore;

namespace Voxellizer
{

	enum VoxCoordinate
	{
		TOP_FACE_TOP_RIGHT = 0,
		TOP_FACE_TOP_LEFT,
		TOP_FACE_BOTTOM_LEFT,
		TOP_FACE_BOTTOM_RIGHT,
		BOTTOM_FACE_TOP_RIGHT,
		BOTTOM_FACE_TOP_LEFT,
		BOTTOM_FACE_BOTTOM_LEFT,
		BOTTOM_FACE_BOTTOM_RIGHT		
	} ;
	
	
	class Voxel
	{
		Vector3D* mCentre;
		Vector3D* mVoxelSize;
		
	public:
		Voxel(Vector3D* voxelCenterPoint, Vector3D* voxelSize);
		Vector3D getCoordinate(VoxCoordinate option);
		std::vector<Vector3D> getNeighbourVertices(VoxCoordinate option);
		void getTriangleFacet(int facetNo, Vector3D& v1, Vector3D& v2, Vector3D& v3);
		void getCoordinates(std::vector<Vector3D>& coordinates);
	} ;
	
	
	class Node
	{
		
		Vector3D  mVoxelSize;
		Vector3D* mMinVoxSize;
		Vector3D mCentre;
		bool mHasChildren;
	public:
	
		Node* parent;
		std::vector<Node*> children;
	
		Node::Node(const Vector3D& centre,const Vector3D& voxelSize,Vector3D* minVoxelSize) 
					: mCentre(centre), mVoxelSize(voxelSize), mMinVoxSize(minVoxelSize), 
					mHasChildren(false),children(8,nullptr) {}
		bool hasChildren();
		bool subdivide();
		bool insertTriangle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3);
		bool isIntersectingBBox(const Vector3D& inMinPoint, const Vector3D& inMaxPoint);
		bool isIntersectingTriangle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3);
		bool isRayTriangleIntersecting(const Vector3D &orig,const Vector3D &dir,const Vector3D &v1,const Vector3D &v2,const Vector3D &v3, double &t, double &u, double &v);
		const Vector3D* getCentrePoint() const;
		Voxel getVoxel();
		void getVoxelBBox(Vector3D& outMinPoint , Vector3D& outMaxPoint);
	
	} ;
	
	
	class VoxelOctree
	{
	public:
		Node* Root;
		VoxelOctree(Vector3D& centre, Vector3D& voxelSize, Vector3D& minVoxelSize);
		void clear();
		
		bool voxelizeTriangle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3);
		void generateMesh();
	};

}




