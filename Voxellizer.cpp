#include<fstream>
#include"Vector3D.h"														  
#include"Voxellizer.h"

#define PRIME		2147483647 //2^31 - 1

#define NODE_ALLOC(PTR,CENTER,VOXELSIZE,MINVOXSIZE) {if(!PTR){PTR = new Node(CENTER,VOXELSIZE,MINVOXSIZE); if(!PTR)throw("Could not allocate memory!");} 

using namespace Voxellizer;

static size_t hash(const Vector3D* point)
{
	std::hash<double> hashD;
	size_t n = hashD(point->x()*PRIME*PRIME + point->y()*PRIME +  point->z());
	return n;
}
struct NodeHash 
{
	size_t operator() ( const Node * v) const  
	{ 
		return hash(v->getCentrePoint());
	}
};
struct NodeComparator 
{
	bool operator()( const Node* a, const Node* b) const 
	{
		return a->getCentrePoint() == b->getCentrePoint();
	}
};
struct PointHash 
{
	size_t operator() ( const Vector3D& v) const  
	{ 
		return hash(&v);
	}
};
struct PointComparator 
{
	bool operator()( const Vector3D& a, const Vector3D& b) const 
	{
		return a == b;
	}
};


typedef std::unordered_set<Node*,NodeHash,NodeComparator> NodeTable;
typedef std::unordered_map<Vector3D,int,PointHash,PointComparator> VertexTable;

NodeTable Voxeltable;
VertexTable vertexTable;
std::vector<Node*> allocatedNodesList;


int VoxelTessData[36] = {
	0, 1, 2,
    0, 2, 3,
    3, 2, 6,
    3, 6, 7,
    0, 7, 4,
    0, 3, 7,
    4, 7, 5,
    7, 6, 5,
    0, 4, 5,
    0, 5, 1,
    1, 5, 6,
    1, 6, 2,
};

VoxelOctree::VoxelOctree(Vector3D& centre, Vector3D& voxelSize, Vector3D& minVoxelSize)
{
	//check voxelSize and minVoxelSize are > 0
	Root = new Node(centre,voxelSize,&minVoxelSize) ;
	Root->subdivide();
}

void VoxelOctree::clear()
{
	vertexTable.clear();
	Voxeltable.clear();

	for (auto item = allocatedNodesList.begin(); item != allocatedNodesList.end();item++)
	{
		if(!(*item))
			delete *item;
	}
}
	
bool VoxelOctree::voxelizeTriangle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3)
{
	size_t voxelTableSize = Voxeltable.size();
	Root->insertTriangle(v1,v2,v3);
	if(Voxeltable.size() > voxelTableSize)
		return true;
	return false;
}

void VoxelOctree::generateMesh()
{
	int index = 0;
	std::vector<int> indices;

	std::ofstream off("../voxel.obj",std::ios::out);

	for (auto i = Voxeltable.begin(); i!= Voxeltable.end() ; i++)
	{
		Voxel voxelNode = (*i)->getVoxel();

		for (size_t i = 0; i < 12; i++)
		{
			Vector3D v1,v2,v3;
			voxelNode.getTriangleFacet(i,v1,v2,v3);
			if(vertexTable.find(v1) == vertexTable.end())
			{
				off << "v " << v1.x() << " " << v1.y() << " " << v1.z() <<std::endl;
				vertexTable[v1] = ++index;
			}
				indices.push_back(vertexTable[v1]);

			if(vertexTable.find(v2) == vertexTable.end())
			{
				off << "v " << v2.x() << " " << v2.y() << " " << v2.z() <<std::endl;
				vertexTable[v2] = ++index;	
			}
				indices.push_back(vertexTable[v2]);

			if(vertexTable.find(v3) == vertexTable.end())
			{
				off << "v " << v3.x() << " " << v3.y() << " " << v3.z() <<std::endl;
				vertexTable[v3] = ++index;
			}
				indices.push_back(vertexTable[v3]);
			
		}
	}

	

	for (auto i = 0; i < indices.size(); i+=3)
	{
		off << "f " << indices[i] << " " << indices[i+1] << " " << indices[i+2] <<std::endl;
	}


}

bool Node::hasChildren()
{
	return mHasChildren;
}
	
bool Node::subdivide()
{
	try
	{	if(children[0] == nullptr)
		children[0] = new Node(mCentre + Vector3D( mVoxelSize.x()/4,  mVoxelSize.y()/4,  mVoxelSize.z()/4), mVoxelSize/2, mMinVoxSize);
																  					 					
		if(children[1] == nullptr)								  					 					
		children[1] = new Node(mCentre + Vector3D(-mVoxelSize.x()/4,  mVoxelSize.y()/4,  mVoxelSize.z()/4), mVoxelSize/2, mMinVoxSize);
																  					 					
		if(children[2] == nullptr)								  					 					
		children[2] = new Node(mCentre + Vector3D(-mVoxelSize.x()/4, -mVoxelSize.y()/4,  mVoxelSize.z()/4), mVoxelSize/2, mMinVoxSize);
																  					 					
		if(children[3] == nullptr)								  					 					
		children[3] = new Node(mCentre + Vector3D( mVoxelSize.x()/4, -mVoxelSize.y()/4,  mVoxelSize.z()/4), mVoxelSize/2, mMinVoxSize);
																  					 					
		if(children[4] == nullptr)								  					 					
		children[4] = new Node(mCentre + Vector3D( mVoxelSize.x()/4,  mVoxelSize.y()/4, -mVoxelSize.z()/4), mVoxelSize/2, mMinVoxSize);
																  					 					
		if(children[5] == nullptr)								  					 					
		children[5] = new Node(mCentre + Vector3D(-mVoxelSize.x()/4,  mVoxelSize.y()/4, -mVoxelSize.z()/4), mVoxelSize/2, mMinVoxSize);
																  					 					
		if(children[6] == nullptr)								  					 					
		children[6] = new Node(mCentre + Vector3D(-mVoxelSize.x()/4, -mVoxelSize.y()/4, -mVoxelSize.z()/4), mVoxelSize/2, mMinVoxSize);
																  					 					
		if(children[7] == nullptr)								  					 					
		children[7] = new Node(mCentre + Vector3D( mVoxelSize.x()/4, -mVoxelSize.y()/4, -mVoxelSize.z()/4), mVoxelSize/2, mMinVoxSize);

		for (int i = 0; i < children.size(); i++)
		{
			allocatedNodesList.push_back(children[i]);
		}

		mHasChildren = true;
		return true;
	}
	catch(std::exception ex)
	{
		return false;
	}
}

bool Node::insertTriangle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3)
{
	BoundingBox3D triangleBBox;
	triangleBBox.add(v1);
	triangleBBox.add(v2);
	triangleBBox.add(v3);

	if(fabs(mVoxelSize.x() - mMinVoxSize->x()) < 0.005*mMinVoxSize->x() /*&& fabs(mVoxelSize.y() - mMinVoxSize->y()) < 0.05*mMinVoxSize->y() && fabs(mVoxelSize.z() - mMinVoxSize->z()) < 0.05*mMinVoxSize->z()*/)
	{
		if(!this->isIntersectingTriangle(v1,v2,v3))
			return false;
		else
		{
			Voxeltable.insert(this);
			return true;  // also mark this node as filled so more triangle insertions can't delete it
		}
	}



	//if(!this->hasChildren())  //should have check that re-initializes all null nodes
	this->subdivide();

	int added = 0;
	for(size_t i = 0;i<8;i++)
	{
		if(this->children[i]->isIntersectingBBox(triangleBBox.lower(),triangleBBox.upper()))
		{	 //Voxeltable.insert(this);
			if(!this->children[i]->insertTriangle(v1,v2,v3))
			{
				if(Voxeltable.find(this->children[i]) == Voxeltable.end() && this->children[i] != nullptr)	   // revise nullptr check
				{
					//Node* toBeDeleted = children[i];
					//children[i] = nullptr;
					//delete toBeDeleted;
				}
			}
			else
				++added;
		}
		else
		{
			if(this->children[i] != nullptr)
				{
					//Node* toBeDeleted = children[i];
					//children[i] = nullptr;
					//delete toBeDeleted;
				}
		}
	}
	return added > 0 ? true : false ;
}


bool Node::isIntersectingBBox(const Vector3D& inMinPoint, const Vector3D& inMaxPoint)
{
	double tol = 0.000001;

	Vector3D voxelMinPoint,voxelMaxPoint;

	getVoxelBBox(voxelMinPoint,voxelMaxPoint);

	return (	voxelMinPoint.x() - tol > inMaxPoint.x()
			||	voxelMinPoint.y() - tol > inMaxPoint.y()
			||	voxelMinPoint.z() - tol > inMaxPoint.z()
			||	voxelMaxPoint.x() + tol < inMinPoint.x()
			||	voxelMaxPoint.y() + tol < inMinPoint.y()
			||	voxelMaxPoint.z() + tol < inMinPoint.z() ?false:true	);
}


bool Node::isIntersectingTriangle(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3)
{

	double tol = 0.0001;

	Vector3D voxelMinPoint,voxelMaxPoint;
	getVoxelBBox(voxelMinPoint,voxelMaxPoint);
	Vector3D tol3D = 0.01* (voxelMaxPoint-voxelMinPoint);

	std::vector<Vector3D> voxCoordinates;
	Voxel voxel = getVoxel();
	voxel.getCoordinates(voxCoordinates);

	Vector3D normal =  (v1-v2)^(v3-v2);
	normal.normalize();

	// this part checks which of the 8 vertices of the voxel is closest to the plane
	int closestVertexIndex = 0;
	double minDistanceFromPlane = fabs( ( voxCoordinates[0]-(v1+v2+v3)/3 ) * normal );
	for (int i = 1; i < 8; i++)
	{
		double distanceFromPlane = fabs( ( voxCoordinates[i]-(v1+v2+v3)/3 ) * normal );
		if(distanceFromPlane < minDistanceFromPlane)
		{
			minDistanceFromPlane = distanceFromPlane;
			closestVertexIndex = i;
		}
	}
	//for	(int closestVertexIndex = 0; closestVertexIndex < 8; closestVertexIndex++){
	std::vector<Vector3D>neighbours =  voxel.getNeighbourVertices(VoxCoordinate(closestVertexIndex));
	for (int i = 0; i < neighbours.size(); i++)
	{
		double t,u,v;
		Vector3D direction = voxCoordinates[closestVertexIndex] - neighbours[i];
		if(isRayTriangleIntersecting(mCentre,direction,v1,v2,v3,t,u,v))
		{
			Vector3D intersection = (1-u-v)*v1 + u*v2 + v*v3;
			 if( 	voxelMinPoint.x() - tol3D.x() < intersection.x() && voxelMaxPoint.x() + tol3D.x()  > intersection.x()
				&&	voxelMinPoint.y() - tol3D.y() < intersection.y() && voxelMaxPoint.y() + tol3D.y()  > intersection.y()
				&&	voxelMinPoint.z() - tol3D.y() < intersection.z() && voxelMaxPoint.z() + tol3D.y()  > intersection.z())
			 {
				 return true;
			 }
		}
	}//}
		   
	  return false;

	  //- tol3D.x()	 + tol3D.x()
	  //- tol3D.y()	 + tol3D.y()
	  //- tol3D.y()	 + tol3D.y()

	/*Vector3D normal =  (v1-v2)^(v3-v2);
	normal.normalize();
	if(fabs((mCentre - v2)*normal) < 0.5*mVoxelSize.x() )
		return true;
	return false;*/

}

bool Node::isRayTriangleIntersecting(const Vector3D &orig,const Vector3D &dir,const Vector3D &v1,const Vector3D &v2,const Vector3D &v3, double &t, double &u, double &v) 
{ 

    Vector3D v1v2 = v2 - v1; 
    Vector3D v1v3 = v3 - v1; 
    Vector3D pvec = dir^v1v3; 
	double	 det  = v1v2*pvec; 

    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < 0.00000000001) return false; 

	double invDet = 1 / det; 
 
    Vector3D tvec = orig - v1; 
    u = (tvec*pvec) * invDet; 
    if (u < 0 || u > 1) return false; 
 
    Vector3D qvec = tvec^v1v2; 
    v = (dir*qvec) * invDet; 
    if (v < 0 || u + v > 1) return false; 
 
    t = v1v3*(qvec) * invDet; 
 
    return true;
}


const Vector3D* Node::getCentrePoint()  const
{
	return &this->mCentre;
}

Voxel Node::getVoxel()
{
	return Voxel(&mCentre, &mVoxelSize);
}

void Node::getVoxelBBox(Vector3D& outMinPoint , Vector3D& outMaxPoint)
{
	outMinPoint = mCentre - 0.5*mVoxelSize;
	outMaxPoint = mCentre + 0.5*mVoxelSize;
}


Voxel::Voxel(Vector3D* voxelCenterPoint, Vector3D* voxelSize)
{
	mCentre = voxelCenterPoint;
	mVoxelSize = voxelSize;
}

void Voxel::getCoordinates(std::vector<Vector3D>& coordinates)
{
	if(coordinates.capacity() == 0)
		coordinates.reserve(8);

	coordinates.push_back(*mCentre + 0.5*Vector3D( mVoxelSize->x(),  mVoxelSize->y() ,  mVoxelSize->z() ));
	coordinates.push_back(*mCentre + 0.5*Vector3D(-mVoxelSize->x(),  mVoxelSize->y() ,  mVoxelSize->z() ));
	coordinates.push_back(*mCentre + 0.5*Vector3D(-mVoxelSize->x(), -mVoxelSize->y() ,  mVoxelSize->z() ));
	coordinates.push_back(*mCentre + 0.5*Vector3D( mVoxelSize->x(), -mVoxelSize->y() ,  mVoxelSize->z() ));
	
	coordinates.push_back(*mCentre + 0.5*Vector3D( mVoxelSize->x(),  mVoxelSize->y() , -mVoxelSize->z() ));
	coordinates.push_back(*mCentre + 0.5*Vector3D(-mVoxelSize->x(),  mVoxelSize->y() , -mVoxelSize->z() ));
	coordinates.push_back(*mCentre + 0.5*Vector3D(-mVoxelSize->x(), -mVoxelSize->y() , -mVoxelSize->z() ));
	coordinates.push_back(*mCentre + 0.5*Vector3D( mVoxelSize->x(), -mVoxelSize->y() , -mVoxelSize->z() ));
}

Vector3D Voxel::getCoordinate(VoxCoordinate option) // gets the coordinate based on enum options
{
	switch (option)
	{
		case TOP_FACE_TOP_RIGHT			: return  *mCentre + 0.5*Vector3D( mVoxelSize->x(),  mVoxelSize->y() ,  mVoxelSize->z() );											 
		case TOP_FACE_TOP_LEFT			: return  *mCentre + 0.5*Vector3D(-mVoxelSize->x(),  mVoxelSize->y() ,  mVoxelSize->z() );
		case TOP_FACE_BOTTOM_LEFT		: return  *mCentre + 0.5*Vector3D(-mVoxelSize->x(), -mVoxelSize->y() ,  mVoxelSize->z() );
		case TOP_FACE_BOTTOM_RIGHT		: return  *mCentre + 0.5*Vector3D( mVoxelSize->x(), -mVoxelSize->y() ,  mVoxelSize->z() );
		case BOTTOM_FACE_TOP_RIGHT		: return  *mCentre + 0.5*Vector3D( mVoxelSize->x(),  mVoxelSize->y() , -mVoxelSize->z() );
		case BOTTOM_FACE_TOP_LEFT		: return  *mCentre + 0.5*Vector3D(-mVoxelSize->x(),  mVoxelSize->y() , -mVoxelSize->z() );
		case BOTTOM_FACE_BOTTOM_LEFT	: return  *mCentre + 0.5*Vector3D(-mVoxelSize->x(), -mVoxelSize->y() , -mVoxelSize->z() );	
		case BOTTOM_FACE_BOTTOM_RIGHT	: return  *mCentre + 0.5*Vector3D( mVoxelSize->x(), -mVoxelSize->y() , -mVoxelSize->z() );
		default	: return Vector3D();
	}
}

std::vector<Vector3D> Voxel::getNeighbourVertices(VoxCoordinate option) // gets the neighbouring vertex based on enum options
{
	std::vector<Vector3D> neighbours;
	neighbours.reserve(3);
	switch (option)
	{
		case TOP_FACE_TOP_RIGHT :
									neighbours.push_back(getCoordinate(TOP_FACE_TOP_LEFT));
									neighbours.push_back(getCoordinate(TOP_FACE_BOTTOM_RIGHT));
									neighbours.push_back(getCoordinate(BOTTOM_FACE_TOP_RIGHT));

		case TOP_FACE_TOP_LEFT :
									neighbours.push_back(getCoordinate(TOP_FACE_TOP_RIGHT));
									neighbours.push_back(getCoordinate(TOP_FACE_BOTTOM_LEFT));
									neighbours.push_back(getCoordinate(BOTTOM_FACE_TOP_LEFT));

		case TOP_FACE_BOTTOM_LEFT :
									neighbours.push_back(getCoordinate(TOP_FACE_TOP_LEFT));
									neighbours.push_back(getCoordinate(TOP_FACE_BOTTOM_RIGHT));
									neighbours.push_back(getCoordinate(BOTTOM_FACE_BOTTOM_LEFT));

		case TOP_FACE_BOTTOM_RIGHT :
									neighbours.push_back(getCoordinate(TOP_FACE_TOP_RIGHT));
									neighbours.push_back(getCoordinate(TOP_FACE_BOTTOM_LEFT));
									neighbours.push_back(getCoordinate(BOTTOM_FACE_BOTTOM_RIGHT));

		case BOTTOM_FACE_TOP_RIGHT :
									neighbours.push_back(getCoordinate(BOTTOM_FACE_TOP_LEFT));
									neighbours.push_back(getCoordinate(BOTTOM_FACE_BOTTOM_RIGHT));
									neighbours.push_back(getCoordinate(TOP_FACE_TOP_RIGHT));

		case BOTTOM_FACE_TOP_LEFT :
									neighbours.push_back(getCoordinate(BOTTOM_FACE_TOP_RIGHT));
									neighbours.push_back(getCoordinate(BOTTOM_FACE_BOTTOM_LEFT));
									neighbours.push_back(getCoordinate(TOP_FACE_TOP_LEFT));

		case BOTTOM_FACE_BOTTOM_LEFT :
									neighbours.push_back(getCoordinate(BOTTOM_FACE_TOP_LEFT));
									neighbours.push_back(getCoordinate(BOTTOM_FACE_BOTTOM_RIGHT));
									neighbours.push_back(getCoordinate(TOP_FACE_BOTTOM_LEFT));

		case BOTTOM_FACE_BOTTOM_RIGHT :
									neighbours.push_back(getCoordinate(BOTTOM_FACE_TOP_RIGHT));
									neighbours.push_back(getCoordinate(BOTTOM_FACE_BOTTOM_LEFT));
									neighbours.push_back(getCoordinate(TOP_FACE_BOTTOM_RIGHT));

	}
	return neighbours;
}

void Voxel::getTriangleFacet(int facetNo, Vector3D& v1, Vector3D& v2, Vector3D& v3)
{
	v1 = getCoordinate(VoxCoordinate(VoxelTessData[3*facetNo]));
	v2 = getCoordinate(VoxCoordinate(VoxelTessData[3*facetNo+1]));
	v3 = getCoordinate(VoxCoordinate(VoxelTessData[3*facetNo+2]));
}
