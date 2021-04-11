/****************************************************************************************************************************
name : Kang Tae Wook
digipenID : taewook.kang
assignment number : 3
course name : CS250
term : Spring 2019
*****************************************************************************************************************************/

#include "FrustumMesh.h"

const Mesh::Face FrustumMesh::faces[12] = 
{
	Face(1,5,7),
	Face(1,7,3),
	Face(2,4,8),
	Face(2,8,6),
	Face(5,6,8),
	Face(5,8,7),
	Face(1,3,4),
	Face(1,4,2),
	Face(1,2,5),
	Face(2,6,5),
	Face(3,7,4),
	Face(4,7,8)
};

const Mesh::Edge FrustumMesh::edges[16] =
{
	Edge(1,5),
	Edge(5,7),
	Edge(3,7),
	Edge(1,3),
	Edge(1,2),
	Edge(2,4),
	Edge(3,4),
	Edge(2,6),
	Edge(6,8),
	Edge(4,8),
	Edge(5,6),
	Edge(7,8),
	Edge(0,5),
	Edge(0,6),
	Edge(0,7),
	Edge(0,8)
};

FrustumMesh::FrustumMesh(float fov, float a, float n, float f)
{
	center = Point(0, 0, 0);

	float angle = std::tan(fov / 2.f);
	float distance = f;
	float width = angle * 2;
	float height = width / a;

	vertices[0] = center;
	vertices[1] = Point(f* width/2, f* height/2, -f);
	vertices[2] = Point(f*-width/2, f* height/2, -f);
	vertices[3] = Point(f* width/2, f*-height/2, -f);
	vertices[4] = Point(f*-width/2, f*-height/2, -f);
	vertices[5] = Point(n* width/2, n* height/2, -n);
	vertices[6] = Point(n*-width/2, n* height/2, -n);
	vertices[7] = Point(n* width/2, n*-height/2, -n);
	vertices[8] = Point(n*-width/2, n*-height/2, -n);

	width = angle * 2 * distance;
	height = width / a;
	dimensions = Vector(width*2, height*2, f-n);
}

int FrustumMesh::VertexCount(void)
{
	return 9;
}

Point FrustumMesh::GetVertex(int i)
{
	return vertices[i];
}

Vector FrustumMesh::Dimensions(void)
{
	return dimensions;
}

Point FrustumMesh::Center(void)
{
	return Point(0,0,0);
}

int FrustumMesh::FaceCount(void)
{
	return 12;
}

Mesh::Face FrustumMesh::GetFace(int i)
{
	return faces[i];
}

int FrustumMesh::EdgeCount(void)
{
	return 16;
}

Mesh::Edge FrustumMesh::GetEdge(int i)
{
	return edges[i];
}