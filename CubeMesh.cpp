/****************************************************************************************************************************
name : Kang Tae Wook
digipenID : taewook.kang
assignment number : 2
course name : CS250
term : Spring 2019
*****************************************************************************************************************************/

#include "CubeMesh.h"

const Point CubeMesh::vertices[8] =
{
	Point(1,1,1),
	Point(1,1,-1),
	Point(1,-1,1),
	Point(1,-1,-1),
	Point(-1,1,1),
	Point(-1,1,-1),
	Point(-1,-1,1),
	Point(-1,-1,-1),
};

const CubeMesh::Face CubeMesh::faces[12] =
{
	Face(0,4,6),
	Face(0,6,2),
	Face(1,3,7),
	Face(1,7,5),
	Face(4,5,7),
	Face(4,7,6),
	Face(0,2,3),
	Face(0,3,1),
	Face(0,1,4),
	Face(1,5,4),
	Face(2,6,3),
	Face(3,6,7)
};

const CubeMesh::Edge CubeMesh::edges[12] =
{
	Edge(0,4),
	Edge(4,6),
	Edge(2,6),
	Edge(0,2),
	Edge(0,1),
	Edge(1,3),
	Edge(2,3),
	Edge(1,5),
	Edge(5,7),
	Edge(3,7),
	Edge(4,5),
	Edge(6,7)
};

int CubeMesh::VertexCount(void)
{
	return 8;
}

Point CubeMesh::GetVertex(int i)
{
	return vertices[i];
}

Vector CubeMesh::Dimensions(void)
{
	return Vector(2,2,2);
}

Point CubeMesh::Center(void)
{
	return Point(0,0,0);
}

int CubeMesh::FaceCount(void)
{
	return 12;
}

Mesh::Face CubeMesh::GetFace(int i)
{
	return faces[i];
}

int CubeMesh::EdgeCount(void)
{
	return 12;
}

Mesh::Edge CubeMesh::GetEdge(int i)
{
	return edges[i];
}
