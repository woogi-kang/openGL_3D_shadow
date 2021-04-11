#include "Affine.h"
#include <iostream>

namespace
{
	Affine crossProductMatrix(const Vector& v);
	Affine operator+(const Affine& A, const Affine& B);
	Affine transpose(const Affine& A);
	Vector determinant(const Vector& u, const Vector& v);
}

Hcoord::Hcoord(float X, float Y, float Z, float W) : x(X),y(Y),z(Z),w(W)
{
}

Point::Point(float X, float Y, float Z) : Hcoord(X,Y,Z,1)
{
}

Vector::Vector(float X, float Y, float Z) : Hcoord(X,Y,Z,0)
{
}

Affine::Affine(void)
{
	row[0] = Vector();
	row[1] = Vector();
	row[2] = Vector();
	row[3] = Point();
}

Affine::Affine(const Vector & Lx, const Vector & Ly, const Vector & Lz, const Point & D)
{
	row[0] = Hcoord(Lx.x, Ly.x,Lz.x, D.x);
	row[1] = Hcoord(Lx.y, Ly.y, Lz.y, D.y);
	row[2] = Hcoord(Lx.z, Ly.z, Lz.z, D.z);
	row[3] = Hcoord(Lx.w, Ly.w, Lz.w, D.w);
}

Hcoord operator+(const Hcoord & u, const Hcoord & v)
{
	Hcoord temp;
	temp.x = u.x + v.x;
	temp.y = u.y + v.y;
	temp.z = u.z + v.z;
	temp.w = u.w + v.w;

	return temp;
}

Hcoord operator-(const Hcoord & u, const Hcoord & v)
{
	Hcoord temp;
	temp.x = u.x - v.x;
	temp.y = u.y - v.y;
	temp.z = u.z - v.z;
	temp.w = u.w - v.w;

	return temp;
}

Hcoord operator-(const Hcoord & v)
{
	Hcoord temp;
	temp.x = -v.x;
	temp.y = -v.y;
	temp.z = -v.z;
	temp.w = -v.w;

	return temp;
}

Hcoord operator*(float r, const Hcoord & v)
{
	Hcoord temp;
	temp.x = v.x * r;
	temp.y = v.y * r;
	temp.z = v.z * r;
	temp.w = v.w * r;

	return temp;
}

Hcoord operator*(const Matrix & A, const Hcoord & v)
{
	Hcoord temp;

	temp.x = ((A.row[0].x * v.x) + (A.row[0].y * v.y) + (A.row[0].z * v.z) + (A.row[0].w * v.w));
	temp.y = ((A.row[1].x * v.x) + (A.row[1].y * v.y) + (A.row[1].z * v.z) + (A.row[1].w * v.w));
	temp.z = ((A.row[2].x * v.x) + (A.row[2].y * v.y) + (A.row[2].z * v.z) + (A.row[2].w * v.w));
	temp.w = ((A.row[3].x * v.x) + (A.row[3].y * v.y) + (A.row[3].z * v.z) + (A.row[3].w * v.w));

	return temp;
}

Matrix operator*(const Matrix & A, const Matrix & B)
{
	Matrix temp;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			for (int k = 0; k < 4; k++)
			{
				temp[i][j] += (A[i][k] * B[k][j]);
			}
		}
	}

	return temp;
}

float dot(const Vector & u, const Vector & v)
{
	return ((v.x * u.x) + (v.y * u.y) + (v.z * u.z));
}

float abs(const Vector & v)
{
	return std::sqrt(dot(v,v));
}

Vector normalize(const Vector & v)
{
	Vector temp;
	float magnitude = std::sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));

	temp.x = v.x / magnitude;
	temp.y = v.y / magnitude;
	temp.z = v.z / magnitude;

	return temp;
}

Vector cross(const Vector & u, const Vector & v)
{
	Vector temp;

	temp.x = (u.y * v.z) - (u.z * v.y);
	temp.y = (u.z * v.x) - (u.x * v.z);
	temp.z = (u.x * v.y) - (u.y * v.x);
	temp.w = 0;

	return temp;
}

Affine rotate(float t, const Vector & v)
{
	Affine first = scale(std::cos(t));

	float outer_product = std::abs((1 - std::cos(t)) / dot(v,v));
	Affine outer_product_matrix;

	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
		{
			outer_product_matrix[i][j] = outer_product*(v[i] * v[j]);
			
		}
	}

	Affine cross_product_matrix = crossProductMatrix(v);
	float cross_product = std::sin(t) / abs(v);

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			cross_product_matrix[i][j] *= cross_product;
		}
	}

	return first + outer_product_matrix + cross_product_matrix;
}

Affine translate(const Vector & v)
{
	return  Affine(Vector(1,0,0), Vector(0,1,0), Vector(0,0,1), Point(v.x, v.y, v.z));
}

Affine scale(float r)
{
	return Affine(Vector(r,0,0), Vector(0, r, 0), Vector(0,0,r), Point(0,0,0));
}

Affine scale(float rx, float ry, float rz)
{
	return Affine(Vector(rx, 0, 0), Vector(0, ry, 0), Vector(0, 0, rz), Point());
}

Affine inverse(const Affine & A)
{
	Affine cofactor;

	Affine translation_affine = translate(-Vector(A.row[0][3], A.row[1][3], A.row[2][3]));

	cofactor.row[0] = determinant(Vector(A.row[1].x, A.row[1].y, A.row[1].z), Vector(A.row[2].x, A.row[2].y, A.row[2].z));
	cofactor.row[1] = determinant(Vector(A.row[0].x, A.row[0].y, A.row[0].z), Vector(A.row[2].x, A.row[2].y, A.row[2].z));
	cofactor.row[2] = determinant(Vector(A.row[0].x, A.row[0].y, A.row[0].z), Vector(A.row[1].x, A.row[1].y, A.row[1].z));

	cofactor.row[0].y = -cofactor.row[0].y;
	cofactor.row[1].x = -cofactor.row[1].x;
	cofactor.row[1].z = -cofactor.row[1].z;
	cofactor.row[2].y = -cofactor.row[2].y;

	Affine transposed_cofactor = transpose(cofactor);

	float determinant = dot(Vector(A.row[0].x, A.row[1].x, A.row[2].x),
		Vector(cofactor.row[0].x, cofactor.row[1].x, cofactor.row[2].x));

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			transposed_cofactor[i][j] *= 1.f/determinant;
		}
	}

	Affine inverse = transposed_cofactor * translation_affine;

	return inverse;
}

namespace
{
	Affine crossProductMatrix(const Vector& v)
	{
		return Affine(Vector(0, v.z, -v.y), Vector(-v.z, 0, v.x),
			Vector(v.y, -v.x, 0), Point(0,0,0));
	}

	Affine operator+(const Affine& A, const Affine& B)
	{
		Affine temp;

		temp[0] = A[0] + B[0];
		temp[1] = A[1] + B[1];
		temp[2] = A[2] + B[2];
		
		temp[3][0] = A[3][0] + B[3][0];
		temp[3][1] = A[3][1] + B[3][1];
		temp[3][2] = A[3][2] + B[3][2];

		return temp;
	}

	Affine transpose(const Affine& A)
	{	
		Affine temp;

		temp.row[0][0] = A.row[0][0];
		temp.row[0][1] = A.row[1][0];
		temp.row[0][2] = A.row[2][0];
		temp.row[0][3] = A.row[3][0];
		temp.row[1][0] = A.row[0][1];
		temp.row[1][1] = A.row[1][1];
		temp.row[1][2] = A.row[2][1];
		temp.row[1][3] = A.row[3][1];
		temp.row[2][0] = A.row[0][2];
		temp.row[2][1] = A.row[1][2];
		temp.row[2][2] = A.row[2][2];
		temp.row[2][3] = A.row[3][2];
		temp.row[3][0] = A.row[0][3];
		temp.row[3][1] = A.row[1][3];
		temp.row[3][2] = A.row[2][3];
		temp.row[3][3] = A.row[3][3];

		return temp;
	}

	Vector determinant(const Vector& u, const Vector& v)
	{
		float x = (u.y * v.z) - (u.z*v.y);
		float y = (u.x * v.z) - (u.z*v.x);
		float z = (u.x * v.y) - (u.y*v.x);

		return Vector(x, y, z);
	}
}
