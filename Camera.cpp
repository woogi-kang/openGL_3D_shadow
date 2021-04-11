/****************************************************************************************************************************
name : Kang Tae Wook
digipenID : taewook.kang
assignment number : 3
course name : CS250
term : Spring 2019
*****************************************************************************************************************************/

#include "Camera.h"
#include <cmath>

Camera::Camera(void)
{
	eye = Point(0, 0, 0);
	back = Vector(0, 0, 1);
	right = cross(Vector(0, 0, -1),Vector(0, 0, 1));
	up = cross(right, back);
	
	width = 20.f;
	height = 20.f;
	distance = 10.f;
	near = 0.1f;
	far = 10.f;
}

Camera::Camera(const Point & E, const Vector & look, const Vector & vp, float fov, float aspect, float near, float far)
	: eye(E), distance(far),near(near), far(far)
{	
	const float d_look = std::sqrt(dot(look, look));
	back = (1 / d_look) * -look;

	const float d_look_back = std::sqrt(dot(look, look)) * sqrt(dot(vp, vp));
	right = (1 / d_look_back * cross(look, vp));
	up = cross(back, right);

	const float angle = std::tan(fov / 2.f);
	width = angle * 2 * distance;
	height = width / aspect;
}

Point Camera::Eye(void) const
{
	return eye;
}

Vector Camera::Right(void) const
{
	return right;
}

Vector Camera::Up(void) const
{
	return up;
}

Vector Camera::Back(void) const
{
	return back;
}

Vector Camera::ViewportGeometry(void) const
{
	return Vector(width, height, distance);
}

float Camera::NearDistance(void) const
{
	return near;
}

float Camera::FarDistance(void) const
{
	return far;
}

Camera & Camera::Zoom(float factor)
{
	width *= factor;
	height *= factor;
	return *this;
}

Camera & Camera::Forward(float distance_increment)
{
	eye = eye - distance_increment * back;
	return *this;
}

Camera & Camera::Yaw(float angle)
{
	Affine rotation = rotate(angle, up);

	right = rotation * right;
	back = rotation * back;

	return *this;
}

Camera & Camera::Pitch(float angle)
{
	Affine rotation = rotate(angle, right);

	up = rotation * up;
	back = rotation * back;
	return *this;
}

Camera & Camera::Roll(float angle)
{
	Affine rotation = rotate(angle, back);

	up = rotation * up;
	right = rotation * right;
	return *this;
}
