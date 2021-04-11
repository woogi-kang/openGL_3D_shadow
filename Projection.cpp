/****************************************************************************************************************************
name : Kang Tae Wook
digipenID : taewook.kang
assignment number : 4
course name : CS250
term : Spring 2019
*****************************************************************************************************************************/

#include "Projection.h"

Affine CameraToWorld(const Camera & cam)
{
	Affine camTow;

	camTow.row[0] = Hcoord(cam.Right().x, cam.Up().x, cam.Back().x, cam.Eye().x);
	camTow.row[1] = Hcoord(cam.Right().y, cam.Up().y, cam.Back().y, cam.Eye().y);
	camTow.row[2] = Hcoord(cam.Right().z, cam.Up().z, cam.Back().z, cam.Eye().z);
	camTow.row[3] = Hcoord(0, 0, 0, 1);

	return camTow;
}

Affine WorldToCamera(const Camera & cam)
{
	Affine wTocam = inverse(CameraToWorld(cam));

	return wTocam;
}

Matrix CameraToNDC(const Camera & cam)
{
	Matrix camTondc;

	camTondc.row[0] = Hcoord((2 * cam.ViewportGeometry().z / cam.ViewportGeometry().x) , 0,0,0);
	camTondc.row[1] = Hcoord(0, (2 * cam.ViewportGeometry().z / cam.ViewportGeometry().y), 0, 0);
	camTondc.row[2] = Hcoord(0, 0, ((cam.NearDistance()+cam.FarDistance())/ (cam.NearDistance() - cam.FarDistance()))
		, ((2*cam.NearDistance()*cam.FarDistance()) / (cam.NearDistance() - cam.FarDistance())));
	camTondc.row[3] = Hcoord(0, 0, -1, 0);

	return  camTondc;
}
