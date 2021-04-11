/*------------------------------------------------------------------------
Copyright (C) 2019 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: CS250 Project2
Project: 3D Meshes
Author: Kang Tae Wook
Created: 04/06/19
------------------------------------------------------------------------*/

#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <ctime>
#include <vector>
#include <iostream>
#include <sstream>
#include "Affine.h"
#include "Mesh.h"
#include "Render.h"
#include "Camera.h"
#include "SnubDodecMesh.h"
#include "CubeMesh.h"
#include "Projection.h"

using namespace std;

const Point Origin(0, 0, 0);

const Vector EX(1, 0, 0),
             EY(0, 1, 0),
             EZ(0, 0, 1);

const float PI = 4.0f * atan(1.0f);

static bool mouse_click = false;

Vector  WHITE(1, 1, 1),
		PURPLE(1, 0, 1),
		BLACK(0, 0, 0),
		RED(1, 0, 0),
		GREEN(0, 1, 0),
		BLUE(102, 210, 255),
		PICK1(3, 0, 0),
		PICK2(3, 0, 0),
		PICK3(3, 0, 0),
		PICK4(3, 0, 0);


class CameraRender
{
public:
	CameraRender(Render &r);
	~CameraRender(void);
	void SetCamera(const Camera &cam);
	void DisplayEdges(Mesh& m, const Affine& A, const Vector& color);
	void DisplayFaces(Mesh& m, const Affine& A, const Vector& color);
	void DisplayShadow(Mesh& m, const Affine& A);

	void SetLightingPoint(Point* input);
private:
	Render &render;
	Affine world2camera;
	
	Point* light_pos;
	Matrix camera2ndc;
};

CameraRender::CameraRender(Render& r)
	: render(r),light_pos(0)
{
}

CameraRender::~CameraRender()
{
}

void CameraRender::SetCamera(const Camera& cam)
{
	world2camera = WorldToCamera(cam);
	camera2ndc = CameraToNDC(cam);
}

void CameraRender::DisplayEdges(Mesh& m, const Affine& A, const Vector& color)
{
	const Affine wToc = world2camera * A;
	render.SetColor(color);

	for (int i = 0; i < m.EdgeCount(); ++i)
	{
		Hcoord first = m.GetVertex(m.GetEdge(i).index1);
		Hcoord second = m.GetVertex(m.GetEdge(i).index2);

		first = wToc * first;
		second = wToc * second;

		if (first.z >= 0 || second.z >= 0)
			continue;

		first = camera2ndc * first;
		second = camera2ndc * second;

		first = (1.0f / first.w)*first;
		second = (1.0f / second.w)*second;

		render.DrawLine(first, second);
	}
}

void CameraRender::DisplayFaces(Mesh& m, const Affine& A, const Vector& color)
{
	const Affine cam2w = inverse(world2camera);
	const Point  eye{ cam2w.row[0].w, cam2w.row[1].w, cam2w.row[2].w };
	Vector light = *light_pos - (A * m.GetVertex(m.GetFace(0).index1));

	for (int i = 0; i < m.FaceCount(); ++i)
	{
		Hcoord P = m.GetVertex(m.GetFace(i).index1);
		Hcoord Q = m.GetVertex(m.GetFace(i).index2);
		Hcoord R = m.GetVertex(m.GetFace(i).index3);

		P = A * P;
		Q = A * Q;
		R = A * R;

		Vector mVector = cross(Q - P, R - P);

		if (dot((eye - P), mVector) > 0)
		{
			P = world2camera * P;
			Q = world2camera * Q;
			R = world2camera * R;

			if (P.z >= 0 || Q.z >= 0 || R.z >= 0)
				continue;

			float dot_with_light = dot(mVector, light);
			float mu = 0.f;

			if(dot_with_light > 0.f)
				mu = std::abs(dot(mVector, light)) / (abs(mVector) * abs(light));

			P = camera2ndc * P;
			P = (1.0f / P.w) * P;
			Q = camera2ndc * Q;
			Q = (1.0f / Q.w) * Q;
			R = camera2ndc * R;
			R = (1.0f / R.w) * R;

			render.SetColor(mu*color);
			render.FillTriangle(P, Q, R);
		}
	}
}

void CameraRender::DisplayShadow(Mesh& m, const Affine& A)
{
	Vector light = *light_pos - (A * m.GetVertex(m.GetFace(0).index1));

	Matrix generate_shadow = { Hcoord(light.y, -light.x, 0,0),
	Hcoord(0,0,0,0),
	Hcoord(0, -light.z, light.y, 0),
	Hcoord(0,-1,0,light.y) };

	for (int i = 0; i < m.FaceCount(); ++i)
	{
		Hcoord P = m.GetVertex(m.GetFace(i).index1);
		Hcoord Q = m.GetVertex(m.GetFace(i).index2);
		Hcoord R = m.GetVertex(m.GetFace(i).index3);

		P = A * P;
		Q = A * Q;
		R = A * R;

		P = generate_shadow * P;
		Q = generate_shadow * Q;
		R = generate_shadow * R;

		P = world2camera * P;
		Q = world2camera * Q;
		R = world2camera * R;

		P = camera2ndc * P;
		Q = camera2ndc * Q;
		R = camera2ndc * R;

		render.SetColor(BLACK);
		render.FillTriangle(P, Q, R);
	}
}

void CameraRender::SetLightingPoint(Point* input)
{
	light_pos = input;
}

class Client
{
public:
	Client(SDL_Window* window);
	~Client(void);
	void draw(double dt);
	void keypress(SDL_Keycode kc);
	void SetMousePos(int x, int y);
	void resize(int W, int H);
	void Picking(SnubDodecMesh& mesh, int num_);

private:
	// variables for frame rate:
	SDL_Window* window;
	int frame_count;
	double frame_time;

	Point mouse_pos;

private:
	Camera cam1;
	SnubDodecMesh snub_mesh;
	CubeMesh cube_mesh;
	Affine light2world, picking2world[4], floor2world;
	Point cam_center, light_center, floor_center;
	Point pick_center1, pick_center2, pick_center3, pick_center4;
	Render* render;
	CameraRender* crender;
	float aspect = 1.f;

	float light_rotation = 0.001f;
};

Client::Client(SDL_Window* win) : window(win)
{
	frame_count = 0;
	frame_time = 0;

	render = new Render();
	crender = new CameraRender(*render);

	cam_center = Point(0, 2, 5);
	cam1 = Camera(cam_center, -EZ, EY, 0.5f * PI, 1, 1.f, 100.f);
	
	light_center = Point(0, 4.f, -1.f);
	floor_center = Point(0, -0.8f, -3.f);
	pick_center1 = Point(-1.5f, 0.8f, -0.f);
	pick_center2 = Point(-0.3f, 0.8f, -0.f);
	pick_center3 = Point(0.9f, 0.8f, -0.f);
	pick_center4 = Point(2.1f, 0.8f, -0.f);

	floor2world = translate(floor_center - Origin);
	light2world = translate(light_center - Origin) * scale(0.2f, 0.2f, 0.2f);
	picking2world[0] = translate(pick_center1 - Origin) * scale(0.2f, 0.2f, 0.2f);
	picking2world[1] = translate(pick_center2 - Origin) * scale(0.2f, 0.2f, 0.2f);
	picking2world[2] = translate(pick_center3 - Origin) * scale(0.2f, 0.2f, 0.2f);
	picking2world[3] = translate(pick_center4 - Origin) * scale(0.2f, 0.2f, 0.2f);

	crender->SetCamera(cam1);
	crender->SetLightingPoint(&light_center);
}


Client::~Client(void)
{
	delete crender;
	delete render;
}


void Client::draw(double dt)
{
	++frame_count;
	frame_time += dt;
	if (frame_time >= 0.5)
	{
		double fps = frame_count / frame_time;
		frame_count = 0;
		frame_time = 0;
		stringstream ss;   
		ss << "CS 250: Project #2 [fps=" << int(fps) << "]";
		SDL_SetWindowTitle(window, ss.str().c_str());
	}

	// Cam drawing code:
	render->ClearBuffers(BLUE);

	///////////////////////////
	// Draw calls

	light_rotation += 0.005f;
	light_center.x = sin(light_rotation);
	light_center.z = cos(light_rotation);

	light2world = translate(light_center - Origin) * scale(0.2f, 0.2f, 0.2f);;

	crender->DisplayFaces(snub_mesh, CameraToWorld(cam1) * scale(0.035f, 0.035f, 0.035f), RED);
	crender->DisplayFaces(cube_mesh, light2world , WHITE);

	crender->DisplayFaces(cube_mesh, floor2world * scale(6.5f, 0.1f, 6.5f), WHITE);

	crender->DisplayFaces(snub_mesh, picking2world[0], PICK1);
	crender->DisplayShadow(snub_mesh, picking2world[0]);
	crender->DisplayFaces(snub_mesh, picking2world[1], PICK2);
	crender->DisplayShadow(snub_mesh, picking2world[1]);
	crender->DisplayFaces(snub_mesh, picking2world[2], PICK3);
	crender->DisplayShadow(snub_mesh, picking2world[2]);
	crender->DisplayFaces(snub_mesh, picking2world[3], PICK4);
	crender->DisplayShadow(snub_mesh, picking2world[3]);


	if(mouse_click)
	{
		Picking(snub_mesh, 1);
		Picking(snub_mesh, 2);
		Picking(snub_mesh, 3);
		Picking(snub_mesh, 4);
	}
}

void Client::keypress(SDL_Keycode kc)
{
	switch (kc)
	{
	case SDLK_SPACE:
	{
	
	}
	default:
		break;
	}
}

void Client::SetMousePos(int x, int y)
{
	int width, height;

	SDL_GetWindowSize(window, &width, &height);

	Point mousePos((float)x,(float)y, 1.f);

	const Affine vp_to_ndc = { Hcoord(2 / (float)width, 0,0),
							   Hcoord(0, -2 / (float)height, 0),
							   Hcoord((1 - (float)width) / (float)width, ((float)height - 1) / (float)height,1),
								Point() };

	mousePos = vp_to_ndc * mousePos;

	float right = cam1.GetWidth()/2;
	float left = -right;
	float top = cam1.GetHeight() / 2;
	float bot = -top;


	const Affine ndc_to_vf{ Hcoord((right - left) / 2 , 0, 0),
							Hcoord(0, (top - bot) / 2, 0),
							Hcoord((right + left) / 2, (top + bot) / 2, 1),
							Point() };

	mousePos = ndc_to_vf * mousePos;
	mousePos.z = -100.f;

	const Matrix v_to_w{ Hcoord(cam1.Right().x, cam1.Up().x, cam1.Back().x, cam1.Eye().x),
						 Hcoord(cam1.Right().y, cam1.Up().y, cam1.Back().y, cam1.Eye().y),
						 Hcoord(cam1.Right().z, cam1.Up().z, cam1.Back().z, cam1.Eye().z),
						 Point() };

	mousePos = v_to_w * mousePos;

	mouse_pos = Point(mousePos);
}

void Client::resize(int W, int H)
{
	glViewport(0, 0, W, H);
	aspect = float(W) / float(H);
}

void Client::Picking(SnubDodecMesh& mesh, int num_)
{
	Vector ray(mouse_pos - cam_center);

	switch (num_)
	{
	case 1:
	{
		Hcoord v = picking2world[0] * mesh.GetVertex(0) - pick_center1 ;

		float radius = dot(v, v);
		float p = dot(pick_center1 - cam_center, ray);
		
		if(p > 0)
		{
			float m = p / abs(ray);

			float dot_center_and_cam = dot(pick_center1 - cam_center, pick_center1 - cam_center);
			float n = dot_center_and_cam - m * m;

			if(n < radius)
			{
				float s = sqrt(radius - n);

				float t = m - s / abs(ray);

				if(t > 0)
				{
					mouse_click = !mouse_click;
					int r = std::rand() % 3;
					int g = std::rand() & 3;
					int b = std::rand() & 3;
					PICK1 = Vector((float)r, (float)g, (float)b);
				}
			}
		}
		break;
	}
	case 2:
	{
		Hcoord v = picking2world[1] * mesh.GetVertex(0) - pick_center2;

		float radius = dot(v, v);
		float p = dot(pick_center2 - cam_center, ray);

		if (p > 0)
		{
			float m = p / abs(ray);

			float dot_center_and_cam = dot(pick_center2 - cam_center, pick_center2 - cam_center);
			float n = dot_center_and_cam - m * m;

			if (n < radius)
			{
				float s = sqrt(radius - n);

				float t = m - s / abs(ray);

				if (t > 0)
				{
					mouse_click = !mouse_click;
					int r = std::rand() % 3;
					int g = std::rand() & 3;
					int b = std::rand() & 3;
					PICK2 = Vector((float)r, (float)g, (float)b);
				}
			}
		}
		break;
	}
	case 3:
	{
		Hcoord v = picking2world[2] * mesh.GetVertex(0) - pick_center3;

		float radius = dot(v, v);
		float p = dot(pick_center3 - cam_center, ray);

		if (p > 0)
		{
			float m = p / abs(ray);

			float dot_center_and_cam = dot(pick_center3 - cam_center, pick_center3 - cam_center);
			float n = dot_center_and_cam - m * m;

			if (n < radius)
			{
				float s = sqrt(radius - n);

				float t = m - s / abs(ray);

				if (t > 0)
				{
					mouse_click = !mouse_click;
					int r = std::rand() % 3;
					int g = std::rand() & 3;
					int b = std::rand() & 3;
					PICK3 = Vector((float)r, (float)g, (float)b);
				}
			}
		}
		break;
	}
	case 4:
	{
		Hcoord v = picking2world[3] * mesh.GetVertex(0) - pick_center4;

		float radius = dot(v, v);
		float p = dot(pick_center4 - cam_center, ray);

		if (p > 0)
		{
			float m = p / abs(ray);

			float dot_center_and_cam = dot(pick_center4 - cam_center, pick_center4 - cam_center);
			float n = dot_center_and_cam - m * m;

			if (n < radius)
			{
				float s = sqrt(radius - n);

				float t = m - s / abs(ray);

				if (t > 0)
				{
					mouse_click = !mouse_click;
					int r = std::rand() % 3;
					int g = std::rand() & 3;
					int b = std::rand() & 3;
					PICK4 = Vector((float)r, (float)g, (float)b);
				}
			}
		}
		break;
	}
	default:
		break;
	}
}

/////////////////////////////////////////////////////////////////
int main(int /*argc*/, char* /*argv*/[])
{
	srand(unsigned(time(nullptr)));

	// SDL: initialize and create a window
	SDL_Init(SDL_INIT_VIDEO);
	const char* title = "CS250: Project #2";
	int width = 1280, height = 1024;
	SDL_Window* window =
		SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		                 width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
	SDL_GLContext context = SDL_GL_CreateContext(window);

	// GLEW: get function bindings (if possible)
	glewInit();
	if (!GLEW_VERSION_2_0)
	{
		cout << "needs OpenGL version 3.0 or better" << endl;
		return -1;
	}

	// animation loop
	try
	{
		bool done = false;
		Client* client = new Client(window);
		Uint32 ticks_last = SDL_GetTicks();
		while (!done)
		{
			SDL_Event event;
			while (SDL_PollEvent(&event))
			{
				switch (event.type)
				{
					case SDL_QUIT:
					{
						done = true;
						break;
					}
					case SDL_KEYDOWN:
					{
						if (event.key.keysym.sym == SDLK_ESCAPE)
							done = true;
						else
							client->keypress(event.key.keysym.sym);
						break;
					}
					case SDL_MOUSEBUTTONDOWN:
					{
						client->SetMousePos(event.motion.x, event.motion.y);
						mouse_click = !mouse_click;
						break;
					}
					case SDL_WINDOWEVENT:
					{
						if (event.window.event == SDL_WINDOWEVENT_RESIZED)
							client->resize(event.window.data1, event.window.data2);
						break;
					}
				}
			}
			Uint32 ticks = SDL_GetTicks();
			double dt = 0.001 * (ticks - ticks_last);
			ticks_last = ticks;
			client->draw(dt);
			SDL_GL_SwapWindow(window);
		}
		delete client;
	}

	catch (exception& e)
	{
		cout << e.what() << endl;
	}

	SDL_GL_DeleteContext(context);
	SDL_Quit();
	return 0;
}
