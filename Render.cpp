/****************************************************************************************************************************
name : Kang Tae Wook
digipenID : taewook.kang
assignment number : 2
course name : CS250
term : Spring 2019
*****************************************************************************************************************************/

#include "Render.h"
#include <stdexcept>

Render::Render(void)
{
	const char *simple_fragment_shader =
		"#version 330\n\
		uniform vec3 color;\
		out vec4 frag_color;\
		void main(void) {\
        frag_color = vec4(color,1);\
     }";

	GLint result;
	GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &simple_fragment_shader, 0);
	glCompileShader(fragment_shader);

	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &result);

	if (result == GL_FALSE)
	{
		glDeleteShader(fragment_shader);
		throw std::runtime_error("Compile Failed");
	}

	const char *simple_vertex_shader =
		"#version 330\n\
		attribute vec4 position;\
		void main(void) {\
        gl_Position = position;\
     }";

	GLuint vertext_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertext_shader, 1, &simple_vertex_shader, 0);
	glCompileShader(vertext_shader);

	glGetShaderiv(vertext_shader, GL_COMPILE_STATUS, &result);

	if (result == GL_FALSE)
	{
		glDeleteShader(fragment_shader);
		glDeleteShader(vertext_shader);
		throw std::runtime_error("Compile Failed");
	}

	program = glCreateProgram();
	glAttachShader(program, vertext_shader);
	glAttachShader(program, fragment_shader);
	glLinkProgram(program);

	glGetProgramiv(program, GL_LINK_STATUS, &result);

	if (result == GL_FALSE)
	{
		glDeleteShader(fragment_shader);
		glDeleteShader(vertext_shader);
		throw std::runtime_error("Link Failed");
	}

	ucolor = glGetUniformLocation(program, "color");
	aposition = glGetAttribLocation(program, "position");

	glEnable(GL_DEPTH_TEST);

	glDeleteShader(fragment_shader);
	glDeleteShader(vertext_shader);
}

Render::~Render(void)
{
	glUseProgram(0);
	glDeleteProgram(program);
}

void Render::ClearBuffers(const Vector & c)
{
	glClearColor(c.x, c.y, c.z, c.w);
	glClearDepth(1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Render::SetColor(const Vector & c)
{
	glUseProgram(program);
	glUniform3f(ucolor, c.x, c.y, c.z);
}

void Render::DrawLine(const Hcoord & P, const Hcoord & Q)
{
	GLuint buffer;

	Hcoord temp[2] = { P,Q };

	glGenBuffers(1, &buffer);
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(Hcoord), &temp, GL_STATIC_DRAW);

	glUseProgram(program);
	glEnableVertexAttribArray(aposition);

	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	glVertexAttribPointer(aposition, 4, GL_FLOAT, false, 0, 0);

	glDrawArrays(GL_LINES, 0, 2);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, &buffer);
}


void Render::FillTriangle(const Hcoord & P, const Hcoord & Q, const Hcoord & R)
{
	GLuint buffer;

	Hcoord temp[3] = { P,Q,R };

	glGenBuffers(1, &buffer);
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(Hcoord), &temp, GL_STATIC_DRAW);

	glUseProgram(program);
	glEnableVertexAttribArray(aposition);

	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	glVertexAttribPointer(aposition, 4, GL_FLOAT, false, 0, 0);

	glDrawArrays(GL_TRIANGLES, 0, 3);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, &buffer);
}
