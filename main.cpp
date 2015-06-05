#include <GLFW/glfw3.h>
#include <iostream>
#include "PointMass.cpp"
#include "Rope.cpp"
#include "RubberRope.cpp"
#include "ChainRope.cpp"
using namespace std;

int main(int argc, char** argv)
{
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    /* Make the window's context current */
    glfwMakeContextCurrent(window);

	PointMass<double> p(1.0, Vec3d(0, 0, 0), Vec3d(0, -10, 0));
	PointMass<double> q(1.0, Vec3d(0, -1, 0), Vec3d(0, 0, 0));
	PointMass<double> r(1.0, Vec3d(1, 0, 0), Vec3d(0, 0, 0));

	ChainRope<double> rope2(10, p.pos+Vec3d(-.75, 0, 0), r.pos + Vec3d(-.75, 0, 0), 100);
	RubberRope<double> rope(10, p.pos, r.pos, 100, 100, 1);
	rope2.links[0].mass = 100;
	rope2.links[99].mass = 100;
	//rope.addPermanentForceAt(Vec3d(-1, -1, 0), 0);
	rope2.addImpulseAt(Vec3d(0, -55, 0), 49);
	//rope2.addImpulseAt(Vec3d(0, -0.2, 0), 4);
	//rope2.addImpulseAt(Vec3d(0, -0.2, 0), 1);
	//rope2.addPermanentForce(Vec3d(0, -0.1, 0));
	rope2.collide(p, q, r);
    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
		glClear(GL_COLOR_BUFFER_BIT);
		glBegin(GL_LINES);
		for (size_t i = 0; i < rope2.links.size()-1; ++i)
		{
			//glVertex3d(rope.links[i].pos(0), rope.links[i].pos(1), rope.links[i].pos(2));
			//glVertex3d(rope.links[i+1].pos(0), rope.links[i+1].pos(1), rope.links[i+1].pos(2));

			glVertex3d(rope2.links[i].pos(0), rope2.links[i].pos(1), rope2.links[i].pos(2));
			glVertex3d(rope2.links[i+1].pos(0), rope2.links[i+1].pos(1), rope2.links[i+1].pos(2));
		}
		glEnd();

		/*glBegin(GL_TRIANGLES);
		for (size_t i = 0; i < rope.links.size(); ++i)
		{
			glVertex3d(rope.links[i].pos(0)-0.025, rope.links[i].pos(1)-0.025, rope.links[i].pos(2));
			glVertex3d(rope.links[i].pos(0)+0.025, rope.links[i].pos(1)-0.025, rope.links[i].pos(2));
			glVertex3d(rope.links[i].pos(0), rope.links[i].pos(1)+0.025*sqrt(2), rope.links[i].pos(2));

			glVertex3d(rope2.links[i].pos(0)-0.025, rope2.links[i].pos(1)-0.025, rope2.links[i].pos(2));
			glVertex3d(rope2.links[i].pos(0)+0.025, rope2.links[i].pos(1)-0.025, rope2.links[i].pos(2));
			glVertex3d(rope2.links[i].pos(0), rope2.links[i].pos(1)+0.025*sqrt(2), rope2.links[i].pos(2));
		}
		glEnd();*/

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();

		rope.step(10);
		rope2.step(10);
	}

	glfwTerminate();
	return 0;
}