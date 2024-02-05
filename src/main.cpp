#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <string>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <Eigen/Dense>

#include "GLSL.h"
#include "Program.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Texture.h"
#include "Link.h"

#include "ObjectiveRosenbrock.h"
#include "ObjectiveIK.h"
#include "OptimizerGD.h"
#include "OptimizerBFGS.h"
#include "OptimizerNM.h"

using namespace std;
using namespace glm;
using namespace Eigen;

char taskLetter = 'A';
int taskNumber = 1;

bool keyToggles[256] = {false}; // only for English keyboards!

GLFWwindow *window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from

shared_ptr<Program> progSimple;
shared_ptr<Program> progTex;
shared_ptr<Shape> shape;
shared_ptr<Texture> texture;

vector<shared_ptr<Link> > links;

class IK
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int nlinks;
	Vector3d weights;
	Vector2d target;
};
IK ik;

static string testRosenbrock()
{
	stringstream output;
	auto objective = make_shared<ObjectiveRosenbrock>();
	
	// TODO: Implement Task A
	
	// Eigen vectors and matrices can be written out with
	// the << operator
	Vector2d a;
	//a << 1.0, 2.0;
	//output << a << endl;
	a << -1, 0;

	switch (taskNumber) {
		case 1:
		{
			//---------- Task A.1 testing --------------
			VectorXd grad(2);
			MatrixXd hess(2, 2);
			double answer = objective->evalObjective(a, grad, hess);
			output << answer << endl;
			output << grad << endl;
			output << hess << endl;
			//---------- Task A.1 testing --------------
			break;
		}
			
		case 2:
		{
			//---------- Task A.2 testing --------------
			shared_ptr<OptimizerGD> gd;
			gd = make_shared<OptimizerGD>();
			gd->setTol(pow(10, -6));
			gd->setIterMax(50);
			int iterations;
			VectorXd newx = gd->optimize(objective, a);
			iterations = gd->iterationsUsed;
			output << iterations << endl;
			output << newx;
			//---------- Task A.2 testing --------------
			break;
		}

		case 3: 
		{
			//---------- Task A.3 testing --------------
			shared_ptr<OptimizerGD> gd;
			gd = make_shared<OptimizerGD>();
			gd->setTol(pow(10, -6));
			gd->setAlphaInit(1);
			gd->setGamma(0.8);
			gd->setIterMaxLS(20);
			gd->setIterMax(50);
			int iterations;
			gd->useLineSearch = true;
			VectorXd newx = gd->optimize(objective, a);
			iterations = gd->iterationsUsed;
			gd->useLineSearch = false;
			output << iterations << endl;
			output << newx;
			//---------- Task A.3 testing --------------
			break;
		}
		case 4: 
		{
			//---------- Task A.4 testing --------------
			shared_ptr<OptimizerNM> nm;
			nm = make_shared<OptimizerNM>();
			nm->setIterMax(50);
			nm->setTol(pow(10, -6));
			nm->setAlphaInit(1);
			nm->setIterMaxLS(20);
			nm->setGamma(0.8);
			int iterations;
			VectorXd newx = nm->optimize(objective, a);
			iterations = nm->iterationsUsed;
			output << iterations << endl;
			output << newx;
			//---------- Task A.4 testing --------------
			break;
		}
		case 5:
		{
			//---------- Task A.5 testing --------------
			shared_ptr<OptimizerBFGS> bfgs;
			bfgs = make_shared<OptimizerBFGS>();
			bfgs->setTol(pow(10, -6));
			bfgs->setAlphaInit(1);
			bfgs->setIterMax(50);
			bfgs->setIterMaxLS(20);
			bfgs->setGamma(0.8);
			int iterations;
			VectorXd newx = bfgs->optimize(objective, a);
			iterations = bfgs->iterationsUsed;
			output << iterations << endl;
			output << newx;
			//---------- Task A.5 testing --------------
			break;
		}
			

	}

	return output.str();
}

static void createLinks()
{
	switch(taskNumber) {
		case 1:
			ik.nlinks = 1;
			ik.target << 0.0, 1.0;
			ik.weights << 1e3, 1e0, 0.0;
			break;
		case 2:
			ik.nlinks = 2;
			ik.target << 1.0, 1.0;
			ik.weights << 1e3, 1e0, 1e0;
			break;
		case 3:
			ik.nlinks = 4;
			ik.target << 3.0, 1.0;
			ik.weights << 1e3, 0.0, 1e0;
			break;
		case 4:
		case 5:
			ik.nlinks = 10;
			ik.target << 9.0, 1.0;
			ik.weights << 1e3, 0.0, 1e0;
			break;
	}
	
	// Create the links
	Matrix4d E(Matrix4d::Identity());
	for(int i = 0; i < ik.nlinks; ++i) {
		auto link = make_shared<Link>();
		links.push_back(link);
		link->setAngle(0.0);
		link->setPosition((i == 0 ? 0.0 : 1.0), 0.0);
		E(0,3) = 0.5;
		link->setMeshMatrix(E);
		if(i > 0) {
			links[i-1]->addChild(links[i]);
		}
	}
}

static string runIK()
{
	stringstream output;
	
	int n = ik.nlinks;
	const Vector3d &weights = ik.weights;
	const Vector2d &target = ik.target;
	
	// Extract angles
	VectorXd x(n);
	for(int i = 0; i < n; ++i) {
		x(i) = links[i]->getAngle();
	}
	
	// TODO: Implement Task B
	switch (taskNumber) {
		case 1:
			{
				auto objectiveIK = make_shared<ObjectiveIK>();

				VectorXd a(1);
				a << 0;

				MatrixXd temp(2, 1);
				temp << 0, 1;

				objectiveIK->ptarget = temp;
				objectiveIK->wreg.push_back(1);
				VectorXd grad(1);

				shared_ptr<OptimizerBFGS> bfgs;
				bfgs = make_shared<OptimizerBFGS>();
				bfgs->setTol(pow(10, -6));
				bfgs->setAlphaInit(1);
				bfgs->setIterMax(5);
				bfgs->setIterMaxLS(20);
				bfgs->setGamma(0.5);
				int iterations;
				VectorXd newx = bfgs->optimize(objectiveIK, a);
				/*shared_ptr<OptimizerGD> gd;
				gd = make_shared<OptimizerGD>();
				gd->setTol(pow(10, -6));
				gd->setAlphaInit(1);
				gd->setIterMax(5);
				gd->setIterMaxLS(20);
				gd->setGamma(0.5);
				int iterations;
				gd->useLineSearch = true;
				VectorXd newx = gd->optimize(objectiveIK, a);*/
				
				int number = newx.size();

				for (int i = 0; i < number; i++) {
					double angle = newx(i);
					while (angle > M_PI) {
						angle -= (2 * M_PI);
					}
					while (angle < -M_PI) {
						angle += (2 * M_PI);
					}
					newx(i) = angle;
				}
				x = newx;
				iterations = bfgs->iterationsUsed;
				//iterations = gd->iterationsUsed;
				break;
			}

		case 2:
		{
			auto objectiveIK = make_shared<ObjectiveIK>();
			VectorXd a(2);
			a << 0, 0;
			MatrixXd temp(2, 1);
			temp << 1, 1;

			objectiveIK->ptarget = temp;
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);

			shared_ptr<OptimizerBFGS> bfgs;
			bfgs = make_shared<OptimizerBFGS>();
			bfgs->setTol(pow(10, -6));
			bfgs->setAlphaInit(1);
			bfgs->setIterMax(5);
			bfgs->setIterMaxLS(20);
			bfgs->setGamma(0.5);
			int iterations;
			VectorXd newx = bfgs->optimize(objectiveIK, a);

			int number = newx.size();

			for (int i = 0; i < number; i++) {
				double angle = newx(i);
				while (angle > M_PI) {
					angle -= (2 * M_PI);
				}
				while (angle < -M_PI) {
					angle += (2 * M_PI);
				}
				newx(i) = angle;
			}
			x = newx;
			iterations = bfgs->iterationsUsed;
			break;
		}
		case 3:
		{
			auto objectiveIK = make_shared<ObjectiveIK>();
			VectorXd a(4);
			a << 0, 0, 0, 0;
			MatrixXd temp(2, 1);
			temp << 3, 1;
			objectiveIK->ptarget = temp;
			objectiveIK->wreg.push_back(0);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			/*VectorXd gradient(4);
			objectiveIK->evalObjective(a);*/

			shared_ptr<OptimizerBFGS> bfgs;
			bfgs = make_shared<OptimizerBFGS>();
			bfgs->setTol(pow(10, -6));
			bfgs->setAlphaInit(1);
			bfgs->setIterMax(50);
			bfgs->setIterMaxLS(20);
			bfgs->setGamma(0.5);
			int iterations;
			VectorXd newx = bfgs->optimize(objectiveIK, a);
			/*shared_ptr<OptimizerGD> gd;
			gd = make_shared<OptimizerGD>();
			int iterations;
			gd->useLineSearch = true;
			VectorXd newx = gd->optimize(objectiveIK, a);*/

			int number = newx.size();

			for (int i = 0; i < number; i++) {
				double angle = newx(i);
				while (angle > M_PI) {
					angle -= (2 * M_PI);
				}
				while (angle < -M_PI) {
					angle += (2 * M_PI);
				}
				newx(i) = angle;
			}
			x = newx;
			iterations = bfgs->iterationsUsed;
			//iterations = gd->iterationsUsed;

			break;
		}
		case 4:
		{
			auto objectiveIK = make_shared<ObjectiveIK>();
			VectorXd a(10);
			a << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
			MatrixXd temp(2, 1);
			temp << 9, 1;
			objectiveIK->ptarget = temp;
			objectiveIK->wreg.push_back(0);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);
			objectiveIK->wreg.push_back(1);

			shared_ptr<OptimizerBFGS> bfgs;
			bfgs = make_shared<OptimizerBFGS>();
			bfgs->setTol(pow(10, -6));
			bfgs->setAlphaInit(1);
			bfgs->setIterMax(150);
			bfgs->setIterMaxLS(20);
			bfgs->setGamma(0.5);
			int iterations;
			VectorXd newx = bfgs->optimize(objectiveIK, a);

			int number = newx.size();

			for (int i = 0; i < number; i++) {
				double angle = newx(i);
				while (angle > M_PI) {
					angle -= (2 * M_PI);
				}
				while (angle < -M_PI) {
					angle += (2 * M_PI);
				}
				newx(i) = angle;
			}
			x = newx;
			iterations = bfgs->iterationsUsed;
			
			break;
		}
	}
	
	// Set angles
	for(int i = 0; i < n; ++i) {
		double xi = x(i);
		links[i]->setAngle(xi);
	}
	
	// Write angles to output
	for(const auto &link : links) {
		output << link->getAngle() << endl;
	}
	
	return output.str();
}

static void writeOutput(const string &output)
{
	string filename = RESOURCE_DIR + "output" + taskLetter + to_string(taskNumber) + ".txt";
	ofstream out;
	out.open(filename, ofstream::out);
	if(!out.good()) {
		std::cout << "Cannot write to " << filename << endl;
		return;
	}
	std::cout << "Writing to " << filename << endl;
	out << output;
	out.close();
}

static void error_callback(int error, const char *description)
{
	cerr << description << endl;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
}

static void char_callback(GLFWwindow *window, unsigned int key)
{
	keyToggles[key] = !keyToggles[key];
	switch(key) {
		case 'r':
			// Reset all angles to 0.0
			for(auto link : links) {
				link->setAngle(0.0);
			}
			break;
		case '.':
			// Increment all angles
			if(!keyToggles[(unsigned)' ']) {
				for(auto link : links) {
					link->setAngle(link->getAngle() + 0.1);
				}
			}
			break;
		case ',':
			// Decrement all angles
			if(!keyToggles[(unsigned)' ']) {
				for(auto link : links) {
					link->setAngle(link->getAngle() - 0.1);
				}
			}
			break;
	}
}

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
	// Get current window size.
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	// Convert from window coord to world coord assuming that we're
	// using an orthgraphic projection
	double aspect = (double)width/height;
	double ymax = (double)links.size();
	double xmax = aspect*ymax;
	Vector2d x;
	x(0) = 2.0 * xmax * ((xmouse / width) - 0.5);
	x(1) = 2.0 * ymax * (((height - ymouse) / height) - 0.5);
	if(keyToggles[(unsigned)' ']) {
		ik.target = x;
		runIK();
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	// Get the current mouse position.
	double xmouse, ymouse;
	glfwGetCursorPos(window, &xmouse, &ymouse);
}

static void init()
{
	GLSL::checkVersion();
	
	// Set background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);
	
	keyToggles[(unsigned)'c'] = true;
	
	progSimple = make_shared<Program>();
	progSimple->setShaderNames(RESOURCE_DIR + "simple_vert.glsl", RESOURCE_DIR + "simple_frag.glsl");
	progSimple->setVerbose(true); // Set this to true when debugging.
	progSimple->init();
	progSimple->addUniform("P");
	progSimple->addUniform("MV");
	progSimple->setVerbose(false);
	
	progTex = make_shared<Program>();
	progTex->setVerbose(true); // Set this to true when debugging.
	progTex->setShaderNames(RESOURCE_DIR + "tex_vert.glsl", RESOURCE_DIR + "tex_frag.glsl");
	progTex->init();
	progTex->addUniform("P");
	progTex->addUniform("MV");
	progTex->addAttribute("aPos");
	progTex->addAttribute("aTex");
	progTex->addUniform("texture0");
	progTex->setVerbose(false);
	
	texture = make_shared<Texture>();
	texture->setFilename(RESOURCE_DIR + "metal_texture_15_by_wojtar_stock.jpg");
	texture->init();
	texture->setUnit(0);
	
	shape = make_shared<Shape>();
	shape->loadMesh(RESOURCE_DIR + "link.obj");
	shape->setProgram(progTex);
	shape->init();
	
	// Initialize time.
	glfwSetTime(0.0);
	
	// If there were any OpenGL errors, this will print something.
	// You can intersperse this line in your code to find the exact location
	// of your OpenGL error.
	GLSL::checkError(GET_FILE_LINE);
}

void render()
{
	// Get current frame buffer size.
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(keyToggles[(unsigned)'c']) {
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
	if(keyToggles[(unsigned)'l']) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	
	auto P = make_shared<MatrixStack>();
	auto MV = make_shared<MatrixStack>();
	P->pushMatrix();
	MV->pushMatrix();
	
	// Apply camera transforms
	double aspect = (double)width/height;
	double ymax = (double)links.size();
	double xmax = aspect*ymax;
	P->multMatrix(glm::ortho(-xmax, xmax, -ymax, ymax, -1.0, 1.0));
	
	// Draw grid
	progSimple->bind();
	glUniformMatrix4fv(progSimple->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
	glUniformMatrix4fv(progSimple->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	// Draw axes
	glLineWidth(2.0f);
	glColor3d(0.2, 0.2, 0.2);
	glBegin(GL_LINES);
	glVertex2d(-xmax, 0.0);
	glVertex2d( xmax, 0.0);
	glVertex2d(0.0, -ymax);
	glVertex2d(0.0,  ymax);
	glEnd();
	// Draw grid lines
	glLineWidth(1.0f);
	glColor3d(0.8, 0.8, 0.8);
	glBegin(GL_LINES);
	for(int x = 1; x < xmax; ++x) {
		glVertex2d( x, -ymax);
		glVertex2d( x,  ymax);
		glVertex2d(-x, -ymax);
		glVertex2d(-x,  ymax);
	}
	for(int y = 1; y < ymax; ++y) {
		glVertex2d(-xmax,  y);
		glVertex2d( xmax,  y);
		glVertex2d(-xmax, -y);
		glVertex2d( xmax, -y);
	}
	glEnd();
	progSimple->unbind();
	
	// Draw shape
	progTex->bind();
	texture->bind(progTex->getUniform("texture0"));
	glUniformMatrix4fv(progTex->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
	MV->pushMatrix();
	if(!links.empty()) {
		links.front()->draw(progTex, MV, shape);
	}
	MV->popMatrix();
	texture->unbind();
	progTex->unbind();
	
	// Pop stacks
	MV->popMatrix();
	P->popMatrix();
	
	GLSL::checkError(GET_FILE_LINE);
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		std::cout << "Please specify the resource directory." << endl;
		return 0;
	}
	RESOURCE_DIR = argv[1] + string("/");
	
	if(argc < 3) {
		std::cout << "Please specify the task type." << endl;
	}
	string type = argv[2];
	if(type.length() < 3) {
		std::cout << "Please specify A.1-A.5 or B.1-B.5." << endl;
	}
	taskLetter = type.at(0);
	taskNumber = type.at(2) - '0';
	
	if(taskLetter == 'A') {
		// Task A
		string output = testRosenbrock();
		writeOutput(output);
		return 0;
	}
	
	// Task B
	bool useGL = (argc == 4);
	if(useGL) {
		// Set error callback.
		glfwSetErrorCallback(error_callback);
		// Initialize the library.
		if(!glfwInit()) {
			return -1;
		}
		// Create a windowed mode window and its OpenGL context.
		window = glfwCreateWindow(640, 480, "YOUR NAME", NULL, NULL);
		if(!window) {
			glfwTerminate();
			return -1;
		}
		// Make the window's context current.
		glfwMakeContextCurrent(window);
		// Initialize GLEW.
		glewExperimental = true;
		if(glewInit() != GLEW_OK) {
			cerr << "Failed to initialize GLEW" << endl;
			return -1;
		}
		glGetError(); // A bug in glewInit() causes an error that we can safely ignore.
		std::cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
		std::cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
		// Set vsync.
		glfwSwapInterval(1);
		// Set keyboard callback.
		glfwSetKeyCallback(window, key_callback);
		// Set char callback.
		glfwSetCharCallback(window, char_callback);
		// Set cursor position callback.
		glfwSetCursorPosCallback(window, cursor_position_callback);
		// Set mouse button callback.
		glfwSetMouseButtonCallback(window, mouse_button_callback);
		// Initialize scene.
		init();
		createLinks();
		// Loop until the user closes the window.
		while(!glfwWindowShouldClose(window)) {
			if(!glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
				// Render scene.
				render();
				// Swap front and back buffers.
				glfwSwapBuffers(window);
			}
			// Poll for and process events.
			glfwPollEvents();
		}
		// Quit program.
		glfwDestroyWindow(window);
		glfwTerminate();
	} else {
		createLinks();
		string output = runIK();
		writeOutput(output);
	}
	return 0;
}
