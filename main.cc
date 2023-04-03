#include "Camera.h"
#include "Math.h"
#include "OpenGL.h"
#include <array>
#include <memory>

using Real = float;

Eigen::Vector2i gMouseDiff;
Eigen::Vector2i gMouseLast;
Eigen::Vector2i gMouseCur;

Eigen::Vector2i screenSize(1000, 800);

bool gCameraRotating = false;
bool gCameraZooming = false;
bool gCameraPanning = false;

auto gCamera = std::make_unique<Camera<Real>>();

constexpr std::array<Real, 4> gAmbientFull = {{0.4, 0.4, 0.4, 1.0}};
constexpr std::array<Real, 4> gSpecularFull = {1.0, 1.0, 1.0, 1.0};
constexpr std::array<Real, 1> gShininess = {50.0};
constexpr std::array<Real, 4> gLightPosition = {1.0, 1.0, 1.0, 1.0};
constexpr std::array<Real, 4> gDiffuseBlue = {0.3, 0.6, 1.0, 1.0};
constexpr std::array<Real, 4> gDiffuseGray = {0.8, 0.8, 0.8, 1.0};
constexpr std::array<Real, 4> gDiffuseOther = {0.3, 0.3, 1.0, 1.0};

void GlutMotionFunc(int x, int y) {
  gMouseCur[0] = x;
  gMouseCur[1] = y;

  Real xDiff = gMouseLast[0] - gMouseCur[0];
  Real yDiff = gMouseLast[1] - gMouseCur[1];

  if (gCameraPanning) {
    gCamera->Pan(xDiff, yDiff);
  }
  if (gCameraRotating) {
    gCamera->Rotate(xDiff, yDiff);
  }
  if (gCameraZooming) {
    gCamera->Zoom(yDiff * 0.1);
  }

  gMouseLast[0] = gMouseCur[0];
  gMouseLast[1] = gMouseCur[1];

  glutPostRedisplay();
}

void GlutMouseFunc(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    gCameraRotating = true;
  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    gCameraRotating = false;

  if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
    gCameraZooming = true;
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
    gCameraZooming = false;

  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
    gCameraPanning = true;
  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_UP)
    gCameraPanning = false;

  gMouseCur[0] = x;
  gMouseCur[1] = y;
  gMouseLast[0] = gMouseCur[0];
  gMouseLast[1] = gMouseCur[1];

  glutPostRedisplay();
}

void GlutReshapeFunc(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gCamera->Resize(width, height);
  glMultMatrixf(gCamera->GetProjectionMatrix().data());
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glutPostRedisplay();
}

void GlutKeyboardFunc(unsigned char key, int x, int y) { glutPostRedisplay(); }

void Display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
  glMultMatrixf(gCamera->ToViewMatrix().data());

  glBegin(GL_LINES);
  // Red - X Axis
  glColor3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);

  // Green - Y Axis
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0);

  // Blue - Z Axis
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 1);
  glEnd();

  glPopMatrix();
  glFlush();
}

auto main(int argc, char **argv) -> int {
  // Set initial camera zoom
  gCamera->Zoom(10);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA);
  glutInitWindowSize(screenSize.x(), screenSize.y());
  glutCreateWindow("Testing");
  glutDisplayFunc(Display);
  glutMouseFunc(GlutMouseFunc);
  glutMotionFunc(GlutMotionFunc);
  glutReshapeFunc(GlutReshapeFunc);
  glutKeyboardFunc(GlutKeyboardFunc);

  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glPointSize(4.0f);
  glEnable(GL_DEPTH_TEST);

  // USE THESE TO SHADE MATERIALS
  // glShadeModel(GL_SMOOTH);
  // glMaterialfv(GL_FRONT, GL_DIFFUSE, kDiffuseGray.data());
  // glMaterialfv(GL_FRONT, GL_AMBIENT, kAmbientFull.data());
  // glMaterialfv(GL_FRONT, GL_SPECULAR, kSpecularFull.data());
  // glMaterialfv(GL_FRONT, GL_SHININESS, kShininess.data());
  // glLightfv(GL_LIGHT0, GL_POSITION, kLightPosition.data());

  // glEnable(GL_LIGHTING);
  // glEnable(GL_LIGHT0);

  glutMainLoop();
}
