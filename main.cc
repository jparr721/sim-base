#include "Camera.h"
#include "Math.h"
#include "OpenGL.h"
#include "SNH.h"
#include "STVK.h"
#include "TetMesh.h"
#include <memory>

Eigen::Vector2i gMouseDiff;
Eigen::Vector2i gMouseLast;
Eigen::Vector2i gMouseCur;

Eigen::Vector2i screenSize(1000, 800);

bool gCameraRotating = false;
bool gCameraZooming = false;
bool gCameraPanning = false;

auto gCamera = std::make_unique<Camera<float>>();

TetMesh gMesh(Meshes / "bunny_oded.obj");

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

void GlutKeyboardFunc(unsigned char key, int x, int y) {
  if (key == 'n') {
    gMesh.ToggleNormals();
  }
  glutPostRedisplay();
}

static void DrawGLGrid(int size, float spacing) {
  glColor3f(0.5, 0.5, 0.5);
  glBegin(GL_LINES);
  for (float ii = -size; ii < size; ii += spacing) {
    glVertex3f(ii, 0, -size);
    glVertex3f(ii, 0, size);

    glVertex3f(size, 0, ii);
    glVertex3f(-size, 0, ii);
  }
  glEnd();
}

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

  float density = 0.15;
  float fogColor[4] = {0.15, 0.15, 0.15, 1.0};
  float fogStart[1] = {0.0f};
  float fogEnd[1] = {800.0f};

  glFogi(GL_FOG_MODE, GL_LINEAR);
  glFogfv(GL_FOG_START, fogStart);
  glFogfv(GL_FOG_END, fogEnd);
  glFogfv(GL_FOG_COLOR, fogColor);
  glFogf(GL_FOG_DENSITY, density);
  glHint(GL_FOG_HINT, GL_NICEST);
  DrawGLGrid(100, 0.25);

  gMesh.Draw();

  glPopMatrix();
  glFlush();
}

auto main(int argc, char **argv) -> int {
  bool isTesting = argc > 1 && std::string(argv[1]) == "test";
  if (argc > 1 && !isTesting) {
    gMesh = TetMesh(argv[1]);
  }

  if (isTesting) {
    // Test energy functions
    auto stvkEnergy = std::make_unique<STVK>(1, 1);
    if (!stvkEnergy->FiniteDifferenceTestPk1(Mat3<Real>::Random() * 10)) {
      return EXIT_FAILURE;
    }

    auto snhEnergy = std::make_unique<SNH>(1, 1);
    if (!snhEnergy->FiniteDifferenceTestPk1(Mat3<Real>::Random())) {
      return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
  }

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

  glClearColor(0.15, 0.15, 0.15, 1.0f);

  GLfloat lightPosition[] = {0.0, 0.0, 0.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

  glEnable(GL_FOG);

  glutMainLoop();
}
