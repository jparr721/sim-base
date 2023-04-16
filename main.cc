#include "Camera.h"
#include "ForwardEuler.h"
#include "LibMath.h"
#include "OpenGL.h"
#include <Energy/Volume/SNH.h>
#include <Energy/Volume/STVK.h>
#include "TetMesh.h"
#include <igl/writeOBJ.h>
#include <memory>

Eigen::Vector2i gMouseDiff;
Eigen::Vector2i gMouseLast;
Eigen::Vector2i gMouseCur;

Eigen::Vector2i gScreenSize(1000, 800);

bool gCameraRotating = false;
bool gCameraZooming = false;
bool gCameraPanning = false;
bool gAnimating = false;
bool gSingleStep = false;

int gSteps = 0;

auto gCamera = std::make_unique<Camera<float>>();

std::shared_ptr<TetMesh> gMesh =
    std::make_shared<TetMesh>(Meshes / "bunny.obj");
std::shared_ptr<SNH> gMaterial = std::make_shared<SNH>(30.0, 0.45);
//std::shared_ptr<STVK> gMaterial = std::make_shared<STVK>(30.0, 0.45);
std::unique_ptr<ForwardEuler<Real>> gIntegrator =
    std::make_unique<ForwardEuler<Real>>(gMesh, gMaterial, 1.0 / 3000.0);

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

static void KeyboardControls() {
  std::cout << "Keyboard Controls:" << std::endl;
  std::cout << "  n: Toggle drawing normals" << std::endl;
  std::cout << "  a: Toggle animation" << std::endl;
  std::cout << "  c: Print camera info" << std::endl;
  std::cout << "  space: Single step of animation" << std::endl;
}

void GlutKeyboardFunc(unsigned char key, int x, int y) {
  if (key == 'n') {
    gMesh->ToggleDrawNormals();
  }

  if (key == ' ') {
    // Pause the animation too
    if (gAnimating) {
      gAnimating = false;
    }
    gSingleStep = !gSingleStep;
  }

  if (key == 'a') {
    gAnimating = !gAnimating;
  }

  if (key == 'c') {
    std::cout << "Radius " << gCamera->GetR() << std::endl;
    std::cout << "Theta " << gCamera->GetTheta() << std::endl;
    std::cout << "Phi " << gCamera->GetPhi() << std::endl;
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

  gMesh->Draw();

  glPopMatrix();
  glFlush();
}

static void GlutIdle() {
  if (gAnimating || gSingleStep) {
    gIntegrator->AddGravity(Vec3<Real>(0, -9, 0));
    gIntegrator->Step();

    ++gSteps;

    if (gSingleStep) {
      gSingleStep = !gSingleStep;
    }
  }

  if (gSteps % 100 == 0) {
    glutPostRedisplay();
  }
}

auto main(int argc, char **argv) -> int {
  if (argc > 1) {
    gMesh = std::make_shared<TetMesh>(argv[1]);

  } else {
    std::cout << "No mesh specified, defaulting to Meshes/bunny.obj"
              << std::endl;
  }

  // Pin the top vertices of the mesh in gMesh
  for (int ii = 0; ii < gMesh->v.rows(); ++ii) {
    // Find the top vertices
    if (gMesh->v(ii, 1) > 0.9) {
      gMesh->PinVertex(ii);
    }
  }

  KeyboardControls();

  // Set initial camera zoom
  gCamera->Zoom(10);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA);
  glutInitWindowSize(gScreenSize.x(), gScreenSize.y());
  glutCreateWindow("Testing");
  glutDisplayFunc(Display);
  glutMouseFunc(GlutMouseFunc);
  glutMotionFunc(GlutMotionFunc);
  glutReshapeFunc(GlutReshapeFunc);
  glutKeyboardFunc(GlutKeyboardFunc);
  glutIdleFunc(GlutIdle);

  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glPointSize(4.0f);
  glEnable(GL_DEPTH_TEST);

  glClearColor(0.15, 0.15, 0.15, 1.0f);

  GLfloat lightPosition[] = {0.0, 0.0, 0.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

  glEnable(GL_FOG);

  glutMainLoop();
}
