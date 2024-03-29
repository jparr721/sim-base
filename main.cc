#include "Camera.h"
#include "OpenGL.h"
#include <Scenes/StrandDropScene.h>
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
bool gShowGrid = true;
bool gShowCenterAxes = true;
bool gSaveFrame = false;

// Gravity interactions
bool gAddPosYAxisPull = false;
bool gAddNegYAxisPull = false;
bool gAddPosXAxisPull = false;
bool gAddNegXAxisPull = false;

int gSteps = 0;

auto gCamera = std::make_shared<Camera<float>>();

// UNCOMMENT HERE FOR BUNNY SCENE
auto gScene = std::make_unique<DiscreteElasticRods>(gCamera);
// auto gScene = std::make_unique<CoarseBunnyExplicit>();

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
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    gCameraRotating = true;
  }
  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
    gCameraRotating = false;
  }

  if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
    gCameraZooming = true;
  }
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP) {
    gCameraZooming = false;
  }

  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {
    gCameraPanning = true;
  }
  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_UP) {
    gCameraPanning = false;
  }

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
  std::cout << "  a: Toggle animation" << std::endl;
  std::cout << "  c: Print camera info" << std::endl;
  std::cout << "  g: Toggle grid mesh" << std::endl;
  std::cout << "  space: Single step of animation" << std::endl;
}

void GlutKeyboardFunc(unsigned char key, int x, int y) {
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

  if (key == 'g') {
    gShowGrid = !gShowGrid;
    gShowCenterAxes = !gShowCenterAxes;
  }

  if (key == 'c') {
    std::cout << gCamera->Parameters() << std::endl;
  }

  glutPostRedisplay();
}

void GlutSpecialInputFunc(int key, int x, int y) {
  if (key == GLUT_KEY_UP) {
    gAddPosYAxisPull = true;
  }

  if (key == GLUT_KEY_DOWN) {
    gAddNegYAxisPull = true;
  }

  if (key == GLUT_KEY_LEFT) {
    gAddNegXAxisPull = true;
  }

  if (key == GLUT_KEY_RIGHT) {
    gAddPosXAxisPull = true;
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

static void DrawText(const std::string &text, int windowWidth,
                     int windowHeight) {
  // Set the current matrix mode to "projection"
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  // Set the viewport to cover the entire screen
  glViewport(0, 0, windowWidth, windowHeight);
  // Set the orthographic projection
  glOrtho(0, windowWidth, 0, windowHeight, -1, 1);
  // Set the current matrix mode to "modelview"
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  // Translate to the bottom right corner of the screen
  int string_width = glutBitmapLength(GLUT_BITMAP_HELVETICA_18,
                                      (const unsigned char *)text.c_str());
  glTranslatef(windowWidth - (string_width + 10), 10, 0);
  // Set the raster position to the bottom left corner of the screen
  glRasterPos2i(0, 0);
  // Loop through the characters in the string and draw them using
  // glutBitmapCharacter
  glColor3f(1, 1, 1);
  for (const auto &c : text) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
  }
  // Pop the modelview matrix
  glPopMatrix();
  // Pop the projection matrix
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  // Set the current matrix mode back to "modelview"
  glMatrixMode(GL_MODELVIEW);
}

void Display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
  glMultMatrixf(gCamera->ToViewMatrix().data());

  if (gShowCenterAxes) {
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
  }

  // Show the frame number in the bottom right
  DrawText("Frame: " + std::to_string(gScene->frame), gScreenSize.x(),
           gScreenSize.y());

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

  if (gShowGrid) {
    DrawGLGrid(100, 0.25);
  }

  gScene->Draw();

  glPopMatrix();
  glFlush();
}

static void GlutIdle() {
  if (gAnimating || gSingleStep) {
    Vec3<Real> gravity(0, -0.981, 0);

    if (gAddPosYAxisPull) {
      gravity += Vec3<Real>(0, 9, 0);
      gAddPosYAxisPull = false;
    }

    if (gAddNegYAxisPull) {
      gravity += Vec3<Real>(0, -9, 0);
      gAddNegYAxisPull = false;
    }

    if (gAddPosXAxisPull) {
      gravity += Vec3<Real>(9, 0, 0);
      gAddPosXAxisPull = false;
    }

    if (gAddNegXAxisPull) {
      gravity += Vec3<Real>(-9, 0, 0);
      gAddNegXAxisPull = false;
    }

    gScene->StepScriptedScene(gravity);

    ++gSteps;

    if (gSingleStep) {
      gSingleStep = !gSingleStep;
    }
  }

  if (gSteps % 50 == 0 && gAnimating) {
    if (gSaveFrame) {
      gScene->DumpFrame();
    }
    glutPostRedisplay();
    ++gScene->frame;
    if (gScene->frame > 400) {
      ++gScene->sceneFrames;
    }
  }
}

auto main(int argc, char **argv) -> int {
  if (argc > 1) {
    gSaveFrame = strcmp(argv[1], "--save") == 0;
  } else {
    std::cout << "No Scene specified, defaulting to Scenes/CoarseBunnyExplicit"
              << std::endl;
  }

  KeyboardControls();

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA);
  glutInitWindowSize(gScreenSize.x(), gScreenSize.y());
  glutCreateWindow("Simulator");
  glutDisplayFunc(Display);
  glutMouseFunc(GlutMouseFunc);
  glutMotionFunc(GlutMotionFunc);
  glutReshapeFunc(GlutReshapeFunc);
  glutKeyboardFunc(GlutKeyboardFunc);
  glutSpecialFunc(GlutSpecialInputFunc);
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
