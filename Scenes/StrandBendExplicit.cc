#include "Camera.h"
#include "OpenGL.h"
#include <LibGui.h>
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
bool gShowGrid = false;
bool gShowCenterAxes = false;
bool gSaveFrame = false;

int gSteps = 0;

auto gCamera = std::make_shared<Camera<float>>();

// UNCOMMENT HERE FOR BUNNY SCENE
bool gStopped = false;
Real gCollisionEnvelopeCheck = 0.4;
std::shared_ptr<StrandMesh> gMesh;
std::shared_ptr<ForwardEulerStrand> gIntegrator;

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
    const auto &disp = gCamera->GetDisplacement().transpose();
    std::cout << "->SetRadius(" << gCamera->GetR() << ")" << std::endl;
    std::cout << "->SetTheta(" << gCamera->GetTheta() << ")" << std::endl;
    std::cout << "->SetPhi(" << gCamera->GetPhi() << ")" << std::endl;
    std::cout << "->SetDisplacement(Vec3<Real>(" << disp.x() << ", " << disp.y()
              << ", " << disp.z() << "))" << std::endl;

    const auto &eye = gCamera->GetEye();
    const auto &lookAt = gCamera->GetLookAt();
    const auto &up = gCamera->GetUp();

    std::cout << "Eye " << eye.x() << ", " << eye.y() << ", " << eye.z()
              << std::endl;
    std::cout << "Look At " << lookAt.x() << ", " << lookAt.y() << ", "
              << lookAt.z() << std::endl;
    std::cout << "Up " << up.x() << ", " << up.y() << ", " << up.z()
              << std::endl;
    std::cout << "FOV " << gCamera->GetFOV() << std::endl;
  }

  glutPostRedisplay();
}

void GlutSpecialInputFunc(int key, int x, int y) { glutPostRedisplay(); }

void Display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
  glMultMatrixf(gCamera->ToViewMatrix().data());

  if (gShowCenterAxes) {
    DrawCenterAxes();
  }

  // Show the frame number in the bottom right
  DrawText("Frame: " + std::to_string(gSteps), gScreenSize.x(),
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

  gMesh->Draw();

  glPopMatrix();
  glFlush();
}

static Vec<Real> gTranslationStart = Vec<Real>::LinSpaced(5000, 0, 2.5);
static Vec<Real> gTranslationEnd = Vec<Real>::LinSpaced(5000, -2.5, 0);
static int gTranslationIndex = 0;
static int gFramesSaved = 0;
static int gStopFrame = 10000;
static void GlutIdle() {
  if (gStopped) {
    return;
  }

  if (gAnimating || gSingleStep) {
    Vec3<Real> gravity(0, -0.981, 0);
    ++gSteps;

    gIntegrator->AddGravity(gravity);
    gIntegrator->Step();

    if (gSingleStep) {
      gSingleStep = !gSingleStep;
    }

    if (gSteps > 1000 && gTranslationIndex < gTranslationStart.rows()) {
      gMesh->TranslatePinnedX(gTranslationStart(gTranslationIndex), 0);
      gMesh->TranslatePinnedX(
          gTranslationEnd.reverse().eval()(gTranslationIndex), 4);
      ++gTranslationIndex;
    }
  }

  if (gSteps == gStopFrame) {
    gAnimating = false;
    gStopped = true;
  }

  if (gSteps % 10 == 0 && gAnimating) {
    if (gSaveFrame) {


      char filename[512];
      sprintf(filename, "/Users/jarredparr/Downloads/output/frame_%05i.obj",
              gFramesSaved);
      ++gFramesSaved;

      std::ofstream file(filename);

      const auto &vertices = gMesh->der->vertices;
      for (int jj = 0; jj < vertices.rows(); ++jj) {
        // Write vertices.row(ii) to the file
        file << "v " << vertices.row(jj) << std::endl;
      }

      file << "l ";
      for (int jj = 0; jj < vertices.rows(); ++jj) {
        int index = (jj + 1);
        file << index << " ";
      }

      file << std::endl;
      file.close();
    }

    glutPostRedisplay();
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

  // Set initial camera zoom
  gCamera->Zoom(10);

  // Set the mesh
  Mat<Real> v(5, 3);
  for (int ii = 0; ii < v.rows(); ++ii) {
    v.row(ii) = Vec3<Real>(ii, 0, 0);
  }

  gMesh = std::make_shared<StrandMesh>(v);
  gMesh->pinned(0) = 1;
  gMesh->pinned(v.rows() - 1) = 1;
  gIntegrator = std::make_shared<ForwardEulerStrand>(gMesh, nullptr,
                                                     1.0 / 1'000.0, 5, 0.3);

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
