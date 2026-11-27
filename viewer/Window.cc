#include <Window.h>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <optional>
#include <stdexcept>

namespace viewer {
namespace {

auto TranslateKey(int key) -> std::optional<Key> {
  switch (key) {
  case GLFW_KEY_A:
    return Key::A;
  case GLFW_KEY_SPACE:
    return Key::Space;
  case GLFW_KEY_G:
    return Key::G;
  case GLFW_KEY_UP:
    return Key::Up;
  case GLFW_KEY_DOWN:
    return Key::Down;
  case GLFW_KEY_LEFT:
    return Key::Left;
  case GLFW_KEY_RIGHT:
    return Key::Right;
  default:
    return std::nullopt;
  }
}

auto TranslateButton(int button) -> std::optional<MouseButton> {
  switch (button) {
  case GLFW_MOUSE_BUTTON_LEFT:
    return MouseButton::Left;
  case GLFW_MOUSE_BUTTON_RIGHT:
    return MouseButton::Right;
  case GLFW_MOUSE_BUTTON_MIDDLE:
    return MouseButton::Middle;
  default:
    return std::nullopt;
  }
}

} // namespace

auto Window::Self(GLFWwindow *window) -> Window & {
  return *static_cast<Window *>(glfwGetWindowUserPointer(window));
}

Window::Window(const char *title, int width, int height) {
  if (glfwInit() == GLFW_FALSE) {
    throw std::runtime_error("glfwInit failed");
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4);
  window = glfwCreateWindow(width, height, title, nullptr, nullptr);
  if (window == nullptr) {
    glfwTerminate();
    throw std::runtime_error("cannot create an OpenGL 4.1 core window");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) ==
      0) {
    glfwDestroyWindow(window);
    glfwTerminate();
    throw std::runtime_error("glad could not load OpenGL");
  }

  glfwSetWindowUserPointer(window, this);
  glfwGetCursorPos(window, &cursorX, &cursorY);
  glfwSetKeyCallback(window, [](GLFWwindow *w, int key, int, int action, int) {
    if (action == GLFW_RELEASE) {
      return;
    }
    if (const auto k = TranslateKey(key)) {
      Self(w).pending.emplace_back(KeyPressed{*k});
    }
  });
  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *w, int button, int action, int mods) {
        if (const auto b = TranslateButton(button)) {
          Self(w).pending.emplace_back(MouseButtonChanged{
              *b, action == GLFW_PRESS, (mods & GLFW_MOD_SHIFT) != 0});
        }
      });
  glfwSetCursorPosCallback(window, [](GLFWwindow *w, double x, double y) {
    auto &self = Self(w);
    self.pending.emplace_back(MouseMoved{x - self.cursorX, y - self.cursorY});
    self.cursorX = x;
    self.cursorY = y;
  });
  glfwSetScrollCallback(window, [](GLFWwindow *w, double, double dy) {
    Self(w).pending.emplace_back(Scrolled{dy});
  });
  glfwSetFramebufferSizeCallback(window, [](GLFWwindow *w, int fw, int fh) {
    Self(w).pending.emplace_back(FramebufferResized{fw, fh});
  });
}

Window::~Window() {
  glfwDestroyWindow(window);
  glfwTerminate();
}

auto Window::ShouldClose() const -> bool {
  return glfwWindowShouldClose(window) != 0;
}

auto Window::PollEvents() -> std::vector<Event> {
  glfwPollEvents();
  return std::exchange(pending, {});
}

void Window::SwapBuffers() { glfwSwapBuffers(window); }

auto Window::FramebufferSize() const -> std::pair<int, int> {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  return {width, height};
}

} // namespace viewer
