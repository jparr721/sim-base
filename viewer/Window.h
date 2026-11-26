#pragma once

#include <utility>
#include <variant>
#include <vector>

struct GLFWwindow;

namespace viewer {

enum class Key { A, Space, G, Up, Down, Left, Right };
enum class MouseButton { Left, Right, Middle };

struct KeyPressed {
  Key key;
};
struct MouseButtonChanged {
  MouseButton button;
  bool down;
  bool shift;
};
struct MouseMoved {
  double dx, dy; // cursor delta in window coordinates
};
struct Scrolled {
  double dy;
};
struct FramebufferResized {
  int width, height;
};

using Event = std::variant<KeyPressed, MouseButtonChanged, MouseMoved,
                           Scrolled, FramebufferResized>;

// A GLFW window with a current OpenGL 4.1 core forward-compatible context and
// glad loaded. Translates GLFW input into viewer events.
class Window {
public:
  Window(const char *title, int width, int height);
  ~Window();
  Window(const Window &) = delete;
  auto operator=(const Window &) -> Window & = delete;

  [[nodiscard]] auto ShouldClose() const -> bool;
  // Polls GLFW and returns the events since the last call.
  auto PollEvents() -> std::vector<Event>;
  void SwapBuffers();
  [[nodiscard]] auto FramebufferSize() const -> std::pair<int, int>;
  [[nodiscard]] auto Handle() const -> GLFWwindow * { return window; }

private:
  GLFWwindow *window = nullptr;
  std::vector<Event> pending;
  double cursorX = 0;
  double cursorY = 0;

  static auto Self(GLFWwindow *window) -> Window &;
};

} // namespace viewer
