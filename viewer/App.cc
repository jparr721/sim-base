#include <App.h>
#include <Geometry.h>
#include <Renderer.h>
#include <ViewerState.h>
#include <Window.h>
#include <ui/Panels.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <format>
#include <iostream>
#include <string_view>

namespace viewer {
namespace {

constexpr std::string_view kUsage =
    "usage: sim_viewer [--scene <name>] [--export <dir>]";

constexpr sim::Real kPull = 9;

enum class Drag { None, Rotate, Zoom, Pan };

struct ImGuiSession {
  explicit ImGuiSession(const Window &window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window.Handle(), true);
    ImGui_ImplOpenGL3_Init("#version 410 core");
  }
  ~ImGuiSession() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
  }
  ImGuiSession(const ImGuiSession &) = delete;
  auto operator=(const ImGuiSession &) -> ImGuiSession & = delete;
};

void HandleKey(ViewerState &state, Key key) {
  switch (key) {
  case Key::A:
    state.playback.playing = !state.playback.playing;
    break;
  case Key::Space:
    state.playback.playing = false;
    state.playback.stepOnce = true;
    break;
  case Key::G:
    state.showGrid = !state.showGrid;
    break;
  case Key::Up:
    state.pull.y() += kPull;
    break;
  case Key::Down:
    state.pull.y() -= kPull;
    break;
  case Key::Right:
    state.pull.x() += kPull;
    break;
  case Key::Left:
    state.pull.x() -= kPull;
    break;
  }
}

void HandleEvent(ViewerState &state, Drag &drag, const Event &event) {
  const auto &io = ImGui::GetIO();
  auto &camera = state.camera;
  std::visit(
      [&](const auto &e) {
        using T = std::decay_t<decltype(e)>;
        if constexpr (std::is_same_v<T, KeyPressed>) {
          if (!io.WantCaptureKeyboard) {
            HandleKey(state, e.key);
          }
        } else if constexpr (std::is_same_v<T, MouseButtonChanged>) {
          if (!e.down) {
            drag = Drag::None;
          } else if (!io.WantCaptureMouse) {
            switch (e.button) {
            case MouseButton::Left:
              drag = e.shift ? Drag::Pan : Drag::Rotate;
              break;
            case MouseButton::Right:
              drag = Drag::Zoom;
              break;
            case MouseButton::Middle:
              drag = Drag::Pan;
              break;
            }
          }
        } else if constexpr (std::is_same_v<T, MouseMoved>) {
          const auto du = static_cast<float>(-e.dx);
          const auto dv = static_cast<float>(-e.dy);
          switch (drag) {
          case Drag::None:
            break;
          case Drag::Rotate:
            camera.Rotate(du, dv);
            break;
          case Drag::Zoom:
            camera.Zoom(dv * 0.1f);
            break;
          case Drag::Pan:
            camera.Pan(du, dv);
            break;
          }
        } else if constexpr (std::is_same_v<T, Scrolled>) {
          if (!io.WantCaptureMouse) {
            camera.Zoom(static_cast<float>(-e.dy) * 0.5f);
          }
        } else if constexpr (std::is_same_v<T, FramebufferResized>) {
          if (e.width > 0 && e.height > 0) {
            state.framebufferWidth = e.width;
            state.framebufferHeight = e.height;
            camera.Resize(e.width, e.height);
          }
        }
      },
      event);
}

} // namespace

auto ParseOptions(int argc, char **argv) -> std::expected<Options, std::string> {
  Options options;
  for (int ii = 1; ii < argc; ++ii) {
    const std::string_view arg = argv[ii];
    const bool hasValue = ii + 1 < argc;
    if (arg == "--scene" && hasValue) {
      options.scene = argv[++ii];
    } else if (arg == "--export" && hasValue) {
      options.exportDirectory = argv[++ii];
    } else {
      return std::unexpected(std::format("unexpected argument '{}'\n{}", arg, kUsage));
    }
  }
  return options;
}

auto Run(const Options &options) -> int {
  ViewerState state;
  state.catalog = sim::Catalog();
  if (state.catalog.empty()) {
    std::cerr << "the scene catalog is empty\n";
    return 1;
  }
  std::size_t scene = 0;
  if (options.scene) {
    const auto found = FindScene(state.catalog, *options.scene);
    if (!found) {
      std::cerr << "unknown scene '" << *options.scene << "'. Scenes:\n";
      for (const auto &entry : state.catalog) {
        std::cerr << "  " << entry.name << '\n';
      }
      return 2;
    }
    scene = *found;
  }
  if (options.exportDirectory) {
    state.exporter.directory = *options.exportDirectory;
    state.exporter.everyFrame = true;
  }

  Window window("sim-base", 1280, 800);
  std::tie(state.framebufferWidth, state.framebufferHeight) =
      window.FramebufferSize();
  Renderer renderer;
  ImGuiSession imgui(window);

  LoadScene(state, scene);
  Drag drag = Drag::None;
  FrameGeometry geometry;
  while (!window.ShouldClose()) {
    for (const auto &event : window.PollEvents()) {
      HandleEvent(state, drag, event);
    }

    if (AdvanceWorld(state) > 0 && state.exporter.everyFrame &&
        !state.exporter.directory.empty()) {
      ExportCurrentFrame(state);
    }

    geometry.Clear();
    if (state.showGrid) {
      AppendGridAndAxes(geometry);
    }
    if (state.world) {
      AppendWorld(geometry, *state.world);
    }
    renderer.Draw(geometry, state.camera.GetViewMatrix(),
                  state.camera.GetProjectionMatrix(), state.framebufferWidth,
                  state.framebufferHeight);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ui::DrawPanels(state);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    window.SwapBuffers();
  }
  return 0;
}

} // namespace viewer
