#pragma once

#include <ViewerState.h>

#include <span>

namespace viewer::ui {

// A panel draws its widgets inside the viewer's sidebar and edits the shared
// state directly. Adding or removing one is a line in Panels().
struct Panel {
  const char *title;
  void (*draw)(ViewerState &state);
};

void ScenePicker(ViewerState &state);
void Playback(ViewerState &state);
void Stats(ViewerState &state);
void Lighting(ViewerState &state);
void Export(ViewerState &state);

auto Panels() -> std::span<const Panel>;

// Draws every registered panel as a collapsing section of one window.
void DrawPanels(ViewerState &state);

} // namespace viewer::ui
