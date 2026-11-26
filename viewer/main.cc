#include <App.h>

#include <exception>
#include <iostream>

auto main(int argc, char **argv) -> int {
  const auto options = viewer::ParseOptions(argc, argv);
  if (!options) {
    std::cerr << options.error() << '\n';
    return 2;
  }
  try {
    return viewer::Run(*options);
  } catch (const std::exception &e) {
    std::cerr << "sim_viewer: " << e.what() << '\n';
    return 1;
  }
}
