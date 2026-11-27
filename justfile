set positional-arguments

build_dir := "build"

# Configure (first run only) and build every target in Release.
build:
    test -f {{build_dir}}/CMakeCache.txt || cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE=Release
    cmake --build {{build_dir}} -j

# Open the viewer on the bunny-drop scene. Extra flags pass through, e.g. `just demo --export out`.
demo *flags: build
    ./{{build_dir}}/viewer/sim_viewer --scene bunny-drop "$@"
