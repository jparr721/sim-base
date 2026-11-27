project(Tests)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../ext/googletest" "ext/googletest")

mark_as_advanced(
        BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
        gmock_build_tests gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

set_target_properties(gtest gmock gtest_main gmock_main PROPERTIES SYSTEM TRUE)
foreach (target gtest gmock gtest_main gmock_main)
    target_compile_options(${target} PRIVATE -w)
endforeach ()
set_target_properties(gtest PROPERTIES FOLDER third_party)
set_target_properties(gtest_main PROPERTIES FOLDER third_party)
set_target_properties(gmock PROPERTIES FOLDER third_party)
set_target_properties(gmock_main PROPERTIES FOLDER third_party)

macro(package_add_test TESTNAME)
    # create an executable in which the tests will be stored
    add_executable(${TESTNAME} ${ARGN})
    # link the Google test infrastructure, mocking library, and a default main function to
    # the test executable.  Remove g_test_main if writing your own main function.
    target_link_libraries(${TESTNAME} sim gmock gtest_main)
    target_compile_options(${TESTNAME} PRIVATE ${SIM_WARNINGS})
    # gtest_discover_tests replaces gtest_add_tests,
    # see https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more options to pass to it
    gtest_discover_tests(${TESTNAME}
            # set a working directory so your project root so that you can find test data via paths relative to the project root
            WORKING_DIRECTORY ${PROJECT_DIR}
            PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}"
            )
    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()

package_add_test(TestTetMesh TestTetMesh.cc)
package_add_test(TestLibMath TestLibMath.cc)
package_add_test(TestStrandForces TestStrandForces.cc)
package_add_test(TestElementEnergies TestElementEnergies.cc)
# Only the energy module, so the test fails to link if the module needs sim.
set_property(TARGET TestElementEnergies PROPERTY LINK_LIBRARIES
        sim_energy_mass_spring sim_energy_vertex_face_collision gmock gtest_main)
package_add_test(TestEnergies TestEnergies.cc)
# Only the energy modules, so the test fails to link if a module needs sim.
set_property(TARGET TestEnergies PROPERTY LINK_LIBRARIES
        sim_volume_energies gmock gtest_main)
package_add_test(TestDamping TestDamping.cc)
# Only the damping module, so the test fails to link if the module needs more.
set_property(TARGET TestDamping PROPERTY LINK_LIBRARIES
        sim_damping_green gmock gtest_main)
package_add_test(TestElasticSystem TestElasticSystem.cc)
# Only sim_solid, so the test fails to link if assembly needs sim.
set_property(TARGET TestElasticSystem PROPERTY LINK_LIBRARIES sim_solid gmock gtest_main)
package_add_test(TestIntegrators TestIntegrators.cc)
# sim_solid for the bodies, sim_integrators for the methods, and never sim.
set_property(TARGET TestIntegrators PROPERTY LINK_LIBRARIES
        sim_integrators sim_solid gmock gtest_main)
package_add_test(TestSceneGolden TestSceneGolden.cc)
set_property(TARGET TestSceneGolden PROPERTY LINK_LIBRARIES sim_scenes gmock gtest_main)
package_add_test(TestWorld TestWorld.cc)
# sim_scenes brings sim; listing sim a second time makes ld64 warn.
set_property(TARGET TestWorld PROPERTY LINK_LIBRARIES sim_scenes gmock gtest_main)

package_add_test(TestAlbedoTexture TestAlbedoTexture.cc)
set_property(TARGET TestAlbedoTexture PROPERTY LINK_LIBRARIES sim_viewer_albedo gmock gtest_main)

add_executable(GenerateGolden Golden/GenerateGolden.cc)
target_link_libraries(GenerateGolden sim_scenes)
target_compile_options(GenerateGolden PRIVATE ${SIM_WARNINGS})

add_library(PublicHeadersCxx20 OBJECT PublicHeadersCxx20.cc)
target_link_libraries(PublicHeadersCxx20 PRIVATE sim)
set_target_properties(PublicHeadersCxx20 PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS ON)
target_compile_options(PublicHeadersCxx20 PRIVATE ${SIM_WARNINGS} -Werror)

# A game with its own global PI, E, ASSERT, VERSION and Real includes the core.
add_library(EmbedNoCollisions OBJECT EmbedNoCollisions.cc)
target_link_libraries(EmbedNoCollisions PRIVATE sim)
set_target_properties(EmbedNoCollisions PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS ON)
# -Werror: a macro redefinition is only a warning, and it must fail here.
target_compile_options(EmbedNoCollisions PRIVATE ${SIM_WARNINGS} -Werror)
