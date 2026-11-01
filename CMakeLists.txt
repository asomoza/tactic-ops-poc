cmake_minimum_required(VERSION 3.24)
project(TacticsPoc LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(MOTION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../motion" CACHE PATH "The motion module (MotionBricks on Wicked Engine)")
set(TACTICS_MODEL_DIR "$ENV{HOME}/Documents/models/motionbricks_soma_diffusers" CACHE PATH
    "Default MotionBricks SOMA diffusers weights")
set(MOTION_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory("${MOTION_DIR}" motion)

# The game logic shared by the game and the tools. lodepng inflates the compressed motion NPZ files.
add_library(tactics_core STATIC
    src/clip.cpp
    src/rig.cpp
    src/grid.cpp
    src/motion_player.cpp
    src/patrol.cpp
    "${WICKED_ENGINE_DIR}/WickedEngine/Utility/lodepng.cpp"
)
target_link_libraries(tactics_core PUBLIC motionbricks WickedEngine)
target_include_directories(tactics_core PUBLIC src PRIVATE "${WICKED_ENGINE_DIR}/WickedEngine")
target_compile_options(tactics_core PUBLIC ${MOTION_NO_RTTI})
target_compile_definitions(tactics_core PUBLIC
    TACTICS_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
    TACTICS_SOMA_DIR="${MOTION_DIR}/assets/soma"
    TACTICS_DEFAULT_MODEL_DIR="${TACTICS_MODEL_DIR}"
)

add_executable(tactics_poc WIN32
    src/main.cpp
    src/game.cpp
    src/character_view.cpp
    "${WICKED_ENGINE_DIR}/Editor/ModelImporter_GLTF.cpp"
)
target_link_libraries(tactics_poc PRIVATE tactics_core)
target_include_directories(tactics_poc PRIVATE "${WICKED_ENGINE_DIR}/Editor")
target_compile_definitions(tactics_poc PRIVATE
    TACTICS_BUILD_DIR="${CMAKE_CURRENT_BINARY_DIR}"
    TACTICS_WICKED_SHADER_SOURCE="${WICKED_ENGINE_DIR}/WickedEngine/shaders"
)
motion_copy_shader_compiler(tactics_poc)
if(MSVC)
    # A window without a console, entered through main().
    target_link_options(tactics_poc PRIVATE /ENTRY:mainCRTStartup)
endif()

# Headless tools: rig and planning checks, short moves at different transition lengths, and many moves on the game's
# map with the contacts with cover.
foreach(tool tactics_check short_moves map_moves crouch_moves patrol_check)
    add_executable(${tool} tools/${tool}.cpp)
    target_link_libraries(${tool} PRIVATE tactics_core)
    motion_copy_shader_compiler(${tool})
endforeach()

# The package: `cmake --install build --prefix <dir>` lays out a game that runs from <dir> (see src/main.cpp). The
# MotionBricks weights are copied from TACTICS_MODEL_DIR.
install(TARGETS tactics_poc RUNTIME DESTINATION .)
install(FILES "${MOTION_SHADER_COMPILER}" DESTINATION .)
# Only the game's own assets: assets/motions/soma holds NVIDIA's evaluation clips, which may not be shipped.
install(DIRECTORY assets/character assets/textures DESTINATION assets)
install(DIRECTORY assets/motions/kimodo DESTINATION assets/motions)
install(FILES "${MOTION_DIR}/assets/soma/soma_skeleton.txt" DESTINATION soma)
install(DIRECTORY "${MOTION_DIR}/shaders/" DESTINATION shaders/motion)
install(DIRECTORY "${WICKED_ENGINE_DIR}/WickedEngine/shaders/" DESTINATION shaders/wicked)
# The weights with their license and notice, which the NVIDIA Open Model Agreement requires to travel with them.
install(DIRECTORY "${TACTICS_MODEL_DIR}/" DESTINATION models/motionbricks_soma_diffusers
    FILES_MATCHING PATTERN "*.safetensors" PATTERN "*.json" PATTERN "LICENSE" PATTERN "NOTICE")
install(FILES LICENSES.md DESTINATION .)
install(FILES "${WICKED_ENGINE_DIR}/LICENSE.txt" DESTINATION licenses RENAME WickedEngine.txt)
