cmake_minimum_required(VERSION 3.22)
project(_fx_saturation VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point JUCE to the in-tree Steinberg VST3 SDK
# This ensures VST3 builds use the SDK bundled with this repository
set(JUCE_VST3_SDK_PATH "${CMAKE_SOURCE_DIR}/vst3sdk" CACHE PATH "Path to local VST3 SDK" FORCE)
# Also set the legacy variable some tools still read
set(VST3_SDK_DIR "${CMAKE_SOURCE_DIR}/vst3sdk" CACHE PATH "Path to local VST3 SDK (legacy var)" FORCE)

# Build universal binaries on macOS (Apple Silicon + Intel)
if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "x86_64;arm64" CACHE STRING "Build architectures" FORCE)
endif()

# Add JUCE (vendored in this repo)
add_subdirectory(JUCE)

# Tell JUCE where to copy built plugins (user-local install)
if(APPLE)
    # Use environment HOME to avoid issues with ~ expansion
    set(JUCE_AU_COPY_DIR    "$ENV{HOME}/Library/Audio/Plug-Ins/Components" CACHE PATH "AU copy dir" FORCE)
    set(JUCE_VST3_COPY_DIR  "$ENV{HOME}/Library/Audio/Plug-Ins/VST3" CACHE PATH "VST3 copy dir" FORCE)
endif()

# Define the plugin target
juce_add_plugin(_fx_saturation
    COMPANY_NAME            "SonicDesign"
    BUNDLE_ID               com.sonicdesign.fx.saturation
    IS_SYNTH                FALSE
    NEEDS_MIDI_INPUT        FALSE
    NEEDS_MIDI_OUTPUT       FALSE
    IS_MIDI_EFFECT          FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD TRUE
    PLUGIN_MANUFACTURER_CODE SDsg
    PLUGIN_CODE             FxSa
    FORMATS                 AU VST3
    PRODUCT_NAME            "_fx_saturation"
)

target_sources(_fx_saturation
    PRIVATE
        Source/PluginProcessor.cpp
        Source/PluginProcessor.h
)

target_compile_definitions(_fx_saturation
    PRIVATE
        JUCE_WEB_BROWSER=0
        JUCE_USE_CURL=0
        JUCE_VST3_CAN_REPLACE_VST2=0
)

target_link_libraries(_fx_saturation
    PRIVATE
        juce::juce_audio_utils
        juce::juce_audio_processors
        juce::juce_audio_basics
        juce::juce_dsp
)

# Generate JUCE header for PCH and resources
juce_generate_juce_header(_fx_saturation)
