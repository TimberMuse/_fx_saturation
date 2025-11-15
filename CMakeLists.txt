cmake_minimum_required(VERSION 3.22)
project(_fx_saturation VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build for Apple Silicon by default (you can add "x86_64;arm64" for universal)
if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "Build architectures" FORCE)
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
