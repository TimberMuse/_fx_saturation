// Dummy file to satisfy a JUCE helper CMake quirk that references
// a relative ../../../../main.cpp from within JUCE/extras/Build/CMake.
// This file is NOT part of the plugin target; it simply prevents
// configuration errors when bootstrapping juceaide and helper targets.
// SPDX-License-Identifier: MIT

int main() { return 0; }
