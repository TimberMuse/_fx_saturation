// SPDX-License-Identifier: MIT
#pragma once

#include <JuceHeader.h>
#include <array>

class FxSaturationAudioProcessor : public juce::AudioProcessor
{
public:
    FxSaturationAudioProcessor();
    ~FxSaturationAudioProcessor() override = default;

    // Basic AudioProcessor overrides
    const juce::String getName() const override { return "_fx_saturation"; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    // Editor (use generic for MVP)
    bool hasEditor() const override { return true; }
    juce::AudioProcessorEditor* createEditor() override;

    // State
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    // Parameters
    using APVTS = juce::AudioProcessorValueTreeState;
    static APVTS::ParameterLayout createParameterLayout();
    APVTS parameters { *this, nullptr, "PARAMETERS", createParameterLayout() };

    // ===== Scope (oscilloscope) bridge API =====
    // Read up to maxSamples from the internal scope FIFO into dest. Returns number of samples read.
    int readScope (float* dest, int maxSamples) noexcept;
    // Reset scope buffers (called on prepareToPlay)
    void resetScope() noexcept;
    // Expose sample rate for UI helpers
    double getSampleRateHz() const noexcept { return sampleRate; }

private:
    // Cached parameter pointers
    std::atomic<float>* driveDbParam  = nullptr; // dB
    std::atomic<float>* outputDbParam = nullptr; // dB
    std::atomic<float>* mixParam      = nullptr; // 0..1
    std::atomic<float>* toneParam     = nullptr; // -1..1 (tilt)
    std::atomic<float>* biasParam     = nullptr; // -1..1

    // Simple utility conversions
    static inline float dbToGain (float db) { return std::pow (10.0f, db * 0.05f); }

    // One-pole lowpass state per channel for tone control
    double sampleRate = 44100.0;
    struct ToneState { float lp = 0.0f; };
    std::vector<ToneState> toneStates;

    // Simple per-sample saturation (tanh)
    static inline float saturateTanh (float x) { return std::tanh (x); }

    // ===== Scope (oscilloscope) data transport =====
    static constexpr int kScopeCapacity = 32768; // samples in the ring buffer
    juce::AbstractFifo scopeFifo { kScopeCapacity };
    std::array<float, kScopeCapacity> scopeRing {};

    // Simple saturation activity meter for UI (0..1)
    std::atomic<float> satLevel { 0.0f };
public:
    float getSaturationLevel() const noexcept { return satLevel.load(); }
};

// Factory function required by JUCE plugin client
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter();
