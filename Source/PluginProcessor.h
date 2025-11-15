// SPDX-License-Identifier: MIT
#pragma once

#include <JuceHeader.h>

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

private:
    std::atomic<float>* driveParam = nullptr; // cached pointer for speed

    // Simple per-sample saturation
    static inline float saturateSample (float x, float drive)
    {
        const float minGain = 1.0f;
        const float maxGain = 20.0f;
        const float gain = minGain + drive * (maxGain - minGain);

        const float y = std::tanh (gain * x);
        const float normalize = 1.0f / std::tanh (maxGain);
        return y * normalize;
    }
};

// Factory function required by JUCE plugin client
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter();
