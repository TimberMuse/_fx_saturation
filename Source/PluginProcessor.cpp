// SPDX-License-Identifier: MIT
#include "PluginProcessor.h"

//==============================================================================
FxSaturationAudioProcessor::FxSaturationAudioProcessor()
    : juce::AudioProcessor (BusesProperties()
                                .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                                .withOutput ("Output", juce::AudioChannelSet::stereo(), true))
{
    driveParam = parameters.getRawParameterValue ("drive");
}

//==============================================================================
void FxSaturationAudioProcessor::prepareToPlay (double /*sampleRate*/, int /*samplesPerBlock*/)
{
}

void FxSaturationAudioProcessor::releaseResources()
{
}

bool FxSaturationAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    // Only support mono or stereo symmetric layouts
    const auto& mainOut = layouts.getMainOutputChannelSet();
    if (mainOut != juce::AudioChannelSet::mono() && mainOut != juce::AudioChannelSet::stereo())
        return false;

    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;

    return true;
}

void FxSaturationAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer,
                                               juce::MidiBuffer& /*midiMessages*/)
{
    juce::ScopedNoDenormals noDenormals;

    const int numChannels = buffer.getNumChannels();
    const int numSamples  = buffer.getNumSamples();
    const float drive = driveParam != nullptr ? driveParam->load() : 0.5f;

    for (int ch = 0; ch < numChannels; ++ch)
    {
        float* samples = buffer.getWritePointer (ch);
        for (int i = 0; i < numSamples; ++i)
            samples[i] = saturateSample (samples[i], drive);
    }
}

//==============================================================================
juce::AudioProcessorEditor* FxSaturationAudioProcessor::createEditor()
{
    return new juce::GenericAudioProcessorEditor (*this);
}

//==============================================================================
void FxSaturationAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto state = parameters.copyState(); state.isValid())
    {
        juce::MemoryOutputStream mos (destData, true);
        state.writeToStream (mos);
    }
}

void FxSaturationAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    juce::ValueTree tree = juce::ValueTree::readFromData (data, static_cast<size_t>(sizeInBytes));
    if (tree.isValid())
        parameters.replaceState (tree);
}

FxSaturationAudioProcessor::APVTS::ParameterLayout FxSaturationAudioProcessor::createParameterLayout()
{
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> params;
    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"drive", 1},
        "Drive",
        juce::NormalisableRange<float>{0.0f, 1.0f, 0.0f, 1.0f},
        0.5f));

    return { params.begin(), params.end() };
}

// Mandatory factory function for JUCE plugins
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new FxSaturationAudioProcessor();
}
