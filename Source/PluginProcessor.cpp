// SPDX-License-Identifier: MIT
#include "PluginProcessor.h"
#include "PluginEditor.h"
#include <cstring>

//==============================================================================
FxSaturationAudioProcessor::FxSaturationAudioProcessor()
    : juce::AudioProcessor (BusesProperties()
                                .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                                .withOutput ("Output", juce::AudioChannelSet::stereo(), true))
{
    inputDbParam  = parameters.getRawParameterValue ("inputDb");
    driveDbParam  = parameters.getRawParameterValue ("driveDb");
    outputDbParam = parameters.getRawParameterValue ("outputDb");
    mixParam      = parameters.getRawParameterValue ("mix");
    toneParam     = parameters.getRawParameterValue ("tone");
    biasParam     = parameters.getRawParameterValue ("bias");
    modeParam     = parameters.getRawParameterValue ("mode");
    scopeZoomParam= parameters.getRawParameterValue ("scopeZoom");
    osModeParam   = parameters.getRawParameterValue ("osMode");
}

//==============================================================================
void FxSaturationAudioProcessor::prepareToPlay (double newSampleRate, int /*samplesPerBlock*/)
{
    sampleRate = newSampleRate > 0.0 ? newSampleRate : 44100.0;

    // Initialize smoothing
    const double smoothingTime = 0.02; // 20ms
    inputGainSmoothed.reset (sampleRate, smoothingTime);
    driveGainSmoothed.reset (sampleRate, smoothingTime);
    outputGainSmoothed.reset (sampleRate, smoothingTime);
    mixSmoothed.reset (sampleRate, smoothingTime);
    toneSmoothed.reset (sampleRate, smoothingTime);
    biasSmoothed.reset (sampleRate, smoothingTime);

    // Set initial values
    if (inputDbParam != nullptr)  inputGainSmoothed.setCurrentAndTargetValue (dbToGain (inputDbParam->load()));
    if (driveDbParam != nullptr)  driveGainSmoothed.setCurrentAndTargetValue (dbToGain (driveDbParam->load()));
    if (outputDbParam != nullptr) outputGainSmoothed.setCurrentAndTargetValue (dbToGain (outputDbParam->load()));
    if (mixParam != nullptr)      mixSmoothed.setCurrentAndTargetValue (mixParam->load());
    if (toneParam != nullptr)     toneSmoothed.setCurrentAndTargetValue (toneParam->load());
    if (biasParam != nullptr)     biasSmoothed.setCurrentAndTargetValue (biasParam->load());

    // Prepare tone filter states for max possible channels (updated in processBlock)
    const int maxCh = std::max (getTotalNumInputChannels(), getTotalNumOutputChannels());
    toneProcessors.resize ((size_t) std::max (1, maxCh));
    juce::dsp::ProcessSpec spec { sampleRate, (juce::uint32) 512, (juce::uint32) maxCh };
    for (auto& tp : toneProcessors) tp.prepare (spec);

    resetScope();

    // Setup/recreate oversampling based on current parameter
    int mode = 0;
    if (osModeParam != nullptr) mode = (int) std::round (osModeParam->load());
    
    osFactor = 1;
    int stages = 0;
    if (mode == 1) { osFactor = 2; stages = 1; }
    else if (mode == 2) { osFactor = 4; stages = 2; }
    else if (mode == 3) { osFactor = 8; stages = 3; }

    if (osFactor == 1) {
        oversampling.reset();
        setLatencySamples (0);
    } else {
        const int numCh = std::max (1, getTotalNumOutputChannels());
        oversampling = std::make_unique<juce::dsp::Oversampling<float>>(numCh, stages,
            juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, /*isMinPhase*/ false);
        oversampling->reset();
        oversampling->initProcessing ((size_t) 512);
        setLatencySamples ((int) oversampling->getLatencyInSamples());
    }
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

    // Update targets for smoothing
    if (inputDbParam != nullptr)  inputGainSmoothed.setTargetValue (dbToGain (inputDbParam->load()));
    if (driveDbParam != nullptr)  driveGainSmoothed.setTargetValue (dbToGain (driveDbParam->load()));
    if (outputDbParam != nullptr) outputGainSmoothed.setTargetValue (dbToGain (outputDbParam->load()));
    if (mixParam != nullptr)      mixSmoothed.setTargetValue (juce::jlimit (0.0f, 1.0f, mixParam->load()));
    if (toneParam != nullptr)     toneSmoothed.setTargetValue (juce::jlimit (-1.0f, 1.0f, toneParam->load()));
    if (biasParam != nullptr)     biasSmoothed.setTargetValue (juce::jlimit (-1.0f, 1.0f, biasParam->load()));

    if ((int) toneProcessors.size() < numChannels)
        toneProcessors.resize ((size_t) numChannels);

    // Update Tone filters based on smoothed value
    const float toneFreq = 2000.0f;
    for (int ch = 0; ch < (int) toneProcessors.size(); ++ch)
    {
        const float t = toneSmoothed.getCurrentValue();
        if (t >= 0.0f) {
            // High shelf boost
            *toneProcessors[(size_t)ch].filter.coefficients = *juce::dsp::IIR::Coefficients<float>::makeHighShelf (sampleRate * osFactor, toneFreq, 0.707f, dbToGain (t * 6.0f));
        } else {
            // Low shelf boost (effectively high cut relative to low)
            *toneProcessors[(size_t)ch].filter.coefficients = *juce::dsp::IIR::Coefficients<float>::makeLowShelf (sampleRate * osFactor, toneFreq, 0.707f, dbToGain (-t * 6.0f));
        }
    }

    float satPeak = 0.0f;
    float inPk = 0.0f, outPk = 0.0f;

    // Save dry (post-input gain) for mix
    dryBuffer.setSize (numChannels, numSamples, false, false, true);

    // Process input gain and meter (pre-OS)
    for (int ch = 0; ch < numChannels; ++ch)
    {
        const float* src = buffer.getReadPointer (ch);
        float* dry = dryBuffer.getWritePointer (ch);
        
        // We use a local copy of the smoother to process this channel's dry buffer
        auto inputSmoother = inputGainSmoothed;
        for (int i = 0; i < numSamples; ++i) {
            const float v = inputSmoother.getNextValue() * src[i];
            dry[i] = v;
            inPk = juce::jmax (inPk, std::abs (v));
        }
    }

    // Process path (possibly oversampled)
    if (osFactor > 1 && oversampling != nullptr)
    {
        // Up -> process -> Down
        juce::dsp::AudioBlock<float> inBlock (dryBuffer);
        auto upBlock = oversampling->processSamplesUp (inBlock);

        // Per-sample shape at osFactor, process in-place on upBlock
        const int osNumSamples = (int) upBlock.getNumSamples();
        const int osNumCh = (int) upBlock.getNumChannels();
        
        for (int ch = 0; ch < osNumCh; ++ch)
        {
            float* s = upBlock.getChannelPointer ((size_t) ch);
            auto& tp = toneProcessors[(size_t) (ch % (int) toneProcessors.size())];
            
            auto toneOS = toneSmoothed;
            auto biasOS = biasSmoothed;
            auto driveOS = driveGainSmoothed;

            for (int i = 0; i < osNumSamples; ++i)
            {
                // Advance smoothers only on even samples to keep timing consistent with base rate
                if (i % osFactor == 0) {
                    toneOS.getNextValue();
                    biasOS.getNextValue();
                    driveOS.getNextValue();
                }
                const float currentBias = biasOS.getCurrentValue();
                const float currentDrive = driveOS.getCurrentValue();

                const float dryVal = s[i];
                const float pre = tp.filter.processSample (dryVal);
                const float x = pre + 0.2f * currentBias;
                const float preDrive = x;
                const float d = currentDrive * preDrive;
                int mode = 0;
                if (modeParam != nullptr)
                    mode = juce::jlimit (0, 5, (int) std::round (modeParam->load()));
                float ySat = 0.0f;
                switch (mode)
                {
                    case 1: ySat = saturateHardClip (d); break;
                    case 2: ySat = saturateSoftCubic (d); break;
                    case 3: ySat = saturateArctan (d); break;
                    case 4: ySat = saturateEvenFullRect (d); break;
                    case 5: ySat = saturateOddCubic (d); break;
                    case 0:
                    default: ySat = saturateTanh (d); break;
                }
                s[i] = ySat; // store shaped for downsample
                if (ch == 0)
                {
                    const float delta = std::abs (ySat - preDrive);
                    if (delta > satPeak) satPeak = delta;
                }
            }
        }

        // Downsample from internal upsampled buffer into the output buffer
        juce::dsp::AudioBlock<float> outBlock (buffer);
        oversampling->processSamplesDown (outBlock);

        // Mix with dry and apply output gain and safety
        for (int ch = 0; ch < numChannels; ++ch)
        {
            float* dst = buffer.getWritePointer (ch);
            const float* dry = dryBuffer.getReadPointer (ch);
            
            auto mixSm = mixSmoothed;
            auto outSm = outputGainSmoothed;

            for (int i = 0; i < numSamples; ++i)
            {
                const float m = mixSm.getNextValue();
                float y = m * dst[i] + (1.0f - m) * dry[i];
                y *= outSm.getNextValue();
                const float yClipped = juce::jlimit (-1.0f, 1.0f, y);
                dst[i] = yClipped;
                if (ch == 0)
                {
                    int start1, size1, start2, size2;
                    scopeFifo.prepareToWrite (1, start1, size1, start2, size2);
                    int written = 0;
                    if (size1 > 0) { scopeRing[(size_t) start1] = yClipped; ++written; }
                    else if (size2 > 0) { scopeRing[(size_t) start2] = yClipped; ++written; }
                    if (written > 0)
                        scopeFifo.finishedWrite (written);
                }
                outPk = juce::jmax (outPk, std::abs (yClipped));
            }
        }
    }
    else
    {
        // Process at base rate, starting from dryBuffer (post input gain)
        for (int ch = 0; ch < numChannels; ++ch)
        {
            float* samples = buffer.getWritePointer (ch);
            const float* dryPostIn = dryBuffer.getReadPointer (ch);
            auto& tp = toneProcessors[(size_t) ch];

            auto biasSm  = biasSmoothed;
            auto driveSm = driveGainSmoothed;
            auto mixSm   = mixSmoothed;
            auto outSm   = outputGainSmoothed;

            for (int i = 0; i < numSamples; ++i)
            {
                const float dryVal = dryPostIn[i];
                const float pre = tp.filter.processSample (dryVal);
                // Add bias to create asymmetry (subtle range)
                const float x = pre + 0.2f * biasSm.getNextValue();
                // Waveshape
                const float preDrive = x;
                const float d = driveSm.getNextValue() * preDrive;
                int mode = 0;
                if (modeParam != nullptr)
                    mode = juce::jlimit (0, 5, (int) std::round (modeParam->load()));
                float ySat = 0.0f;
                switch (mode)
                {
                    case 1: ySat = saturateHardClip (d); break;
                    case 2: ySat = saturateSoftCubic (d); break;
                    case 3: ySat = saturateArctan (d); break;
                    case 4: ySat = saturateEvenFullRect (d); break;
                    case 5: ySat = saturateOddCubic (d); break;
                    case 0:
                    default: ySat = saturateTanh (d); break;
                }
                const float m = mixSm.getNextValue();
                float y = m * ySat + (1.0f - m) * dryVal;
                y *= outSm.getNextValue();
                const float yClipped = juce::jlimit (-1.0f, 1.0f, y);
                samples[i] = yClipped;
                if (ch == 0)
                {
                    int start1, size1, start2, size2;
                    scopeFifo.prepareToWrite (1, start1, size1, start2, size2);
                    int written = 0;
                    if (size1 > 0) { scopeRing[(size_t) start1] = yClipped; ++written; }
                    else if (size2 > 0) { scopeRing[(size_t) start2] = yClipped; ++written; }
                    if (written > 0)
                        scopeFifo.finishedWrite (written);
                    const float delta = std::abs (ySat - preDrive);
                    if (delta > satPeak) satPeak = delta;
                }
                outPk = juce::jmax (outPk, std::abs (yClipped));
            }
        }
    }

    // Advance the main smoothers by the number of samples processed
    inputGainSmoothed.skip (numSamples);
    driveGainSmoothed.skip (numSamples);
    outputGainSmoothed.skip (numSamples);
    mixSmoothed.skip (numSamples);
    toneSmoothed.skip (numSamples);
    biasSmoothed.skip (numSamples);

    // Smooth and publish saturation level for UI (0..1)
    float old = satLevel.load (std::memory_order_relaxed);
    float newLvl = juce::jlimit (0.0f, 1.0f, satPeak);
    satLevel.store (0.85f * old + 0.15f * newLvl, std::memory_order_relaxed);

    float oldIn = inPeak.load (std::memory_order_relaxed);
    float oldOut = outPeak.load (std::memory_order_relaxed);
    inPeak.store (0.8f * oldIn + 0.2f * juce::jlimit (0.0f, 1.0f, inPk), std::memory_order_relaxed);
    outPeak.store (0.8f * oldOut + 0.2f * juce::jlimit (0.0f, 1.0f, outPk), std::memory_order_relaxed);
}

//==============================================================================
juce::AudioProcessorEditor* FxSaturationAudioProcessor::createEditor()
{
    // Use custom editor for a better UX
    return new FxSaturationAudioProcessorEditor (*this);
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

    // Input trim in dB (-24..+24 dB)
    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"inputDb", 1},
        "Input",
        juce::NormalisableRange<float>{-24.0f, 24.0f, 0.01f, 0.5f},
        0.0f));

    // Drive in dB (0..36 dB)
    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"driveDb", 1},
        "Drive",
        juce::NormalisableRange<float>{0.0f, 36.0f, 0.01f, 0.5f},
        12.0f));

    // Output trim in dB (-24..+24 dB)
    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"outputDb", 1},
        "Output",
        juce::NormalisableRange<float>{-24.0f, 24.0f, 0.01f, 0.5f},
        -6.0f));

    // Mix 0..1
    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"mix", 1},
        "Mix",
        juce::NormalisableRange<float>{0.0f, 1.0f, 0.0f, 1.0f},
        1.0f));

    // Tone -1..1
    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"tone", 1},
        "Tone",
        juce::NormalisableRange<float>{-1.0f, 1.0f, 0.0f, 1.0f},
        0.0f));

    // Bias -1..1
    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"bias", 1},
        "Bias",
        juce::NormalisableRange<float>{-1.0f, 1.0f, 0.0f, 1.0f},
        0.0f));

    // Saturation algorithm mode
    params.push_back (std::make_unique<juce::AudioParameterChoice>(
        juce::ParameterID{"mode", 1},
        "Mode",
        juce::StringArray { "Tanh", "Hard Clip", "Soft Cubic", "Arctan", "Even (Rect)", "Odd Cubic" },
        0));

    // Scope zoom (UI hint/automation). 0..1
    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"scopeZoom", 1},
        "Scope Zoom",
        juce::NormalisableRange<float>{0.0f, 1.0f, 0.0f, 1.0f},
        0.5f));

    // Oversampling mode
    params.push_back (std::make_unique<juce::AudioParameterChoice>(
        juce::ParameterID{"osMode", 1},
        "Oversampling",
        juce::StringArray{ "Off", "2x", "4x", "8x" },
        0));

    return { params.begin(), params.end() };
}

// Mandatory factory function for JUCE plugins
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new FxSaturationAudioProcessor();
}

// ===== Scope bridge methods =====
int FxSaturationAudioProcessor::readScope (float* dest, int maxSamples) noexcept
{
    if (dest == nullptr || maxSamples <= 0) return 0;

    int totalRead = 0;
    while (totalRead < maxSamples)
    {
        int start1, size1, start2, size2;
        scopeFifo.prepareToRead (maxSamples - totalRead, start1, size1, start2, size2);
        if (size1 == 0 && size2 == 0)
            break;

        if (size1 > 0)
        {
            std::memcpy (dest + totalRead, scopeRing.data() + start1, (size_t) size1 * sizeof (float));
            totalRead += size1;
        }
        if (size2 > 0)
        {
            std::memcpy (dest + totalRead, scopeRing.data() + start2, (size_t) size2 * sizeof (float));
            totalRead += size2;
        }
        scopeFifo.finishedRead (size1 + size2);
    }
    return totalRead;
}

void FxSaturationAudioProcessor::resetScope() noexcept
{
    scopeFifo.reset();
    std::fill (scopeRing.begin(), scopeRing.end(), 0.0f);
}
