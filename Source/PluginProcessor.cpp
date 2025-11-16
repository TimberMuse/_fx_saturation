// SPDX-License-Identifier: MIT
#include "PluginProcessor.h"
#include <cstring>

namespace {
class BasicEditor : public juce::AudioProcessorEditor, private juce::Timer {
public:
    explicit BasicEditor (FxSaturationAudioProcessor& p)
        : juce::AudioProcessorEditor (&p), proc (p)
    {
        using Slider = juce::Slider;

        auto setupKnob = [this] (juce::Slider& s, const juce::String& name) {
            s.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
            s.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 64, 18);
            s.setName (name);
            addAndMakeVisible (s);
        };

        setupKnob (drive,  "Drive");
        setupKnob (output, "Output");
        setupKnob (mix,    "Mix");
        setupKnob (tone,   "Tone");
        setupKnob (bias,   "Bias");

        // Value readouts improvements
        drive.setTextValueSuffix (" dB");
        output.setTextValueSuffix (" dB");
        genFreq.setTextValueSuffix (" Hz");

        mix.textFromValueFunction = [] (double v) { return juce::String (juce::roundToInt (v * 100.0)) + " %"; };
        mix.valueFromTextFunction = [] (const juce::String& t) {
            auto s = t.upToFirstOccurrenceOf ("%", false, false).trim();
            return juce::jlimit (0.0, 1.0, s.getDoubleValue() / 100.0);
        };
        genAmp.textFromValueFunction = [] (double v) { return juce::String (juce::roundToInt (v * 100.0)) + " %"; };
        genAmp.valueFromTextFunction = [] (const juce::String& t) {
            auto s = t.upToFirstOccurrenceOf ("%", false, false).trim();
            return juce::jlimit (0.0, 1.0, s.getDoubleValue() / 100.0);
        };

        drive.setSkewFactor (0.6); // more resolution at lower drive

        driveAttachment  = std::make_unique<Attachment> (proc.parameters, "driveDb",  drive);
        outputAttachment = std::make_unique<Attachment> (proc.parameters, "outputDb", output);
        mixAttachment    = std::make_unique<Attachment> (proc.parameters, "mix",     mix);
        toneAttachment   = std::make_unique<Attachment> (proc.parameters, "tone",    tone);
        biasAttachment   = std::make_unique<Attachment> (proc.parameters, "bias",    bias);

        // Oscilloscope setup
        scope.setNumChannels (1);
        scope.setBufferSize (2048);
        scope.setSamplesPerBlock (256);
        scope.setColours (juce::Colours::black, juce::Colours::lime.withAlpha (0.9f));
        addAndMakeVisible (scope);

        // Generator controls (UI-only; feeds the scope)
        genEnable.setButtonText ("Gen");
        genEnable.setTooltip ("Enable internal signal generator feeding the scope");
        addAndMakeVisible (genEnable);

        genFreq.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
        genFreq.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 64, 18);
        genFreq.setRange (20.0, 5000.0, 0.01);
        genFreq.setSkewFactorFromMidPoint (500.0);
        genFreq.setValue (1000.0);
        genFreq.setName ("Freq");
        addAndMakeVisible (genFreq);

        genAmp.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
        genAmp.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 64, 18);
        genAmp.setRange (0.0, 1.0, 0.0001);
        genAmp.setValue (0.5);
        genAmp.setName ("Amp");
        addAndMakeVisible (genAmp);

        // Labels for controls
        auto makeLabel = [this] (juce::Label& l, const juce::String& text)
        {
            l.setText (text, juce::dontSendNotification);
            l.setJustificationType (juce::Justification::centred);
            l.setColour (juce::Label::textColourId, juce::Colours::white);
            l.setInterceptsMouseClicks (false, false);
            addAndMakeVisible (l);
        };
        makeLabel (driveLabel,  "Drive");
        makeLabel (outputLabel, "Output");
        makeLabel (mixLabel,    "Mix");
        makeLabel (toneLabel,   "Tone");
        makeLabel (biasLabel,   "Bias");
        makeLabel (freqLabel,   "Freq");
        makeLabel (ampLabel,    "Amp");

        // Simple saturation LED
        addAndMakeVisible (satLed);

        setResizable (true, true);
        setSize (640, 380);

        startTimerHz (60);
    }

    void paint (juce::Graphics& g) override
    {
        g.fillAll (juce::Colours::black);
        auto area = getLocalBounds();
        ColourTitleBar title (area.removeFromTop (28), "_fx_saturation", juce::Colours::orange);
        title.paint (g);
    }

    void resized() override
    {
        auto r = getLocalBounds().reduced (12);
        r.removeFromTop (32); // title spacing

        // Top: scope
        auto scopeArea = r.removeFromTop (juce::jmax (80, r.getHeight() / 2)).reduced (4);
        scope.setBounds (scopeArea);

        // Bottom: controls
        auto row = r.removeFromTop (r.getHeight());
        auto w = row.getWidth() / 9; // 5 knobs + Gen toggle + Freq + Amp + LED
        auto makeBox = [&] () { return row.removeFromLeft (w).reduced (6); };
        auto bDrive  = makeBox();
        auto bOutput = makeBox();
        auto bMix    = makeBox();
        auto bTone   = makeBox();
        auto bBias   = makeBox();
        auto bGenTgl = makeBox();
        auto bFreq   = makeBox();
        auto bAmp    = makeBox();
        auto bLed    = row.reduced (6);

        drive.setBounds  (bDrive);
        output.setBounds (bOutput);
        mix.setBounds    (bMix);
        tone.setBounds   (bTone);
        bias.setBounds   (bBias);
        genEnable.setBounds (bGenTgl);
        genFreq.setBounds   (bFreq);
        genAmp.setBounds    (bAmp);
        satLed.setBounds    (bLed);

        const int labelHeight = 16;
        auto placeLabelAbove = [labelHeight] (juce::Label& l, juce::Rectangle<int> target) {
            l.setBounds (target.withHeight (labelHeight).translated (0, -labelHeight - 2));
        };
        placeLabelAbove (driveLabel,  bDrive);
        placeLabelAbove (outputLabel, bOutput);
        placeLabelAbove (mixLabel,    bMix);
        placeLabelAbove (toneLabel,   bTone);
        placeLabelAbove (biasLabel,   bBias);
        placeLabelAbove (freqLabel,   bFreq);
        placeLabelAbove (ampLabel,    bAmp);
    }

private:
    void timerCallback() override
    {
        const int visualBlock = 256;

        if (genEnable.getToggleState())
        {
            // Generate sine for the scope
            ensureBufferSize (genTemp, visualBlock);
            const double sr = juce::jmax (1.0, proc.getSampleRateHz());
            const double freq = genFreq.getValue();
            const float amp = (float) genAmp.getValue();
            const double inc = juce::MathConstants<double>::twoPi * freq / sr;
            for (int i = 0; i < visualBlock; ++i)
            {
                genPhase += inc;
                if (genPhase > juce::MathConstants<double>::twoPi)
                    genPhase -= juce::MathConstants<double>::twoPi;
                genTemp[i] = amp * (float) std::sin (genPhase);
            }
            const float* chans1[] = { genTemp.data() };
            scope.pushBuffer (chans1, 1, visualBlock);
        }
        else
        {
            // Drain processor scope FIFO
            ensureBufferSize (scopeTemp, visualBlock);
            const int n = proc.readScope (scopeTemp.data(), visualBlock);
            if (n > 0) {
                const float* chans2[] = { scopeTemp.data() };
                scope.pushBuffer (chans2, 1, n);
            }
        }

        // Update saturation LED from processor
        satLed.setLevel ((float) proc.getSaturationLevel());
    }

    static void ensureBufferSize (std::vector<float>& v, int n)
    {
        if ((int) v.size() < n) v.resize ((size_t) n, 0.0f);
    }

    // A tiny helper for a coloured title bar
    class ColourTitleBar {
    public:
        ColourTitleBar (juce::Rectangle<int> b, juce::String t, juce::Colour c)
            : bounds (b), text (std::move (t)), colour (c) {}
        void paint (juce::Graphics& g) const {
            g.setColour (colour.withAlpha (0.14f));
            g.fillRect (bounds);
            g.setColour (colour);
            g.drawFittedText (text, bounds, juce::Justification::centredLeft, 1);
        }
    private:
        juce::Rectangle<int> bounds; juce::String text; juce::Colour colour;
    };

    // Tiny LED widget indicating recent saturation activity
    class SatLED : public juce::Component {
    public:
        void setLevel (float l) { level = juce::jlimit (0.0f, 1.0f, l); repaint(); }
        void paint (juce::Graphics& g) override {
            auto bounds = getLocalBounds();
            auto textArea = bounds.removeFromLeft (bounds.getWidth() / 2);
            g.setColour (juce::Colours::white.withAlpha (0.8f));
            g.drawFittedText ("SAT", textArea, juce::Justification::centred, 1);
            auto led = bounds.reduced (4);
            auto colour = juce::Colours::red.withAlpha (0.25f + 0.75f * level);
            if (level > 0.6f)
                colour = colour.interpolatedWith (juce::Colours::yellow, (level - 0.6f) / 0.4f);
            g.setColour (colour);
            g.fillEllipse (led.toFloat());
            g.setColour (juce::Colours::dimgrey);
            g.drawEllipse (led.toFloat(), 1.0f);
        }
    private:
        float level = 0.0f;
    };

    using Attachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    FxSaturationAudioProcessor& proc;
    juce::AudioVisualiserComponent scope { 1 };
    juce::Slider drive, output, mix, tone, bias;
    juce::ToggleButton genEnable;
    juce::Slider genFreq, genAmp;
    juce::Label driveLabel, outputLabel, mixLabel, toneLabel, biasLabel, freqLabel, ampLabel;
    SatLED satLed;
    std::vector<float> genTemp, scopeTemp;
    double genPhase = 0.0;
    std::unique_ptr<Attachment> driveAttachment, outputAttachment, mixAttachment, toneAttachment, biasAttachment;
};
} // namespace

//==============================================================================
FxSaturationAudioProcessor::FxSaturationAudioProcessor()
    : juce::AudioProcessor (BusesProperties()
                                .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                                .withOutput ("Output", juce::AudioChannelSet::stereo(), true))
{
    driveDbParam  = parameters.getRawParameterValue ("driveDb");
    outputDbParam = parameters.getRawParameterValue ("outputDb");
    mixParam      = parameters.getRawParameterValue ("mix");
    toneParam     = parameters.getRawParameterValue ("tone");
    biasParam     = parameters.getRawParameterValue ("bias");
}

//==============================================================================
void FxSaturationAudioProcessor::prepareToPlay (double newSampleRate, int /*samplesPerBlock*/)
{
    sampleRate = newSampleRate > 0.0 ? newSampleRate : 44100.0;
    // Prepare tone filter states for max possible channels (updated in processBlock)
    const int maxCh = std::max (getTotalNumInputChannels(), getTotalNumOutputChannels());
    toneStates.assign ((size_t) std::max (1, maxCh), {});
    resetScope();
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
    // Read parameters (with safe defaults)
    const float driveDb  = driveDbParam  != nullptr ? driveDbParam->load()  : 12.0f;
    const float outDb    = outputDbParam != nullptr ? outputDbParam->load() : 0.0f;
    const float mix      = mixParam      != nullptr ? juce::jlimit (0.0f, 1.0f, mixParam->load()) : 1.0f;
    const float tone     = toneParam     != nullptr ? juce::jlimit (-1.0f, 1.0f, toneParam->load()) : 0.0f;
    const float biasAmt  = biasParam     != nullptr ? juce::jlimit (-1.0f, 1.0f, biasParam->load()) : 0.0f;

    const float driveGain = dbToGain (driveDb);
    const float outGain   = dbToGain (outDb);

    if ((int) toneStates.size() < numChannels)
        toneStates.resize ((size_t) numChannels);

    // Tone LPF coefficient (fixed cutoff around 800 Hz)
    const float fc = 800.0f;
    const float a = std::exp (-2.0f * juce::MathConstants<float>::pi * fc / (float) sampleRate);
    const float b = 1.0f - a;

    float satPeak = 0.0f;
    for (int ch = 0; ch < numChannels; ++ch)
    {
        float* samples = buffer.getWritePointer (ch);
        auto& st = toneStates[(size_t) ch];
        for (int i = 0; i < numSamples; ++i)
        {
            const float dry = samples[i];

            // One-pole lowpass for tilt EQ
            st.lp = a * st.lp + b * dry;
            const float hp = dry - st.lp; // high-passed portion
            const float pre = dry + tone * hp; // tilt: boost/cut highs

            // Add bias to create asymmetry (subtle range)
            const float x = pre + 0.2f * biasAmt;

            // Waveshape
            const float preDrive = x;
            const float ySat = saturateTanh (driveGain * preDrive);

            // Mix and output trim
            float y = mix * ySat + (1.0f - mix) * dry;
            y *= outGain;

            // Safety clip
            const float yClipped = juce::jlimit (-1.0f, 1.0f, y);
            samples[i] = yClipped;

            // Write left channel (ch 0) to scope FIFO
            if (ch == 0)
            {
                int start1, size1, start2, size2;
                scopeFifo.prepareToWrite (1, start1, size1, start2, size2);
                int written = 0;
                if (size1 > 0) { scopeRing[(size_t) start1] = yClipped; ++written; }
                else if (size2 > 0) { scopeRing[(size_t) start2] = yClipped; ++written; }
                if (written > 0)
                    scopeFifo.finishedWrite (written);

                // Track saturation activity using difference between shaped and input
                const float delta = std::abs (ySat - preDrive);
                if (delta > satPeak) satPeak = delta;
            }
        }
    }

    // Smooth and publish saturation level for UI (0..1)
    float old = satLevel.load (std::memory_order_relaxed);
    float newLvl = juce::jlimit (0.0f, 1.0f, satPeak);
    satLevel.store (0.85f * old + 0.15f * newLvl, std::memory_order_relaxed);
}

//==============================================================================
juce::AudioProcessorEditor* FxSaturationAudioProcessor::createEditor()
{
    // Use custom editor for a better UX
    return new BasicEditor (*this);
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
