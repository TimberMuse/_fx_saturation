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

        setupKnob (input,  "Input");
        setupKnob (drive,  "Drive");
        setupKnob (output, "Output");
        setupKnob (mix,    "Mix");
        setupKnob (tone,   "Tone");
        setupKnob (bias,   "Bias");
        setupKnob (zoom,   "Zoom");

        // Value readouts improvements
        input.setTextValueSuffix (" dB");
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

        inputAttachment  = std::make_unique<Attachment> (proc.parameters, "inputDb",  input);
        driveAttachment  = std::make_unique<Attachment> (proc.parameters, "driveDb",  drive);
        outputAttachment = std::make_unique<Attachment> (proc.parameters, "outputDb", output);
        mixAttachment    = std::make_unique<Attachment> (proc.parameters, "mix",     mix);
        toneAttachment   = std::make_unique<Attachment> (proc.parameters, "tone",    tone);
        biasAttachment   = std::make_unique<Attachment> (proc.parameters, "bias",    bias);
        zoomAttachment   = std::make_unique<Attachment> (proc.parameters, "scopeZoom", zoom);

        // Mode selector
        modeBox.addItemList (juce::StringArray { "Tanh", "Hard Clip", "Soft Cubic", "Arctan", "Even (Rect)", "Odd Cubic" }, 1);
        addAndMakeVisible (modeBox);
        modeAttachment = std::make_unique<ComboAttachment> (proc.parameters, "mode", modeBox);

        // Oversampling selector
        osBox.addItemList (juce::StringArray { "Off", "2x" }, 1);
        addAndMakeVisible (osBox);
        osAttachment = std::make_unique<ComboAttachment> (proc.parameters, "osMode", osBox);

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
        makeLabel (inputLabel,  "Input");
        makeLabel (driveLabel,  "Drive");
        makeLabel (outputLabel, "Output");
        makeLabel (mixLabel,    "Mix");
        makeLabel (toneLabel,   "Tone");
        makeLabel (biasLabel,   "Bias");
        makeLabel (freqLabel,   "Freq");
        makeLabel (ampLabel,    "Amp");
        makeLabel (modeLabel,   "Mode");
        makeLabel (zoomLabel,   "Zoom");
        makeLabel (osLabel,     "OS");

        // Simple saturation LED
        addAndMakeVisible (satLed);

        // Simple meters
        addAndMakeVisible (inMeter);
        addAndMakeVisible (outMeter);

        setResizable (true, true);
        setSize (720, 420);

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
        auto w = row.getWidth() / 14; // 6 knobs + Gen + Freq + Amp + LED + Mode + Zoom + OS + 2 meters
        auto makeBox = [&] () { return row.removeFromLeft (w).reduced (6); };
        auto bInput  = makeBox();
        auto bDrive  = makeBox();
        auto bOutput = makeBox();
        auto bMix    = makeBox();
        auto bTone   = makeBox();
        auto bBias   = makeBox();
        auto bGenTgl = makeBox();
        auto bFreq   = makeBox();
        auto bAmp    = makeBox();
        auto bLed    = makeBox();
        auto bInMet  = makeBox();
        auto bOutMet = makeBox();
        auto bMode   = makeBox();
        auto bZoom   = makeBox();
        auto bOS     = row.reduced (6);

        input.setBounds  (bInput);
        drive.setBounds  (bDrive);
        output.setBounds (bOutput);
        mix.setBounds    (bMix);
        tone.setBounds   (bTone);
        bias.setBounds   (bBias);
        genEnable.setBounds (bGenTgl);
        genFreq.setBounds   (bFreq);
        genAmp.setBounds    (bAmp);
        satLed.setBounds    (bLed);
        modeBox.setBounds   (bMode);
        zoom.setBounds      (bZoom);
        osBox.setBounds     (bOS);
        inMeter.setBounds   (bInMet);
        outMeter.setBounds  (bOutMet);

        const int labelHeight = 16;
        auto placeLabelAbove = [labelHeight] (juce::Label& l, juce::Rectangle<int> target) {
            l.setBounds (target.withHeight (labelHeight).translated (0, -labelHeight - 2));
        };
        placeLabelAbove (inputLabel,  bInput);
        placeLabelAbove (driveLabel,  bDrive);
        placeLabelAbove (outputLabel, bOutput);
        placeLabelAbove (mixLabel,    bMix);
        placeLabelAbove (toneLabel,   bTone);
        placeLabelAbove (biasLabel,   bBias);
        placeLabelAbove (freqLabel,   bFreq);
        placeLabelAbove (ampLabel,    bAmp);
        placeLabelAbove (modeLabel,   bMode);
        placeLabelAbove (zoomLabel,   bZoom);
        placeLabelAbove (osLabel,     bOS);
    }

private:
    void timerCallback() override
    {
        const int visualBlock = 256;

        // Update scope resolution based on zoom slider
        {
            const double z = juce::jlimit (0.0, 1.0, (double) zoom.getValue());
            // Map 0..1 to samples-per-block from 512 (far) to 16 (close)
            const int spp = (int) juce::jlimit (16, 1024, (int) std::round (512.0 * std::pow (0.5, 3.0 * z)));
            if (spp != lastSpp)
            {
                lastSpp = spp;
                scope.setSamplesPerBlock (spp);
                // Keep a few seconds of history regardless of spp
                const int desiredHistorySamples = 4096;
                scope.setBufferSize (juce::jmax (desiredHistorySamples / spp, 64));
            }
        }

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
        inMeter.setLevel (proc.getInputPeak());
        outMeter.setLevel (proc.getOutputPeak());
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

    // Minimal vertical peak meter (0..1)
    class PeakMeter : public juce::Component {
    public:
        void setLevel (float l) { level = juce::jlimit (0.0f, 1.0f, l); repaint(); }
        void paint (juce::Graphics& g) override {
            auto r = getLocalBounds().reduced (3);
            g.setColour (juce::Colours::dimgrey);
            g.drawRect (r);
            auto h = (int) std::round (r.getHeight() * level);
            auto fill = r.removeFromBottom (h);
            g.setColour (juce::Colours::chartreuse.withAlpha (0.9f));
            g.fillRect (fill);
        }
    private:
        float level = 0.0f;
    };

    using Attachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    using ComboAttachment = juce::AudioProcessorValueTreeState::ComboBoxAttachment;
    FxSaturationAudioProcessor& proc;
    juce::AudioVisualiserComponent scope { 1 };
    juce::Slider input, drive, output, mix, tone, bias, zoom;
    juce::ToggleButton genEnable;
    juce::Slider genFreq, genAmp;
    juce::Label inputLabel, driveLabel, outputLabel, mixLabel, toneLabel, biasLabel, freqLabel, ampLabel, modeLabel, zoomLabel, osLabel;
    SatLED satLed;
    PeakMeter inMeter, outMeter;
    juce::ComboBox modeBox;
    juce::ComboBox osBox;
    std::vector<float> genTemp, scopeTemp;
    double genPhase = 0.0;
    std::unique_ptr<Attachment> inputAttachment, driveAttachment, outputAttachment, mixAttachment, toneAttachment, biasAttachment, zoomAttachment;
    std::unique_ptr<ComboAttachment> modeAttachment, osAttachment;
    int lastSpp = -1;
};
} // namespace

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
    // Prepare tone filter states for max possible channels (updated in processBlock)
    const int maxCh = std::max (getTotalNumInputChannels(), getTotalNumOutputChannels());
    toneStates.assign ((size_t) std::max (1, maxCh), {});
    resetScope();

    // Setup/recreate oversampling based on current parameter
    int mode = 0;
    if (osModeParam != nullptr) mode = juce::jlimit (0, 1, (int) std::round (osModeParam->load()));
    osFactor = (mode == 1 ? 2 : 1);
    if (osFactor == 1) {
        oversampling.reset();
        setLatencySamples (0);
    } else {
        const int numCh = std::max (1, getTotalNumOutputChannels());
        oversampling = std::make_unique<juce::dsp::Oversampling<float>>(numCh, /*numStages*/ 1,
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
    // Read parameters (with safe defaults)
    const float inDb    = inputDbParam != nullptr ? inputDbParam->load() : 0.0f;
    const float driveDb  = driveDbParam  != nullptr ? driveDbParam->load()  : 12.0f;
    const float outDb    = outputDbParam != nullptr ? outputDbParam->load() : 0.0f;
    const float mix      = mixParam      != nullptr ? juce::jlimit (0.0f, 1.0f, mixParam->load()) : 1.0f;
    const float tone     = toneParam     != nullptr ? juce::jlimit (-1.0f, 1.0f, toneParam->load()) : 0.0f;
    const float biasAmt  = biasParam     != nullptr ? juce::jlimit (-1.0f, 1.0f, biasParam->load()) : 0.0f;

    const float inGain   = dbToGain (inDb);
    const float driveGain = dbToGain (driveDb);
    const float outGain   = dbToGain (outDb);

    if ((int) toneStates.size() < numChannels)
        toneStates.resize ((size_t) numChannels);

    // Tone LPF coefficient (fixed cutoff around 800 Hz). Compute base-rate and OS-rate variants.
    const float fc = 800.0f;
    const float aBase = std::exp (-2.0f * juce::MathConstants<float>::pi * fc / (float) sampleRate);
    const float bBase = 1.0f - aBase;
    const float aOS   = std::exp (-2.0f * juce::MathConstants<float>::pi * fc / (float) (sampleRate * 2.0));
    const float bOS   = 1.0f - aOS;

    float satPeak = 0.0f;
    float inPk = 0.0f, outPk = 0.0f;

    // Save dry (post-input gain) for mix
    dryBuffer.setSize (numChannels, numSamples, false, false, true);
    for (int ch = 0; ch < numChannels; ++ch)
    {
        const float* src = buffer.getReadPointer (ch);
        float* dry = dryBuffer.getWritePointer (ch);
        for (int i = 0; i < numSamples; ++i) {
            const float v = inGain * src[i];
            dry[i] = v;
            inPk = juce::jmax (inPk, std::abs (v));
        }
    }

    // Process path (possibly oversampled)
    if (osFactor == 2 && oversampling != nullptr)
    {
        // Up -> process -> Down
        juce::dsp::AudioBlock<float> inBlock (dryBuffer);
        auto upBlock = oversampling->processSamplesUp (inBlock);

        // Per-sample shape at 2x, process in-place on upBlock
        const int osNumSamples = (int) upBlock.getNumSamples();
        const int osNumCh = (int) upBlock.getNumChannels();
        for (int ch = 0; ch < osNumCh; ++ch)
        {
            float* s = upBlock.getChannelPointer ((size_t) ch);
            auto& st = toneStates[(size_t) (ch % (int) toneStates.size())];
            for (int i = 0; i < osNumSamples; ++i)
            {
                const float dry = s[i];
                st.lp = aOS * st.lp + bOS * dry;
                const float hp = dry - st.lp;
                const float pre = dry + tone * hp;
                const float x = pre + 0.2f * biasAmt;
                const float preDrive = x;
                const float d = driveGain * preDrive;
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
            for (int i = 0; i < numSamples; ++i)
            {
                float y = mix * dst[i] + (1.0f - mix) * dry[i];
                y *= outGain;
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
            auto& st = toneStates[(size_t) ch];
            for (int i = 0; i < numSamples; ++i)
            {
                const float dry = dryPostIn[i];
                // One-pole lowpass for tilt EQ
                st.lp = aBase * st.lp + bBase * dry;
                const float hp = dry - st.lp; // high-passed portion
                const float pre = dry + tone * hp; // tilt: boost/cut highs
                // Add bias to create asymmetry (subtle range)
                const float x = pre + 0.2f * biasAmt;
                // Waveshape
                const float preDrive = x;
                const float d = driveGain * preDrive;
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
                float y = mix * ySat + (1.0f - mix) * dry;
                y *= outGain;
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
        juce::StringArray{ "Off", "2x" },
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
