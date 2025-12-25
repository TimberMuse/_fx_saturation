// SPDX-License-Identifier: MIT
#include "PluginEditor.h"

FxSaturationAudioProcessorEditor::FxSaturationAudioProcessorEditor (FxSaturationAudioProcessor& p)
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
    osBox.addItemList (juce::StringArray { "Off", "2x", "4x", "8x" }, 1);
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

FxSaturationAudioProcessorEditor::~FxSaturationAudioProcessorEditor()
{
}

void FxSaturationAudioProcessorEditor::paint (juce::Graphics& g)
{
    g.fillAll (juce::Colours::black);
    auto area = getLocalBounds();
    ColourTitleBar title (area.removeFromTop (28), "_fx_saturation", juce::Colours::orange);
    title.paint (g);
}

void FxSaturationAudioProcessorEditor::resized()
{
    auto r = getLocalBounds().reduced (12);
    r.removeFromTop (32); // title spacing

    // Top: scope
    auto scopeArea = r.removeFromTop (juce::jmax (120, r.getHeight() * 3 / 5)).reduced (4);
    scope.setBounds (scopeArea);

    // Bottom: controls
    auto row = r.removeFromTop (r.getHeight());
    auto w = row.getWidth() / 15; // 6 knobs + Gen + Freq + Amp + LED + Mode + Zoom + OS + 2 meters + spacing
    auto makeBox = [&] () { return row.removeFromLeft (w).reduced (4); };
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

void FxSaturationAudioProcessorEditor::timerCallback()
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

void FxSaturationAudioProcessorEditor::ensureBufferSize (std::vector<float>& v, int n)
{
    if ((int) v.size() < n) v.resize ((size_t) n, 0.0f);
}
