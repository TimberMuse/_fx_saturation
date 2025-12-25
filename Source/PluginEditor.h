// SPDX-License-Identifier: MIT
#pragma once

#include "PluginProcessor.h"

/**
    The Editor class handles the User Interface.
    In JUCE, the Editor is a 'Component' that holds sliders, buttons, and graphics.
    It should only contain UI logic and visual updates, leaving the DSP to the Processor.
*/
class FxSaturationAudioProcessorEditor : public juce::AudioProcessorEditor, 
                                          private juce::Timer
{
public:
    explicit FxSaturationAudioProcessorEditor (FxSaturationAudioProcessor& p);
    ~FxSaturationAudioProcessorEditor() override;

    void paint (juce::Graphics& g) override;
    void resized() override;

private:
    void timerCallback() override;

    static void ensureBufferSize (std::vector<float>& v, int n);

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

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (FxSaturationAudioProcessorEditor)
};
