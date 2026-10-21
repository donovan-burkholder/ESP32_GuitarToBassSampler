#include <Arduino.h>
#include <math.h>
#include <ESP_I2S.h>
#include <new>
#include <esp_adc/adc_continuous.h>
#include "Chromagram.h"
#include "ChordDetector.h"

#define ADC_PIN       34
#define SAMPLE_RATE   20000 // 20 kHz sampling rate for ADC
#define FFT_SIZE      512

#define ATTACK_MIN_LEVEL   40

// ============================================================
// PCM5102 / I2S
// ============================================================

#define I2S_BCLK  26
#define I2S_LRCLK 25
#define I2S_DOUT  22

#define AUDIO_SAMPLE_RATE 44100

// Small audio buffer
#define AUDIO_BUFFER_FRAMES 128

// Output volume
// 0.20 = 20% of full-scale
#define AUDIO_AMPLITUDE 0.20

I2SClass I2S;


double capturedAudioFrame[FFT_SIZE];

adc_continuous_handle_t adcHandle = nullptr;
adc_unit_t adcUnit = ADC_UNIT_1;
adc_channel_t adcChannel;
alignas(4) uint8_t adcFrameBuffer[FFT_SIZE * sizeof(adc_digi_output_data_t)];

Chromagram* chromagram = nullptr;
ChordDetector chordDetector;

// ============================================================
// DETECTOR STATE
// ============================================================

bool sampling = false;

uint32_t chromaSamplesProcessed = 0;
int candidateChordKey = -1;
uint8_t candidateChordFrames = 0;
int activeChordKey = -1;
bool chordReportPending = false;
int reportedChordRoot = 0;
int reportedChordQuality = 0;
int reportedChordIntervals = 0;
float reportedBassFrequency = 0.0f;
double reportedRms = 0.0;
double reportedChroma[12];

bool initializeAdc()
{
    esp_err_t error =
        adc_continuous_io_to_channel(
            ADC_PIN,
            &adcUnit,
            &adcChannel
        );

    if (error != ESP_OK)
    {
        Serial.printf(
            "ERROR: ADC pin mapping failed: %s\n",
            esp_err_to_name(error)
        );
        return false;
    }

    adc_continuous_handle_cfg_t handleConfig = {};
    handleConfig.max_store_buf_size = sizeof(adcFrameBuffer) * 4;
    handleConfig.conv_frame_size = sizeof(adcFrameBuffer);

    error =
        adc_continuous_new_handle(
            &handleConfig,
            &adcHandle
        );

    if (error != ESP_OK)
    {
        Serial.printf(
            "ERROR: ADC continuous driver init failed: %s\n",
            esp_err_to_name(error)
        );
        return false;
    }

    adc_digi_pattern_config_t pattern = {};
    pattern.atten = ADC_ATTEN_DB_12;
    pattern.channel = adcChannel;
    pattern.unit = adcUnit;
    pattern.bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t config = {};
    config.sample_freq_hz = SAMPLE_RATE;
    config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
    config.pattern_num = 1;
    config.adc_pattern = &pattern;

    error = adc_continuous_config(adcHandle, &config);
    if (error != ESP_OK)
    {
        Serial.printf(
            "ERROR: ADC continuous config failed: %s\n",
            esp_err_to_name(error)
        );
        return false;
    }

    return true;
}

// ============================================================
// NOTE NAMES
// ============================================================

const char* noteNames[] = {
    "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

// ============================================================
// AUDIO STATE
// ============================================================

// This is shared between the detector and audio task.
volatile float audioFrequency = 0.0f;
volatile bool audioEnabled = false;

// ============================================================
// SINE WAVETABLE
// ============================================================

#define SINE_TABLE_SIZE 256

int16_t sineTable[SINE_TABLE_SIZE];


// ============================================================
// AUDIO BUFFER
// ============================================================

int16_t audioBuffer[AUDIO_BUFFER_FRAMES * 2];


// ============================================================
// FREQUENCY -> NOTE
// ============================================================

String frequencyToNote(double frequency)
{
    if (frequency <= 0)
        return "---";

    double midi =
        69.0 + 12.0 * log2(frequency / 440.0);

    int noteNumber = (int)round(midi);

    if (noteNumber < 0 || noteNumber > 127)
        return "---";

    int octave = (noteNumber / 12) - 1;

    int noteIndex = noteNumber % 12;

    String result = noteNames[noteIndex];

    result += String(octave);

    return result;
}

String chordToName(int rootNote, int quality, int intervals)
{
    if (rootNote < 0 || rootNote >= 12)
        return "Unknown";

    String name = noteNames[rootNote];

    switch (quality)
    {
        case ChordDetector::Major:
            name += intervals == 7 ? "maj7" : "";
            break;
        case ChordDetector::Minor:
            name += intervals == 7 ? "m7" : "m";
            break;
        case ChordDetector::Suspended:
            name += intervals == 2 ? "sus2" : "sus4";
            break;
        case ChordDetector::Dominant:
            name += "7";
            break;
        case ChordDetector::Dimished5th:
            name += "dim";
            break;
        case ChordDetector::Augmented5th:
            name += "+";
            break;
        default:
            name += "?";
            break;
    }

    return name;
}

float chordRootBassFrequency(int rootNote)
{
    const int midiNote = 36 + rootNote;
    return 440.0f * powf(2.0f, (midiNote - 69) / 12.0f);
}


// ============================================================
// AUDIO TASK
// ============================================================

void audioTask(void *parameter)
{
    uint32_t phase = 0;

    while (true)
    {
        float frequency = audioFrequency;
        bool enabled = audioEnabled;

        uint32_t phaseIncrement = 0;

        if (enabled && frequency > 0.0f)
        {
            phaseIncrement =
                (uint32_t)(
                    (frequency / (float)AUDIO_SAMPLE_RATE)
                    * 4294967296.0
                );
        }

        for (int i = 0; i < AUDIO_BUFFER_FRAMES; i++)
        {
            int16_t sample = 0;

            if (enabled && frequency > 0.0f)
            {
                uint8_t index = phase >> 24;

                sample =
                    (int16_t)(
                        sineTable[index]
                        * AUDIO_AMPLITUDE
                    );

                phase += phaseIncrement;
            }

            // Stereo
            audioBuffer[i * 2]     = sample;
            audioBuffer[i * 2 + 1] = sample;
        }

        // Arduino-ESP32 3.x API requires byte pointer
        I2S.write(
            (const uint8_t*)audioBuffer,
            sizeof(audioBuffer)
        );
    }
}


// ============================================================
// CAPTURE FFT FRAME
// ============================================================

bool captureAudioFrame(double &rms)
{
    uint32_t bytesRead = 0;
    esp_err_t error =
        adc_continuous_read(
            adcHandle,
            adcFrameBuffer,
            sizeof(adcFrameBuffer),
            &bytesRead,
            100
        );

    if (error != ESP_OK)
    {
        Serial.printf(
            "ERROR: ADC frame read failed: %s\n",
            esp_err_to_name(error)
        );
        return false;
    }

    if (bytesRead != sizeof(adcFrameBuffer))
    {
        Serial.printf(
            "ERROR: ADC returned %u of %u frame bytes\n",
            bytesRead,
            (unsigned int)sizeof(adcFrameBuffer)
        );
        return false;
    }

    const adc_digi_output_data_t* samples =
        reinterpret_cast<const adc_digi_output_data_t*>(adcFrameBuffer);

    double sum = 0.0;

    for (int i = 0; i < FFT_SIZE; i++)
    {
        if (samples[i].type1.channel != adcChannel)
        {
            Serial.printf(
                "ERROR: ADC frame channel mismatch at sample %d\n",
                i
            );
            return false;
        }

        capturedAudioFrame[i] = samples[i].type1.data;
        sum += capturedAudioFrame[i];
    }


    // --------------------------------------------------------
    // Remove DC offset
    // --------------------------------------------------------

    double average =
        sum / FFT_SIZE;

    double sumSquares = 0.0;

    for (int i = 0; i < FFT_SIZE; i++)
    {
        capturedAudioFrame[i] -= average;
        sumSquares += capturedAudioFrame[i] * capturedAudioFrame[i];
    }

    rms = sqrt(sumSquares / FFT_SIZE);

    return true;
}


// ============================================================
// START SAMPLING
// ============================================================

void startSampling()
{
    chromaSamplesProcessed = 0;
    candidateChordKey = -1;
    candidateChordFrames = 0;
    activeChordKey = -1;
    chordReportPending = false;
    chromagram->reset();

    esp_err_t error = adc_continuous_flush_pool(adcHandle);
    if (error != ESP_OK)
    {
        Serial.printf(
            "ERROR: Unable to clear ADC sample queue: %s\n",
            esp_err_to_name(error)
        );
        return;
    }

    error = adc_continuous_start(adcHandle);
    if (error != ESP_OK)
    {
        Serial.printf(
            "ERROR: Unable to start ADC sampling: %s\n",
            esp_err_to_name(error)
        );
        return;
    }

    sampling = true;

    Serial.println();
    Serial.println("========================================");
    Serial.println("SAMPLING STARTED");
    Serial.println("========================================");
    Serial.println();

    Serial.println("Settings:");
    Serial.printf("Sample rate: %d Hz\n", SAMPLE_RATE);
    Serial.printf("FFT size: %d\n", FFT_SIZE);
    Serial.printf(
        "Chord analysis window: %.2f s\n",
        (double)(Chromagram::FFT_BUFFER_SIZE * 4) / SAMPLE_RATE
    );
    Serial.println("Bass output follows the detected chord root.");
    Serial.println();
}


// ============================================================
// STOP SAMPLING
// ============================================================

void stopSampling()
{
    sampling = false;

    esp_err_t error = adc_continuous_stop(adcHandle);
    if (error != ESP_OK)
    {
        Serial.printf(
            "ERROR: Unable to stop ADC sampling: %s\n",
            esp_err_to_name(error)
        );
    }

    audioEnabled = false;
    audioFrequency = 0.0f;
    chromaSamplesProcessed = 0;
    candidateChordKey = -1;
    candidateChordFrames = 0;
    activeChordKey = -1;
    chordReportPending = false;

    Serial.println();
    Serial.println("SAMPLING STOPPED");
    Serial.println();
}


// ============================================================
// PROCESS ONE FRAME
// ============================================================

void processFrame()
{
    double rms = 0.0;

    if (!captureAudioFrame(rms))
    {
        return;
    }

    chromagram->processAudioFrame(capturedAudioFrame);
    if (chromaSamplesProcessed < Chromagram::FFT_BUFFER_SIZE * 4)
    {
        chromaSamplesProcessed += FFT_SIZE;
    }


    // --------------------------------------------------------
    // Chord recognition
    // --------------------------------------------------------

    unsigned long now =
        millis();

    if (chromagram->isReady() &&
        chromaSamplesProcessed >= Chromagram::FFT_BUFFER_SIZE * 4 &&
        rms >= ATTACK_MIN_LEVEL)
    {
        const std::vector<double> chroma = chromagram->getChromagram();
        double chromaValues[12];
        for (int i = 0; i < 12; i++)
        {
            chromaValues[i] = chroma[i];
        }

        chordDetector.detectChord(chromaValues);

        const int chordKey =
            chordDetector.rootNote * 64 +
            chordDetector.quality * 8 +
            chordDetector.intervals;

        if (chordKey == candidateChordKey)
        {
            if (candidateChordFrames < 2)
                candidateChordFrames++;
        }
        else
        {
            candidateChordKey = chordKey;
            candidateChordFrames = 1;
        }

        if (candidateChordFrames >= 2 &&
            chordKey != activeChordKey)
        {
            const float bassFrequency =
                chordRootBassFrequency(chordDetector.rootNote);

            audioFrequency = bassFrequency;
            audioEnabled = true;
            activeChordKey = chordKey;
            reportedChordRoot = chordDetector.rootNote;
            reportedChordQuality = chordDetector.quality;
            reportedChordIntervals = chordDetector.intervals;
            reportedBassFrequency = bassFrequency;
            reportedRms = rms;
            for (int i = 0; i < 12; i++)
            {
                reportedChroma[i] = chromaValues[i];
            }
            chordReportPending = true;
        }
    }
    else if (rms < ATTACK_MIN_LEVEL)
    {
        candidateChordKey = -1;
        candidateChordFrames = 0;
    }

    if (chordReportPending && Serial.availableForWrite() >= 120)
    {
        const String chordName =
            chordToName(
                reportedChordRoot,
                reportedChordQuality,
                reportedChordIntervals
            );

        Serial.printf(
            "CHORD %s bass %.1f Hz %s RMS %.0f CHROMA",
            chordName.c_str(),
            reportedBassFrequency,
            frequencyToNote(reportedBassFrequency).c_str(),
            reportedRms
        );
        for (int i = 0; i < 12; i++)
        {
            Serial.printf(" %.0f", reportedChroma[i]);
        }
        Serial.println();
        chordReportPending = false;
    }


    // Print RMS periodically without flooding the serial monitor.

    static unsigned long lastMonitorPrint = 0;

    if (now - lastMonitorPrint >= 200 &&
        Serial.availableForWrite() >= 112)
    {
        lastMonitorPrint = now;

        Serial.printf("RMS: %.1f\n", rms);
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32 GUITAR -> PCM5102 TEST");
    Serial.println("========================================");
    Serial.println();

    Serial.printf("Input: %d Hz / %d samples\n", SAMPLE_RATE, FFT_SIZE);

    Serial.printf(
        "Audio: %d Hz / 16-bit stereo\n",
        AUDIO_SAMPLE_RATE
    );

    Serial.println();

    // Chromagram allocates working buffers and KissFFT state, so construct it
    // only after the Arduino runtime has initialized the heap.
    Serial.printf(
        "Free heap before chord analyzer: %u bytes\n",
        ESP.getFreeHeap()
    );
    chromagram = new (std::nothrow) Chromagram(FFT_SIZE, SAMPLE_RATE);
    if (chromagram == nullptr)
    {
        Serial.println("ERROR: Unable to allocate chromagram.");
        while (true)
        {
            delay(1000);
        }
    }

    // --------------------------------------------------------
    // ADC continuous sampling
    // --------------------------------------------------------

    if (!initializeAdc())
    {
        while (true)
        {
            delay(1000);
        }
    }

    // --------------------------------------------------------
    // Create sine wavetable
    // --------------------------------------------------------

    for (int i = 0; i < SINE_TABLE_SIZE; i++)
    {
        double angle =
            2.0 * M_PI *
            ((double)i / SINE_TABLE_SIZE);

        sineTable[i] =
            (int16_t)(
                sin(angle) * 32767.0
            );
    }


    // --------------------------------------------------------
    // Initialize PCM5102
    // --------------------------------------------------------

    Serial.println(
        "Initializing PCM5102..."
    );

    I2S.setPins(
        I2S_BCLK,
        I2S_LRCLK,
        I2S_DOUT
    );


    bool i2sOK =
        I2S.begin(
            I2S_MODE_STD,
            AUDIO_SAMPLE_RATE,
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        );


    if (!i2sOK)
    {
        Serial.println(
            "ERROR: I2S initialization failed!"
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println(
        "PCM5102 I2S initialized."
    );

    Serial.println();

    Serial.println(
        "Press S to start/stop sampling."
    );

    Serial.println();


    // --------------------------------------------------------
    // Start audio task
    // --------------------------------------------------------

    xTaskCreatePinnedToCore(
        audioTask,
        "AudioTask",
        4096,
        NULL,
        1,
        NULL,
        0
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Serial command
    // --------------------------------------------------------

    if (Serial.available())
    {
        char command =
            Serial.read();

        if (command == 'S' ||
            command == 's')
        {
            if (sampling)
            {
                stopSampling();
            }
            else
            {
                startSampling();
            }
        }
    }


    // --------------------------------------------------------
    // Detector
    // --------------------------------------------------------

    if (!sampling)
    {
        delay(10);
        return;
    }


    processFrame();
}