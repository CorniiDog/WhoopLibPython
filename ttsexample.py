import pyttsx3

def test_tts():
    engine = pyttsx3.init()

    # Optional: Set properties (rate, volume, and voice)
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate - 50)  # Slow down the speech rate

    volume = engine.getProperty('volume')
    engine.setProperty('volume', volume + 0.25)  # Increase volume

    voices = engine.getProperty('voices')
    engine.setProperty('voice', voices[1].id)  # Change voice (0 for male, 1 for female)

    # Test message
    engine.say("Hello, this is a test of the text to speech system.")
    engine.runAndWait()

if __name__ == "__main__":
    test_tts()