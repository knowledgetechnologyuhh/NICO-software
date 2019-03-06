# Say text
# Use google tts and cache the spoken mp3s
# As fallback use pyttsx3
def say(sen):
    import os.path
    fname = "./wav_cache/"+sen+".mp3"
    from gtts import gTTS

    #try:
    if True:    
        if not (os.path.isfile(fname)):
            import urllib2
            urllib2.urlopen('http://216.58.192.142', timeout=1)
            tts = gTTS(text=sen, lang='en-au', slow=False)
            # tts.save("/tmp/say.mp3")
            tts.save(fname)
        comm = ["mpg123", fname]
        subprocess.check_call(comm)

