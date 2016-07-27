from nicovision import ImageRecorder

device = ImageRecorder.getDevices()[0]
print('Will take an image from device %s - smile :)' % device)
ir = ImageRecorder.ImageRecorder(device)
path = ir.saveOneImage()
if path is '':
    print('Oh, something has gone wrong')
else:
    print('Thank you - your image can be found at ' + path)