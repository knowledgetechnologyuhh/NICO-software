from nicovision import ImageRecorder

device = ImageRecorder.get_devices()[0]
print('Will take an image from device %s - smile :)' % device)
ir = ImageRecorder.ImageRecorder(device, 640, 480)
path = ir.save_one_image()
if path is '':
    print('Oh, something has gone wrong')
else:
    print('Thank you - your image can be found at ' + path)
