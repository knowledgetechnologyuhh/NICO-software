import numpy as np

import matplotlib.pyplot
import numpy
import Image

def ricker(f, length=0.512, dt=0.001,sca=1,level=0,xoff=0,xstr=1,yoff=0,ystr=1):
    t = np.linspace(-length/2, (length-dt)/2, length/dt)
    y = ((1.-2.*(np.pi**2)*(f**2)*(t**2))*np.exp(-(np.pi**2)*(f**2)*(t**2))*sca)+level
    return t*xstr+xoff, y*ystr+yoff

def fig2data(fig):
    """
    @brief Convert a Matplotlib figure to a 4D numpy array with RGBA channels and return it
    @param fig a matplotlib figure
    @return a numpy 3D array of RGBA values
    http://www.icare.univ-lille1.fr/wiki/index.php/How_to_convert_a_matplotlib_figure_to_a_numpy_array_or_a_PIL_image
    """
    # draw the renderer
    fig.canvas.draw()

    # Get the RGBA buffer from the figure
    w, h = fig.canvas.get_width_height()
    buf = numpy.fromstring(fig.canvas.tostring_argb(), dtype=numpy.uint8)
    buf.shape = (w, h, 4)

    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    buf = numpy.roll(buf, 3, axis=2)
    return buf





def fig2img(fig):
    """
    @brief Convert a Matplotlib figure to a PIL Image in RGBA format and return it
    @param fig a matplotlib figure
    @return a Python Imaging Library ( PIL ) image
    http://www.icare.univ-lille1.fr/wiki/index.php/How_to_convert_a_matplotlib_figure_to_a_numpy_array_or_a_PIL_image
    """
    # put the figure pixmap into a numpy array

    import PIL

    buf = fig2data(fig)
    w, h, d = buf.shape
    image_file=Image.fromstring("RGBA", (w, h), buf.tostring())
    #image_file.convert('1')
    image_file.show()
    image_file = image_file.resize((16,8),PIL.Image.ANTIALIAS)
    #image_file = image_file.resize((16, 8))
    image_file = image_file.resize((320,160))
    image_file = binarize_image(image_file,141)
    return image_file

def binarize_image(image, threshold):
    """Binarize an image."""
    image = image.convert('L')  # convert image to monochrome
    image = numpy.array(image)
    image = Image.fromarray(binarize_array(image, threshold))
    return image

def binarize_array(numpy_array, threshold=200):
    """Binarize a numpy array."""
    for i in range(len(numpy_array)):
        for j in range(len(numpy_array[0])):
            if numpy_array[i][j] > threshold:
                numpy_array[i][j] = 255
            else:
                numpy_array[i][j] = 0
    return numpy_array

figure = matplotlib.pyplot.figure(  )
plot   = figure.add_subplot ( 111 )
figure.patch.set_facecolor('white')

#plot.ylim((-1.1,1.1))
#plot.xlim((-1.1,1.1))
#plt.xlim((min(t),max(t)))

f = 0.2
t, y = ricker (f,2,dt=0.0001,ystr=-1.4,yoff=0.4)
lines=plot.plot(t, y,'k')
matplotlib.pyplot.setp(lines, linewidth=40, color='k')

f = 0.2
t, y = ricker (f,2,dt=0.0001,ystr=0.6,yoff=0.4)
lines2=plot.plot(t, y,'k')
matplotlib.pyplot.setp(lines2, linewidth=40, color='k')

matplotlib.pyplot.axis("off")
matplotlib.pyplot.ylim((-1.1,1.1))
matplotlib.pyplot.xlim((-1.1,1.1))
#matplotlib.pyplot.show()

im = fig2img ( figure )
#pix = np.array(im.getdata()).reshape(8,8)
im.show()