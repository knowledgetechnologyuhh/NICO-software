<b>Frame-Based Emotion Categorization <br>

![Screenshot](demo.png)

This demos is configured to run using two different models: one for categorical emotions and other for arousal/valence intervals.


Both implemented KERAS models are based on the Face Channel of the Crosschanel CNN - more information can be found here: <br>

```sh
Barros, P., & Wermter, S. (2016). Developing crossmodal expression recognition based on a deep neural model. Adaptive behavior, 24(5), 373-396.
http://journals.sagepub.com/doi/full/10.1177/1059712316664017
```

<b>Requirements

Numpy<br>
OpenCV-python<br>
Keras <br>
Tensorflow<br>
dlib<br>
h5py<br>

<b>Instructions


To run the demo with your own model (has to be saved as a KERAS model), add an entry on the modelDictionary.py containing the model's directory, class dictionary and type. Also, change the run.py to matche your inputsize (faceSize).

<br>
The run.py file contains all the necessary configurations.

<br>
To run the demo just use
```sh
$ python run.py

```


<b> Contact

Pablo Barros - barros@informatik.uni-hamburg.de




