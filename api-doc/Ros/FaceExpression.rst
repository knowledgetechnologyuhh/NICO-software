nicoros - FaceExpression
************************

The FaceExpression class allows to manipulate the facial expression of NICO through ROS.

To start the interface run (use -h to list optional arguments):

.. code-block:: bash

   rosrun nicoros FaceExpression.py

Exposed topics
==============

The following ROS topics are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/faceExpression/):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/sendFaceExpression                     | nicomsg/s                                      | Sets facial expression to one of the presets.                                         |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   1. 'happiness','sadness','anger','disgust','surprise','fear','neutral', or 'clean'  |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/send_morphable_expression              | nicomsg/s                                      | Sets facial expression to one of the polynomial based presets.                        |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   1. 'happiness','sadness','anger','disgust','surprise','fear', or 'neutral'          |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/morph_face_expression                  | msg/s                                          | Morphs current facial expression into the given presets. The current face has to be   |
|                                                |                                                | polynomial or wavelet based.                                                          |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   1. 'happiness','sadness','anger','disgust','surprise','fear', or 'neutral'          |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/send_polynomial_mouth                  | msg/polynomial_mouth                           | Sends parameters to create the mouth from up two n-degree polynomials.                |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   - degs1: factors for each degree of the first polynomial                            |
|                                                |                                                |   - degs2: factors for each degree of the second polynomial                           |
|                                                |                                                |   - x_shift: Shifts curve to the right                                                |
|                                                |                                                |   - crop_left: black border on the left of the image                                  |
|                                                |                                                |   - crop_right: black border on the right of the image                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/send_polynomial_eyebrow                | msg/polynomial_eyebrow                         | Sends parameters to create the mouth from an n-degree polynomial.                     |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   - degs: factors for each degree of the first polynomial                             |
|                                                |                                                |   - x_shift: Shifts curve to the right                                                |
|                                                |                                                |   - crop_left: black border on the left of the image                                  |
|                                                |                                                |   - crop_right: black border on the right of the image                                |
|                                                |                                                |   - left: whether left or right eyebrow is generated                                  |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/send_polynomial_face                   | msg/polynomial_face                            | Sends combined message to create a full polynomial based face.                        |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   - mouth: polynomial_mouth message (see send_polynomial_mouth)                       |
|                                                |                                                |   - brow_left: polynomial_eyebrow message (see send_polynomial_eyebrow)               |
|                                                |                                                |   - brow_right: polynomial_eyebrow message (see send_polynomial_eyebrow)              |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/morph_polynomial_face                  | msg/polynomial_face                            | Morph current face into a target polynomial based face.                               |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   - mouth: polynomial_mouth message (see send_polynomial_mouth)                       |
|                                                |                                                |   - brow_left: polynomial_eyebrow message (see send_polynomial_eyebrow)               |
|                                                |                                                |   - brow_right: polynomial_eyebrow message (see send_polynomial_eyebrow)              |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/send_trained_expression                | msg/s                                          | Sets facial expression to one of the wavelet based presets trained by a network.      |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   1. 'happiness','sadness','anger','disgust','surprise','fear', or 'neutral'          |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/sendMouth                              | nicomsg/affffa                                 | Sends up to two wavelets to create the mouth.                                         |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   1. Array of (ystr,yoff,xstr,xoff) tuples                                            |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/sendEyebrow                            | nicomsg/sffff                                  | Sends a wavelet to create one of the eybrows.                                         |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   1. 'l' or 'r'                                                                       |
|                                                |                                                |   2. stretch in y position                                                            |
|                                                |                                                |   3. offset in y position                                                             |
|                                                |                                                |   4. stretch in x position                                                            |
|                                                |                                                |   5. offset in x position                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/send_wavelet_face                      | msg/wavelet_face                               | Sends combined message to create a full wavelet based face.                           |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   - mouth: affffa message (see sendMouth)                                             |
|                                                |                                                |   - brow_left: sffff message (see sendEyebrow)                                        |
|                                                |                                                |   - brow_right: sffff message (see sendEyebrow)                                       |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/morph_wavelet_face                     | msg/wavelet_face                               | Morph current face into a target wavelet based face.                                  |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   - mouth: affffa message (see sendMouth)                                             |
|                                                |                                                |   - brow_left: sffff message (see sendEyebrow)                                        |
|                                                |                                                |   - brow_right: sffff message (see sendEyebrow)                                       |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/send_bitmap                            | msg/bitmap_face                                | Sends bitmaps to directly set the face images.                                        |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |   - brow_left: flattened 8x8 array of 0's (led off) and 1's (led on)                  |
|                                                |                                                |   - brow_right: flattened 8x8 array of 0's (led off) and 1's (led on)                 |
|                                                |                                                |   - mouth: flattened 8x16 array of 0's (led off) and 1's (led on)                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Class documentation
===================

.. automodule:: FaceExpression
    :members:
    :undoc-members:
    :show-inheritance:
