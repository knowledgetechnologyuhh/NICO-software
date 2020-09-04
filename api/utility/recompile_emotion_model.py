# load models from seperate weight and architecture files, compile + save as h5
from os.path import dirname, abspath
from keras.models import Model, Sequential, Input, load_model
from keras.layers import Dense, Dropout, Flatten, Lambda
from keras.layers.convolutional import Conv2D, MaxPooling2D, AveragePooling2D
from keras.layers.core import Activation
from keras.layers.normalization import BatchNormalization
from keras.optimizers import Adam, Adamax
from nicoemotionrecognition._nicoemotionrecognition_internal import metrics
import numpy as np
import keras.backend as K

K.set_image_data_format("channels_first")


def buildModelCategorical(inputShape, numberOfOutputs):
    def shuntingInhibition(inputs):
        inhibitionDecay = 0.5

        v_c, v_c_inhibit = inputs

        output = v_c / (inhibitionDecay + v_c_inhibit)

        return output

    print("Input shape:", inputShape)

    nch = 256
    # h = 5
    # reg = keras.regularizers.L1L2(1e-7, 1e-7)
    #
    # model = Sequential()

    inputLayer = Input(shape=inputShape, name="Network_Input")

    # Conv1 and 2
    conv1 = Conv2D(
        nch // 4,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        name="conv1",
    )(inputLayer)
    bn1 = BatchNormalization(axis=1)(conv1)
    actv1 = Activation("relu")(bn1)

    conv2 = Conv2D(
        nch // 4,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        name="conv2",
    )(actv1)
    bn2 = BatchNormalization(axis=1)(conv2)
    actv2 = Activation("relu")(bn2)

    mp1 = MaxPooling2D(pool_size=(2, 2))(actv2)
    drop1 = Dropout(0.25)(mp1)

    # Conv 3 and 4
    conv3 = Conv2D(
        nch // 2,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        name="conv3",
    )(drop1)
    bn3 = BatchNormalization(axis=1)(conv3)
    actv3 = Activation("relu")(bn3)

    conv4 = Conv2D(
        nch // 2,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        name="conv4",
    )(actv3)
    bn4 = BatchNormalization(axis=1)(conv4)
    actv4 = Activation("relu")(bn4)

    mp2 = MaxPooling2D(pool_size=(2, 2))(actv4)
    drop2 = Dropout(0.25)(mp2)

    # Conv 5 and 6 and 7
    conv5 = Conv2D(
        nch // 2,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        name="conv5",
    )(drop2)
    bn5 = BatchNormalization(axis=1)(conv5)
    actv5 = Activation("relu")(bn5)

    conv6 = Conv2D(
        nch // 2,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        name="conv6",
    )(actv5)
    bn6 = BatchNormalization(axis=1)(conv6)
    actv6 = Activation("relu")(bn6)

    conv7 = Conv2D(
        nch // 2,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        name="conv7",
    )(actv6)
    bn7 = BatchNormalization(axis=1)(conv7)
    actv7 = Activation("relu")(bn7)

    mp3 = MaxPooling2D(pool_size=(2, 2))(actv7)
    drop3 = Dropout(0.25)(mp3)

    # Conv 8 and 9 and 10

    conv8 = Conv2D(
        nch, (3, 3), padding="same", kernel_initializer="glorot_uniform", name="conv8"
    )(drop3)
    bn8 = BatchNormalization(axis=1)(conv8)
    actv8 = Activation("relu")(bn8)

    conv9 = Conv2D(
        nch, (3, 3), padding="same", kernel_initializer="glorot_uniform", name="conv9"
    )(actv8)
    bn9 = BatchNormalization(axis=1)(conv9)
    actv9 = Activation("relu")(bn9)

    conv10 = Conv2D(
        nch,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        activation="relu",
        name="conv10",
    )(actv9)
    # bn10 = BatchNormalization(axis=1)(conv10)
    # actv10 = Activation("relu")(conv10)

    conv10_inhibition = Conv2D(
        nch,
        (3, 3),
        padding="same",
        kernel_initializer="glorot_uniform",
        activation="relu",
        name="conv10_inhibition",
    )(actv9)
    # bn10_inhibition = BatchNormalization(axis=1)(conv10_inhibition)
    # actv10_inhibition = Activation("relu")(conv10_inhibition)

    v_conv_inhibitted = Lambda(function=shuntingInhibition)([conv10, conv10_inhibition])

    mp4 = MaxPooling2D(pool_size=(2, 2))(v_conv_inhibitted)
    drop4 = Dropout(0.25)(mp4)

    flatten = Flatten()(drop4)

    dense = Dense(200, activation="relu")(flatten)
    drop5 = Dropout(0.25)(dense)

    output = Dense(numberOfOutputs, activation="softmax")(drop5)

    model = Model(inputs=inputLayer, outputs=output)

    model.summary()
    return model


# categorical
model_path = dirname(abspath(__file__)) + (
    "/../src/nicoemotionrecognition/scripts/nicoemotionrecognition/"
    + "_nicoemotionrecognition_internal/Trained Networks/Vision/"
    + "FER2013Plus_Augmented_CNN/Model/"
)

model = buildModelCategorical((1, 64, 64), 8)
optimizer = Adamax(lr=0.002, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0)
model.compile(
    loss="categorical_crossentropy",
    optimizer=optimizer,
    metrics=[
        "accuracy",
        "categorical_accuracy",
        metrics.fbeta_score,
        metrics.recall,
        metrics.precision,
    ],
)
model.load_weights(model_path + "CNN.h5")
print(model.predict(np.array([np.zeros((1, 64, 64))])))
# should be:
# array([[0.46694568, 0.11964424, 0.01778358, 0.33286154, 0.02470576,
#         0.00503258, 0.01233773, 0.02068892]], dtype=float32)
model.save(model_path + "CNN-python3.h5")
