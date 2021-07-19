import cv2
import numpy as np
import unittest

from PIL import Image, ImageDraw

from nicoface.FaceExpression import faceExpression


class FaceTest(unittest.TestCase):
    def setUp(self):
        self.face = faceExpression(simulation=True)

    def tearDown(self):
        del self.face
        cv2.destroyAllWindows()

    def test_bitmap(self):
        """
        Tests if send_bitmap_face properly generates images
        """
        # bitmap definitions
        brow_left = np.array(
            [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 1, 0, 0, 0],
                [0, 1, 1, 0, 0, 1, 1, 0],
                [1, 0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
            ],
            dtype="uint8",
        )

        brow_right = np.array(
            [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 1, 0, 0, 0],
                [0, 1, 1, 0, 0, 1, 1, 0],
                [1, 0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
            ],
            dtype="uint8",
        )

        mouth = np.array(
            [
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                [0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
                [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
            ],
            dtype="uint8",
        )
        # generate face
        self.face.send_bitmap_face(brow_left, brow_right, mouth)
        # check if left brow image reflects bitmap
        np.testing.assert_array_equal(brow_left, np.transpose(self.face.left) / 255.0)
        # check if right brow image reflects bitmap
        np.testing.assert_array_equal(brow_right, np.transpose(self.face.right) / 255.0)
        # check if mouth image reflects bitmap
        np.testing.assert_array_equal(
            mouth, np.transpose(self.face.mouth)[::-1] / 255.0
        )

    def test_wavelet_mouth(self):
        """
        test gen_mouth
        """
        # generate expected mouth images
        expected = Image.new("L", (16, 8))
        expected = self.face.draw_wavelet(2, -1.4, 0.7, 1.0, 0, expected)
        # generate wavelet image
        self.face.gen_mouth()
        # check if mouth matches
        np.testing.assert_array_equal(
            np.transpose(expected)[::-1], np.array(self.face.mouth)
        )

    def test_wavelet_eyebrow_l(self):
        """
        test gen_eyebrowse for left
        """
        # genereate expected image
        expected = Image.new("L", (8, 8))
        expected = self.face.draw_wavelet(2, 0.1, 0.4, 1, -0.55, expected)
        # generate left eyebrow
        self.face.gen_eyebrowse()
        # check if left eyebrow matches
        np.testing.assert_array_equal(
            np.array(expected), np.transpose(self.face.left)[::-1]
        )

    def test_wavelet_eyebrow_r(self):
        """
        test gen_eyebrowse for right
        """
        # generate expected image
        expected = Image.new("L", (8, 8))
        expected = self.face.draw_wavelet(2, -0.1, -0.4, 1, -0.55, expected)
        # generate right eyebrow
        self.face.gen_eyebrowse(type="r")
        # check if right eyebrow matches
        np.testing.assert_array_equal(np.array(expected), np.transpose(self.face.right))

    def test_wavelet_face(self):
        """check if setting full wavelet face works properly"""
        # expected images
        expected_m = Image.new("L", (16, 8))
        expected_m = self.face.draw_wavelet(2, -1.4, 0.7, 1.0, 0, expected_m)
        expected_l = Image.new("L", (8, 8))
        expected_l = self.face.draw_wavelet(2, 0.1, 0.4, 1, -0.55, expected_l)
        expected_r = Image.new("L", (8, 8))
        expected_r = self.face.draw_wavelet(2, -0.1, -0.4, 1, -0.55, expected_r)
        # generate wavelet face
        self.face.send_wavelet_face(
            (-1.4, 0.7, 1.0, 0), [None] * 4, (0.1, 0.4, 1, -0.55), (0.1, 0.4, 1, -0.55),
        )
        # check if all images match expectation
        np.testing.assert_array_equal(
            np.transpose(expected_m)[::-1], np.array(self.face.mouth)
        )
        np.testing.assert_array_equal(
            np.array(expected_l), np.transpose(self.face.left)[::-1]
        )
        np.testing.assert_array_equal(
            np.array(expected_r), np.transpose(self.face.right)
        )

    def test_morph_missmatch_1(self):
        # set base function to polynomial
        self.face.send_morphable_face_expression("happiness")
        # try morphing to wavelet
        with self.assertRaises(ValueError):
            self.face.morph_wavelet_face(
                (-1.4, 0.7, 1.0, 0),
                (-1.4, 0.7, 1.0, 0),
                (0.1, 0.4, 1, -0.55),
                (0.1, 0.4, 1, -0.55),
            )

    def test_morph_missmatch_2(self):
        # set base function to wavelet
        self.face.sendTrainedFaceExpression("happiness")
        # try morphing to polynomial
        with self.assertRaises(ValueError):
            self.face.morph_polynomial_face(
                [7, 0, -0.11, 0, 0],
                [7, 0, -0.11, 0, 0],
                7.45,
                2,
                2,
                [4.75, -0.25, 0, 0, 0],
                0,
                0,
                0,
                [3, 0.25, 0, 0, 0],
                0,
                0,
                0,
            )

    def test_wavelet_morph(self):
        """check if morph wavelet face works properly"""
        # make sure wavelet is set
        self.face.sendTrainedFaceExpression("happiness")
        # expected images
        expected_m = Image.new("L", (16, 8))
        expected_m = self.face.draw_wavelet(2, -1.4, 0.7, 1.0, 0, expected_m)
        expected_l = Image.new("L", (8, 8))
        expected_l = self.face.draw_wavelet(2, 0.1, 0.4, 1, -0.55, expected_l)
        expected_r = Image.new("L", (8, 8))
        expected_r = self.face.draw_wavelet(2, -0.1, -0.4, 1, -0.55, expected_r)
        # generate wavelet face
        self.face.morph_wavelet_face(
            (-1.4, 0.7, 1.0, 0),
            (-1.4, 0.7, 1.0, 0),
            (0.1, 0.4, 1, -0.55),
            (0.1, 0.4, 1, -0.55),
        )
        # check if all images match expectation
        np.testing.assert_array_equal(
            np.transpose(expected_m)[::-1], np.array(self.face.mouth)
        )
        np.testing.assert_array_equal(
            np.array(expected_l), np.transpose(self.face.left)[::-1]
        )
        np.testing.assert_array_equal(
            np.array(expected_r), np.transpose(self.face.right)
        )

    def test_polynomial_0st_deg(self):
        polynomial = self.face.polynomial(np.arange(10), [5])
        for i in range(10):
            assert polynomial[i] == 5

    def test_polynomial_1nd_deg(self):
        polynomial = self.face.polynomial(np.arange(10), [0, 2])
        for i in range(10):
            assert polynomial[i] == i * 2

    def test_polynomial_2nd_deg(self):
        polynomial = self.face.polynomial(np.arange(10), [0, 0, 4])
        for i in range(10):
            assert polynomial[i] == 4 * i ** 2

    def test_polynomial_multiple_degs(self):
        polynomial = self.face.polynomial(np.arange(10), [6, 3, 2])
        for i in range(10):
            assert polynomial[i] == 6 + 3 * i + 2 * i ** 2

    def test_polynomial_mouth(self):
        pass  # TODO think of way to test without copy pasting the full method

    def test_polynomial_eyebrow(self):
        pass  # TODO think of way to test without copy pasting the full method

    def test_polynomial_face(self):
        # send face and save expected
        self.face.generate_polynomial_mouth(
            [7, 0, -0.11, 0, 0], [7, 0, -0.11, 0, 0], 7.45, 2, 2
        )
        self.face.generate_polynomial_eyebrow([4.75, -0.25, 0, 0, 0], 0, 0, 0)
        self.face.generate_polynomial_eyebrow([3, 0.25, 0, 0, 0], 0, 0, 0, left=False)
        expected_l = self.face.left
        expected_r = self.face.right
        expected_m = self.face.mouth
        # reset expression
        self.face.send_morphable_face_expression("neutral")
        # morph
        self.face.send_polynomial_face(
            [7, 0, -0.11, 0, 0],
            [7, 0, -0.11, 0, 0],
            7.45,
            2,
            2,
            [4.75, -0.25, 0, 0, 0],
            0,
            0,
            0,
            [3, 0.25, 0, 0, 0],
            0,
            0,
            0,
        )
        # compare
        np.testing.assert_array_equal(np.array(expected_m), np.array(self.face.mouth))
        np.testing.assert_array_equal(np.array(expected_l), np.array(self.face.left))
        np.testing.assert_array_equal(np.array(expected_r), np.array(self.face.right))

    def test_polynomial_morph(self):
        # send face and save expected
        self.face.send_polynomial_face(
            [7, 0, -0.11, 0, 0],
            [7, 0, -0.11, 0, 0],
            7.45,
            2,
            2,
            [4.75, -0.25, 0, 0, 0],
            0,
            0,
            0,
            [3, 0.25, 0, 0, 0],
            0,
            0,
            0,
        )
        expected_l = self.face.left
        expected_r = self.face.right
        expected_m = self.face.mouth
        # reset expression
        self.face.send_morphable_face_expression("neutral")
        # morph
        self.face.morph_polynomial_face(
            [7, 0, -0.11, 0, 0],
            [7, 0, -0.11, 0, 0],
            7.45,
            2,
            2,
            [4.75, -0.25, 0, 0, 0],
            0,
            0,
            0,
            [3, 0.25, 0, 0, 0],
            0,
            0,
            0,
        )
        # compare
        np.testing.assert_array_equal(np.array(expected_m), np.array(self.face.mouth))
        np.testing.assert_array_equal(np.array(expected_l), np.array(self.face.left))
        np.testing.assert_array_equal(np.array(expected_r), np.array(self.face.right))


if __name__ == "__main__":
    unittest.main()
