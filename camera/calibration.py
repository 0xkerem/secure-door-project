# camera/calibration.py
import cv2
import numpy as np
import glob
import os

# Settings
image_folder = 'calib_images'
pattern_size = (7, 7)  # Number of inner corners per chessboard row and column
square_size = 26.0     # Size of a square in mm

# Prepare object points based on real world coordinates
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp = objp * square_size

objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Get all images in the folder
images = glob.glob(os.path.join(image_folder, '*.jpg'))
images += glob.glob(os.path.join(image_folder, '*.jpeg'))
images += glob.glob(os.path.join(image_folder, '*.png'))

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        objpoints.append(objp)
        # Refine the corner locations
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        print(f"Chessboard corners found: {fname}")
    else:
        print(f"Chessboard corners NOT found: {fname}")

if len(objpoints) < 1:
    print("Not enough valid images for calibration!")
    exit(1)

# Camera calibration
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\nCamera Matrix (cameraMatrix):\n", cameraMatrix)
print("\nDistortion Coefficients (distCoeffs):\n", distCoeffs)

# Save the results
np.savez('calibration_result.npz', cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)
print("\nCalibration data saved to 'calibration_result.npz'.")

# -----------------------------
# USAGE EXAMPLE (UNDISTORTION)
# -----------------------------
print("""
Example: How to undistort a new image using the calibration matrix:

import cv2
import numpy as np

# Load the calibration data
data = np.load('camera/calibration_result.npz')
cameraMatrix = data['cameraMatrix']
distCoeffs = data['distCoeffs']

# Read a new image (from file or directly from camera)
img = cv2.imread('any_new_image.jpg')  # or use a frame from your camera
undistorted_img = cv2.undistort(img, cameraMatrix, distCoeffs)

cv2.imshow('Undistorted Image', undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
""")