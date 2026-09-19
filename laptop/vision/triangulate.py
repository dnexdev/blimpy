"""Two pixel observations -> one world point."""
import cv2, numpy as np


def triangulate(camA, camB, uvA, uvB):
    """Returns (X world (3,), max reprojection error in px). Big error = the two cameras disagree."""
    X = cv2.triangulatePoints(camA.P, camB.P, camA.undistort(uvA), camB.undistort(uvB))
    X = (X[:3] / X[3]).ravel()
    err = max(float(np.linalg.norm(cam.project(X) - cam.undistort(uv).ravel()))
              for cam, uv in ((camA, uvA), (camB, uvB)))
    return X, err
