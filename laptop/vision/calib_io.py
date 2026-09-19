"""Camera calibration files, in CALIB_DIR (default ./calib):
  <name>_intrinsics.npz : K, dist, size, rms   (once per camera, at the locked focus)
  <name>_extrinsics.npz : R, t, tag_size, rms  (every time that camera is moved; preflight checks tag_size/rms)
P = K [R|t] maps world (m) -> pixels.
"""
import os
import cv2, numpy as np


class Camera:
    def __init__(self, name, K, dist, size, R=None, t=None):
        self.name = name
        self.K, self.dist, self.size = np.asarray(K, float), np.asarray(dist, float), tuple(int(x) for x in size)
        self.R = None if R is None else np.asarray(R, float)
        self.t = None if t is None else np.asarray(t, float).reshape(3, 1)
        self.P = None if R is None else self.K @ np.hstack([self.R, self.t])

    @classmethod
    def load(cls, name, calib_dir="calib", need_extrinsics=True):
        ip = os.path.join(calib_dir, f"{name}_intrinsics.npz")
        if not os.path.exists(ip):
            raise FileNotFoundError(f"{ip} missing: run  python tools/calib/intrinsics.py --name {name} --source ...")
        i = np.load(ip)
        R = t = None
        ep = os.path.join(calib_dir, f"{name}_extrinsics.npz")
        if os.path.exists(ep):
            e = np.load(ep); R, t = e["R"], e["t"]
        elif need_extrinsics:
            raise FileNotFoundError(f"{ep} missing: run  python tools/calib/extrinsics.py --name {name} --source ...")
        return cls(name, i["K"], i["dist"], i["size"], R, t)

    @classmethod
    def nominal(cls, name, size, calib_dir=None, f_over_w=0.85, f_px=None):
        """Plausible pinhole for a WxH stream: f = 0.85 x the LONG side (a portrait stream is a rotated landscape one),
        or f_px when known (tools/calib/survey_mat.py --fit-f); centre, no distortion. Saved as <name>_intrinsics.npz
        (rms = -1 marks it nominal) when calib_dir is given."""
        w, h = int(size[0]), int(size[1])
        f = float(f_px) if f_px else f_over_w * max(w, h)
        K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], float)
        dist = np.zeros((1, 5))
        if calib_dir:
            save_intrinsics(calib_dir, name, K, dist, (w, h), -1.0)
        return cls(name, K, dist, (w, h))

    def undistort(self, uv):
        """Pixel -> undistorted pixel (still in K's pixel units). Shape (2,1) for cv2.triangulatePoints."""
        return cv2.undistortPoints(np.float32([[uv]]), self.K, self.dist, P=self.K).reshape(2, 1).astype(np.float64)

    def project(self, X):
        """World point (m) -> undistorted pixel."""
        x = self.P @ np.append(np.asarray(X, float), 1.0)
        return x[:2] / x[2]

    def position(self):
        """Camera centre in world coordinates (m)."""
        return (-self.R.T @ self.t).ravel()


TAG_DICT = cv2.aruco.DICT_APRILTAG_36h11


def tag_object_points(S):
    """The 4 corners of the floor tag in the WORLD frame (PROTOCOL.md s5): origin at the tag centre,
    +X = printed left->right, +Y = printed bottom->top, z = 0. Order = ArUco corner order (TL, TR, BR, BL
    as printed), which is also the order cv2.SOLVEPNP_IPPE_SQUARE requires. S = black-square edge in metres."""
    return np.float32([[-S / 2, S / 2, 0], [S / 2, S / 2, 0], [S / 2, -S / 2, 0], [-S / 2, -S / 2, 0]])


def make_tag_detector():
    """ArUco detector for the AprilTag 36h11 family with sub-pixel corner refinement."""
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(TAG_DICT), params)


def save_intrinsics(calib_dir, name, K, dist, size, rms):
    os.makedirs(calib_dir, exist_ok=True)
    p = os.path.join(calib_dir, f"{name}_intrinsics.npz")
    np.savez(p, K=K, dist=dist, size=np.array(size), rms=rms)
    return p


def save_extrinsics(calib_dir, name, R, t, **extra):
    """extra: tag_size (m) and rms (px) of the fit, so laptop/vision/preflight.py can catch a tag-size mismatch later.
    None values are skipped (np.savez would pickle them). Camera.load reads only R, t, so files without them still load."""
    os.makedirs(calib_dir, exist_ok=True)
    p = os.path.join(calib_dir, f"{name}_extrinsics.npz")
    np.savez(p, R=R, t=np.asarray(t).reshape(3, 1), **{k: v for k, v in extra.items() if v is not None})
    return p
