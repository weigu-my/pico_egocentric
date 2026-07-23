#!/usr/bin/env python3
"""
Optimization-based retargeting: Pico 26-joint hand tracking -> Revo2 6-DOF hand.

Classes:
    Revo2Kinematics        - URDF FK for Revo2 hand
    PicoHandCanonicalizer  - Wrist-relative canonical frame
    FeatureExtractor       - Fingertip/direction/pinch features + q_prior
    OptimizationRetargeter - SLSQP optimizer with warm start, structured output
"""

import os
from dataclasses import dataclass, field
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize


# Active joint order (per hand): thumb_metacarpal, thumb_proximal, index, middle, ring, pinky
ACTIVE_JOINT_SUFFIXES = [
    "thumb_metacarpal_joint",
    "thumb_proximal_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
]

ACTIVE_SHORT_NAMES = [
    "thumb_metacarpal", "thumb_proximal",
    "index_proximal", "middle_proximal",
    "ring_proximal", "pinky_proximal",
]

# OpenXR 26-joint indices
PICO_TIP_INDICES = {
    "thumb": 5, "index": 10, "middle": 15, "ring": 20, "pinky": 25,
}
PICO_DIR_PAIRS = {
    "thumb": (4, 5), "index": (9, 10), "middle": (14, 15),
    "ring": (19, 20), "pinky": (24, 25),
}
# Finger chains for curl-angle estimation (used by q_prior)
PICO_FINGER_CHAINS = {
    "thumb":  [1, 2, 3, 4, 5],
    "index":  [0, 6, 7, 8, 9, 10],
    "middle": [0, 11, 12, 13, 14, 15],
    "ring":   [0, 16, 17, 18, 19, 20],
    "pinky":  [0, 21, 22, 23, 24, 25],
}

FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
SPREAD_PAIRS = (
    ("thumb", "index"),
    ("thumb", "middle"),
    ("index", "middle"),
    ("middle", "ring"),
    ("ring", "pinky"),
)

# Default weights — prioritize fingertip placement and opening/closing state.
DEFAULT_WEIGHTS = {
    "w_tip": 10.0,
    "w_spread": 6.0,
    "w_curl": 4.0,
    "w_dir": 0.3,
    "w_pinch": 5.0,
    "w_prior": 0.5,
    "w_smooth": 0.2,
    "w_limit": 0.2,
}

# Pinch detection threshold (in Pico canonical meters, pre-scale)
PINCH_DISTANCE_THRESHOLD = 0.03
PINCH_WEIGHT_BOOST = 3.0  # multiplier on w_pinch when pinching
INVALID_HAND_SPAN_THRESHOLD = 1e-3
TARGET_SIGNATURE_SMOOTH_EPS = 1e-3


@dataclass
class RetargetResult:
    """Structured output from OptimizationRetargeter.retarget()."""
    q_active: np.ndarray            # shape (6,), active joint angles
    q_active_dict: dict             # {short_name: float}
    q_full_dict: dict               # includes mimic-expanded joints
    debug_info: dict = field(default_factory=dict)


# ═══════════════════ Revo2Kinematics ═══════════════════

class Revo2Kinematics:
    """Parse Revo2 URDF and compute forward kinematics for fingertip positions."""

    def __init__(self, urdf_dir: str, side: str = "right"):
        self.side = side
        self.prefix = f"{side}_"
        urdf_path = os.path.join(urdf_dir, "urdf", f"revo2_{side}_hand.urdf")
        self._parse_urdf(urdf_path)

    def _parse_urdf(self, urdf_path: str):
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        self._joints = {}
        for j in root.findall("joint"):
            name = j.get("name")
            origin = j.find("origin")
            xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
            rpy = np.array([float(v) for v in origin.get("rpy", "0 0 0").split()])
            axis_elem = j.find("axis")
            axis = np.array([float(v) for v in axis_elem.get("xyz", "0 0 1").split()]) if axis_elem is not None else np.zeros(3)
            limit_elem = j.find("limit")
            lower = float(limit_elem.get("lower", 0)) if limit_elem is not None else 0.0
            upper = float(limit_elem.get("upper", 0)) if limit_elem is not None else 0.0
            mimic_elem = j.find("mimic")
            mimic = None
            if mimic_elem is not None:
                mimic = {
                    "joint": mimic_elem.get("joint"),
                    "multiplier": float(mimic_elem.get("multiplier", 1)),
                    "offset": float(mimic_elem.get("offset", 0)),
                }
            self._joints[name] = {
                "type": j.get("type"),
                "xyz": xyz, "rpy": rpy,
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "axis": axis, "lower": lower, "upper": upper,
                "mimic": mimic,
            }

        self.joint_names = [self.prefix + s for s in ACTIVE_JOINT_SUFFIXES]
        self.n_dof = len(self.joint_names)
        self.lower = np.array([self._joints[n]["lower"] for n in self.joint_names])
        self.upper = np.array([self._joints[n]["upper"] for n in self.joint_names])

        self._finger_chains = {}
        for finger in FINGER_NAMES:
            self._finger_chains[finger] = self._build_chain(finger)

    def _build_chain(self, finger: str):
        target_tip = f"{self.prefix}{finger}_tip_link"
        chain = []
        current_link = target_tip
        base = f"{self.prefix}base_link"
        while current_link != base:
            found = False
            for name, j in self._joints.items():
                if j["child"] == current_link:
                    chain.append(name)
                    current_link = j["parent"]
                    found = True
                    break
            if not found:
                break
        chain.reverse()
        return chain

    def _resolve_angles(self, q: np.ndarray) -> dict:
        angle_map = {}
        for i, name in enumerate(self.joint_names):
            angle_map[name] = q[i]
        for name, j in self._joints.items():
            if j["mimic"] is not None:
                parent_angle = angle_map.get(j["mimic"]["joint"], 0.0)
                angle_map[name] = parent_angle * j["mimic"]["multiplier"] + j["mimic"]["offset"]
        return angle_map

    def resolve_full_dict(self, q: np.ndarray) -> dict:
        """Return all joint angles (active + mimic) as {short_name: float}."""
        angle_map = self._resolve_angles(q)
        result = {}
        for name, angle in angle_map.items():
            j = self._joints.get(name)
            if j and j["type"] == "revolute":
                short = name.replace(self.prefix, "").replace("_joint", "")
                result[short] = float(angle)
        return result

    def forward_kinematics(self, q: np.ndarray) -> dict:
        angle_map = self._resolve_angles(q)
        tips = {}
        for finger, chain in self._finger_chains.items():
            pos = np.zeros(3)
            rot = np.eye(3)
            for jname in chain:
                j = self._joints[jname]
                R_static = Rotation.from_euler("xyz", j["rpy"]).as_matrix()
                pos = pos + rot @ j["xyz"]
                rot = rot @ R_static
                if j["type"] == "revolute":
                    theta = angle_map.get(jname, 0.0)
                    R_joint = Rotation.from_rotvec(j["axis"] * theta).as_matrix()
                    rot = rot @ R_joint
            tips[finger] = pos.copy()
        return tips

    def fingertip_directions(self, q: np.ndarray) -> dict:
        angle_map = self._resolve_angles(q)
        dirs = {}
        for finger, chain in self._finger_chains.items():
            pos = np.zeros(3)
            rot = np.eye(3)
            tip_parent_pos = None
            tip_pos = None
            for idx, jname in enumerate(chain):
                j = self._joints[jname]
                if idx == len(chain) - 1:
                    tip_parent_pos = pos.copy()
                    tip_pos = pos + rot @ j["xyz"]
                R_static = Rotation.from_euler("xyz", j["rpy"]).as_matrix()
                pos = pos + rot @ j["xyz"]
                rot = rot @ R_static
                if j["type"] == "revolute":
                    theta = angle_map.get(jname, 0.0)
                    R_joint = Rotation.from_rotvec(j["axis"] * theta).as_matrix()
                    rot = rot @ R_joint

            if tip_parent_pos is None or tip_pos is None:
                dirs[finger] = rot[:, 2].copy()
                continue

            d = tip_pos - tip_parent_pos
            n = np.linalg.norm(d)
            dirs[finger] = d / max(n, 1e-8)
        return dirs


# ═══════════════════ PicoHandCanonicalizer ═══════════════════

class PicoHandCanonicalizer:
    """
    Transform Pico 26-joint hand tracking into a wrist-relative canonical frame.

    Uses Wrist + Palm + IndexMetacarpal + LittleMetacarpal to build a robust
    palm coordinate frame. Left hand is mirrored to right-hand convention.
    """

    PALM = 0
    WRIST = 1
    INDEX_META = 6
    LITTLE_META = 21

    def canonicalize(self, positions: np.ndarray, quats: np.ndarray,
                     side: str = "right") -> np.ndarray:
        wrist = positions[self.WRIST].copy()
        palm = positions[self.PALM].copy()
        idx_meta = positions[self.INDEX_META].copy()
        lit_meta = positions[self.LITTLE_META].copy()

        # Z-axis: wrist -> palm (forward direction along hand)
        z_axis = palm - wrist
        z_norm = np.linalg.norm(z_axis)
        if z_norm < 1e-8:
            # Fallback: use wrist quaternion
            R_wrist = Rotation.from_quat(quats[self.WRIST]).as_matrix()
            z_axis = R_wrist[:, 2]
        else:
            z_axis /= z_norm

        # Y-axis: cross-palm direction (little_meta -> index_meta), orthogonalized to z
        y_cand = idx_meta - lit_meta
        y_axis = y_cand - np.dot(y_cand, z_axis) * z_axis
        y_norm = np.linalg.norm(y_axis)
        if y_norm < 1e-8:
            # Second fallback: wrist quaternion
            R_wrist = Rotation.from_quat(quats[self.WRIST]).as_matrix()
            y_axis = R_wrist[:, 1]
            y_axis = y_axis - np.dot(y_axis, z_axis) * z_axis
            y_norm = np.linalg.norm(y_axis)
        if y_norm < 1e-8:
            # Degenerate — return identity-transformed positions
            return positions - wrist[np.newaxis, :]
        y_axis /= y_norm

        x_axis = np.cross(y_axis, z_axis)
        x_norm = np.linalg.norm(x_axis)
        if x_norm < 1e-8:
            return positions - wrist[np.newaxis, :]
        x_axis /= x_norm

        R = np.column_stack([x_axis, y_axis, z_axis])
        centered = positions - wrist[np.newaxis, :]
        canonical = (R.T @ centered.T).T

        if side == "left":
            canonical[:, 0] *= -1

        return canonical


# ═══════════════════ FeatureExtractor ═══════════════════

class FeatureExtractor:
    """Extract features from Pico canonical hand and Revo2 FK, including q_prior."""

    def _compute_curl(self, positions: np.ndarray, chain) -> float:
        if len(chain) < 3:
            return 0.0
        v1 = positions[chain[2]] - positions[chain[1]]
        v2 = positions[chain[-1]] - positions[chain[2]]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-8 or n2 < 1e-8:
            return 0.0
        # Straight fingers should map to curl ~= 0, and flexion should increase curl.
        return np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1, 1))

    def compute_spread_distances(self, tips: dict) -> dict:
        spread = {}
        for a, b in SPREAD_PAIRS:
            spread[f"{a}_{b}"] = float(np.linalg.norm(tips[a] - tips[b]))
        return spread

    def compute_pico_curls(self, canonical: np.ndarray) -> dict:
        max_curl = np.pi * 0.8
        curls = {}
        for finger, chain in PICO_FINGER_CHAINS.items():
            curl = self._compute_curl(canonical, chain)
            curls[finger] = float(np.clip(curl / max_curl, 0.0, 1.0))
        return curls

    def compute_revo2_curls(self, q: np.ndarray, kin) -> dict:
        return {
            "thumb": float(np.clip(q[1] / max(kin.upper[1], 1e-8), 0.0, 1.0)),
            "index": float(np.clip(q[2] / max(kin.upper[2], 1e-8), 0.0, 1.0)),
            "middle": float(np.clip(q[3] / max(kin.upper[3], 1e-8), 0.0, 1.0)),
            "ring": float(np.clip(q[4] / max(kin.upper[4], 1e-8), 0.0, 1.0)),
            "pinky": float(np.clip(q[5] / max(kin.upper[5], 1e-8), 0.0, 1.0)),
        }

    def from_pico(self, canonical: np.ndarray):
        """
        Returns:
            tips: dict finger -> (3,)
            dirs: dict finger -> (3,) unit
            pinch_thumb_index: float
            pinch_thumb_middle: float
            spread: dict pair -> distance
            curls: dict finger -> normalized curl [0, 1]
        """
        tips = {f: canonical[i].copy() for f, i in PICO_TIP_INDICES.items()}
        dirs = {}
        for finger, (i_from, i_to) in PICO_DIR_PAIRS.items():
            d = canonical[i_to] - canonical[i_from]
            n = np.linalg.norm(d)
            dirs[finger] = d / max(n, 1e-8)

        pinch_ti = float(np.linalg.norm(canonical[5] - canonical[10]))
        pinch_tm = float(np.linalg.norm(canonical[5] - canonical[15]))
        spread = self.compute_spread_distances(tips)
        curls = self.compute_pico_curls(canonical)
        return tips, dirs, pinch_ti, pinch_tm, spread, curls

    def from_revo2(self, tips_fk: dict, dirs_fk: dict):
        """
        Returns:
            tips: dict finger -> (3,)
            dirs: dict finger -> (3,) unit
            pinch_thumb_index: float
            pinch_thumb_middle: float
            spread: dict pair -> distance
        """
        tips = {}
        dirs = {}
        for finger in FINGER_NAMES:
            tips[finger] = tips_fk[finger].copy()
            d = dirs_fk[finger].copy()
            n = np.linalg.norm(d)
            dirs[finger] = d / max(n, 1e-8)

        pinch_ti = float(np.linalg.norm(tips["thumb"] - tips["index"]))
        pinch_tm = float(np.linalg.norm(tips["thumb"] - tips["middle"]))
        spread = self.compute_spread_distances(tips)
        return tips, dirs, pinch_ti, pinch_tm, spread

    def compute_q_prior(self, canonical: np.ndarray, lower: np.ndarray,
                        upper: np.ndarray) -> np.ndarray:
        """
        Estimate a geometric prior q from Pico canonical positions using curl angles.
        Maps each finger curl to the corresponding Revo2 joint range.

        Returns: (6,) array clipped to [lower, upper]
        """
        curls = {f: self._compute_curl(canonical, ch) for f, ch in PICO_FINGER_CHAINS.items()}

        td = canonical[3] - canonical[2]
        idx_d = canonical[7] - canonical[6]
        n1, n2 = np.linalg.norm(td), np.linalg.norm(idx_d)
        if n1 > 1e-8 and n2 > 1e-8:
            abd = np.arccos(np.clip(np.dot(td, idx_d) / (n1 * n2), -1, 1))
        else:
            abd = 0.0

        max_curl = np.pi * 0.8

        def _map(curl, lo, hi):
            return lo + np.clip(curl / max_curl, 0, 1) * (hi - lo)

        q_prior = np.array([
            np.clip(abd, lower[0], upper[0]),
            _map(curls["thumb"], lower[1], upper[1]),
            _map(curls["index"], lower[2], upper[2]),
            _map(curls["middle"], lower[3], upper[3]),
            _map(curls["ring"], lower[4], upper[4]),
            _map(curls["pinky"], lower[5], upper[5]),
        ])
        return q_prior


# ═══════════════════ OptimizationRetargeter ═══════════════════

class OptimizationRetargeter:
    """
    SLSQP-based retargeting from Pico hand tracking to Revo2 6-DOF.

    Objective: min_q  w_tip * L_tip + w_spread * L_spread + w_curl * L_curl
                     + w_dir * L_dir + w_pinch * L_pinch + w_prior * L_prior
                     + w_smooth * L_smooth + w_limit * L_limit

    The optimizer prioritizes fingertip placement plus hand opening/closing cues,
    while still using q_prior and temporal warm start for stability.
    """

    def __init__(self, kin: Revo2Kinematics, weights: dict = None):
        self.kin = kin
        self.canon = PicoHandCanonicalizer()
        self.feat = FeatureExtractor()
        self.weights = dict(DEFAULT_WEIGHTS)
        if weights:
            self.weights.update(weights)
        self.prev_q = None
        self.prev_target_signature = None
        self._scale = None
        self._limit_margin = 0.05
        self._has_valid_prev = False
        self._open_q = np.zeros(self.kin.n_dof)
        self._thumb_prior_lut = self._build_thumb_prior_lut()

    def _build_thumb_prior_lut(self, meta_samples: int = 17,
                               flex_samples: int = 13) -> dict:
        meta_grid = np.linspace(self.kin.lower[0], self.kin.upper[0], meta_samples)
        flex_grid = np.linspace(self.kin.lower[1], self.kin.upper[1], flex_samples)
        dirs = []
        qs = []
        for q0 in meta_grid:
            for q1 in flex_grid:
                q = self._open_q.copy()
                q[0] = q0
                q[1] = q1
                d = self.kin.fingertip_directions(q)["thumb"]
                n = np.linalg.norm(d)
                dirs.append(d / max(n, 1e-8))
                qs.append([q0, q1])
        return {
            "dirs": np.asarray(dirs, dtype=float),
            "qs": np.asarray(qs, dtype=float),
        }

    def _estimate_thumb_prior(self, canonical: np.ndarray) -> np.ndarray:
        thumb_dir = canonical[5] - canonical[4]
        if np.linalg.norm(thumb_dir) < 1e-8:
            thumb_dir = canonical[5] - canonical[2]
        n = np.linalg.norm(thumb_dir)
        if n < 1e-8:
            return np.array([0.0, 0.0], dtype=float)
        target = thumb_dir / n

        scores = self._thumb_prior_lut["dirs"] @ target
        topk = min(4, len(scores))
        idx = np.argsort(scores)[-topk:]
        weights = np.clip(scores[idx], 0.0, None)
        if np.sum(weights) < 1e-8:
            best = int(np.argmax(scores))
            return self._thumb_prior_lut["qs"][best].copy()
        weights = weights / np.sum(weights)
        return np.sum(self._thumb_prior_lut["qs"][idx] * weights[:, None], axis=0)

    def _compute_scale(self, pico_tips: dict, revo2_tips_zero: dict) -> float:
        pico_span = 0.0
        revo2_span = 0.0
        for finger in ("index", "middle", "ring", "pinky"):
            if finger in pico_tips and finger in revo2_tips_zero:
                pico_span += np.linalg.norm(pico_tips[finger])
                revo2_span += np.linalg.norm(revo2_tips_zero[finger])
        if pico_span < 1e-8:
            return 1.0
        return revo2_span / pico_span

    def _is_valid_observation(self, positions: np.ndarray, quats: np.ndarray) -> bool:
        positions = np.asarray(positions)
        quats = np.asarray(quats)
        if positions.shape != (26, 3) or quats.shape != (26, 4):
            return False
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(quats)):
            return False
        span = float(np.linalg.norm(np.max(positions, axis=0) - np.min(positions, axis=0)))
        return span >= INVALID_HAND_SPAN_THRESHOLD

    def _build_target_signature(self, pico_tips: dict, pico_spread: dict,
                                pico_curls: dict, pico_pinch_ti: float,
                                pico_pinch_tm: float) -> np.ndarray:
        parts = []
        for finger in FINGER_NAMES:
            parts.extend(np.asarray(pico_tips[finger], dtype=float).tolist())
        for key in sorted(pico_spread):
            parts.append(float(pico_spread[key]))
        for finger in FINGER_NAMES:
            parts.append(float(pico_curls[finger]))
        parts.extend([float(pico_pinch_ti), float(pico_pinch_tm)])
        return np.asarray(parts, dtype=float)

    def _evaluate_robot_features(self, q: np.ndarray):
        revo2_tips_fk = self.kin.forward_kinematics(q)
        revo2_dirs_fk = self.kin.fingertip_directions(q)
        revo2_tips, revo2_dirs, revo2_pinch_ti, revo2_pinch_tm, revo2_spread = self.feat.from_revo2(
            revo2_tips_fk, revo2_dirs_fk)
        robot_curls = self.feat.compute_revo2_curls(q, self.kin)
        return (
            revo2_tips,
            revo2_dirs,
            revo2_pinch_ti,
            revo2_pinch_tm,
            revo2_spread,
            robot_curls,
        )

    def _objective(self, q, pico_tips, pico_dirs, pico_pinch_ti, pico_pinch_tm,
                   pico_spread, pico_curls, scale, q_prior, prev_q,
                   w_pinch_eff, w_smooth_eff):
        revo2_tips, revo2_dirs, revo2_pinch_ti, revo2_pinch_tm, revo2_spread, robot_curls =             self._evaluate_robot_features(q)

        w = self.weights

        L_tip = sum(np.dot(scale * pico_tips[f] - revo2_tips[f],
                           scale * pico_tips[f] - revo2_tips[f])
                    for f in FINGER_NAMES)
        L_spread = sum((scale * pico_spread[k] - revo2_spread[k]) ** 2 for k in pico_spread)
        L_curl = sum((pico_curls[f] - robot_curls[f]) ** 2 for f in FINGER_NAMES)
        L_dir = sum(1.0 - np.dot(pico_dirs[f], revo2_dirs[f]) for f in FINGER_NAMES)
        L_pinch = ((scale * pico_pinch_ti - revo2_pinch_ti) ** 2
                   + (scale * pico_pinch_tm - revo2_pinch_tm) ** 2)

        dq_prior = q - q_prior
        L_prior = np.dot(dq_prior, dq_prior)

        dq_smooth = q - prev_q
        L_smooth = np.dot(dq_smooth, dq_smooth)

        L_limit = 0.0
        margin = self._limit_margin
        for i in range(self.kin.n_dof):
            over = q[i] - (self.kin.upper[i] - margin)
            if over > 0:
                L_limit += over * over
            under = (self.kin.lower[i] + margin) - q[i]
            if under > 0:
                L_limit += under * under

        total = (
            w["w_tip"] * L_tip
            + w["w_spread"] * L_spread
            + w["w_curl"] * L_curl
            + w["w_dir"] * L_dir
            + w_pinch_eff * L_pinch
            + w["w_prior"] * L_prior
            + w_smooth_eff * L_smooth
            + w["w_limit"] * L_limit
        )
        return float(total)

    def _compute_loss_terms(self, q, pico_tips, pico_dirs, pico_pinch_ti,
                            pico_pinch_tm, pico_spread, pico_curls, scale,
                            q_prior, prev_q, w_pinch_eff, w_smooth_eff):
        """Compute individual loss terms for debug_info."""
        revo2_tips, revo2_dirs, revo2_pinch_ti, revo2_pinch_tm, revo2_spread, robot_curls =             self._evaluate_robot_features(q)

        L_tip = sum(np.dot(scale * pico_tips[f] - revo2_tips[f],
                           scale * pico_tips[f] - revo2_tips[f])
                    for f in FINGER_NAMES)
        L_spread = sum((scale * pico_spread[k] - revo2_spread[k]) ** 2 for k in pico_spread)
        L_curl = sum((pico_curls[f] - robot_curls[f]) ** 2 for f in FINGER_NAMES)
        L_dir = sum(1.0 - np.dot(pico_dirs[f], revo2_dirs[f]) for f in FINGER_NAMES)
        L_pinch = ((scale * pico_pinch_ti - revo2_pinch_ti) ** 2
                   + (scale * pico_pinch_tm - revo2_pinch_tm) ** 2)
        dq_prior = q - q_prior
        L_prior = float(np.dot(dq_prior, dq_prior))
        dq_smooth = q - prev_q
        L_smooth = float(np.dot(dq_smooth, dq_smooth))

        L_limit = 0.0
        margin = self._limit_margin
        for i in range(self.kin.n_dof):
            over = q[i] - (self.kin.upper[i] - margin)
            if over > 0:
                L_limit += over * over
            under = (self.kin.lower[i] + margin) - q[i]
            if under > 0:
                L_limit += under * under

        w = self.weights
        return {
            "L_tip": float(L_tip),
            "L_spread": float(L_spread),
            "L_curl": float(L_curl),
            "L_dir": float(L_dir),
            "L_pinch": float(L_pinch),
            "L_prior": L_prior,
            "L_smooth": L_smooth,
            "L_limit": float(L_limit),
            "w_smooth_effective": float(w_smooth_eff),
            "weighted_total": float(
                w["w_tip"] * L_tip
                + w["w_spread"] * L_spread
                + w["w_curl"] * L_curl
                + w["w_dir"] * L_dir
                + w_pinch_eff * L_pinch
                + w["w_prior"] * L_prior
                + w_smooth_eff * L_smooth
                + w["w_limit"] * L_limit
            ),
        }

    def _build_result(self, q_opt: np.ndarray, q_prior: np.ndarray,
                      target_tips: dict, target_dirs: dict,
                      target_pinch: dict, target_spread: dict,
                      target_curls: dict, scale: float,
                      solver_success: bool, solver_message: str,
                      iterations: int, fallback_used: bool,
                      w_pinch_eff: float, w_smooth_eff: float,
                      target_delta: float, prev_q_ref: np.ndarray) -> RetargetResult:
        q_opt = np.asarray(q_opt, dtype=float)
        q_active_dict = {ACTIVE_SHORT_NAMES[i]: float(q_opt[i]) for i in range(6)}
        q_full_dict = self.kin.resolve_full_dict(q_opt)

        robot_tips, robot_dirs, robot_pinch_ti, robot_pinch_tm, robot_spread, robot_curls =             self._evaluate_robot_features(q_opt)

        if target_tips:
            loss_terms = self._compute_loss_terms(
                q_opt,
                target_tips,
                target_dirs,
                target_pinch["thumb_index"],
                target_pinch["thumb_middle"],
                target_spread,
                target_curls,
                scale,
                q_prior,
                prev_q_ref,
                w_pinch_eff,
                w_smooth_eff,
            )
        else:
            loss_terms = {}

        debug_info = {
            "q_prior": q_prior.copy(),
            "target_tip_positions": {f: target_tips[f].tolist() for f in FINGER_NAMES} if target_tips else {},
            "robot_tip_positions": {f: robot_tips[f].tolist() for f in FINGER_NAMES},
            "target_dirs": {f: target_dirs[f].tolist() for f in FINGER_NAMES} if target_dirs else {},
            "robot_dirs": {f: robot_dirs[f].tolist() for f in FINGER_NAMES},
            "target_pinch": dict(target_pinch),
            "robot_pinch": {"thumb_index": robot_pinch_ti, "thumb_middle": robot_pinch_tm},
            "target_spread": dict(target_spread),
            "robot_spread": dict(robot_spread),
            "target_curls": dict(target_curls),
            "robot_curls": dict(robot_curls),
            "loss_terms": loss_terms,
            "solver_success": bool(solver_success),
            "solver_message": str(solver_message),
            "iterations": int(iterations),
            "scale": float(scale),
            "fallback_used": bool(fallback_used),
            "w_pinch_effective": float(w_pinch_eff),
            "w_smooth_effective": float(w_smooth_eff),
            "target_delta": float(target_delta),
        }

        return RetargetResult(
            q_active=q_opt,
            q_active_dict=q_active_dict,
            q_full_dict=q_full_dict,
            debug_info=debug_info,
        )

    def retarget(self, positions: np.ndarray, quats: np.ndarray,
                 side: str = None) -> RetargetResult:
        """
        Retarget Pico hand data to Revo2 joint angles.

        Returns:
            RetargetResult with q_active, q_active_dict, q_full_dict, debug_info
        """
        if side is None:
            side = self.kin.side

        if not self._is_valid_observation(positions, quats):
            if self._has_valid_prev and self.prev_q is not None:
                q_opt = self.prev_q.copy()
                q_prior = q_opt.copy()
                return self._build_result(
                    q_opt=q_opt,
                    q_prior=q_prior,
                    target_tips={},
                    target_dirs={},
                    target_pinch={"thumb_index": 0.0, "thumb_middle": 0.0},
                    target_spread={},
                    target_curls={},
                    scale=self._scale if self._scale is not None else 1.0,
                    solver_success=False,
                    solver_message="invalid_observation_hold_last",
                    iterations=0,
                    fallback_used=True,
                    w_pinch_eff=0.0,
                    w_smooth_eff=0.0,
                    target_delta=0.0,
                    prev_q_ref=q_opt.copy(),
                )

            q_opt = self._open_q.copy()
            q_prior = q_opt.copy()
            return self._build_result(
                q_opt=q_opt,
                q_prior=q_prior,
                target_tips={},
                target_dirs={},
                target_pinch={"thumb_index": 0.0, "thumb_middle": 0.0},
                target_spread={},
                target_curls={},
                scale=1.0,
                solver_success=False,
                solver_message="invalid_observation_open_hand",
                iterations=0,
                fallback_used=True,
                w_pinch_eff=0.0,
                w_smooth_eff=0.0,
                target_delta=0.0,
                prev_q_ref=q_opt.copy(),
            )

        canonical = self.canon.canonicalize(positions, quats, side)
        pico_tips, pico_dirs, pico_pinch_ti, pico_pinch_tm, pico_spread, pico_curls = self.feat.from_pico(canonical)

        if self._scale is None:
            revo2_tips_zero = self.kin.forward_kinematics(self._open_q)
            self._scale = self._compute_scale(pico_tips, revo2_tips_zero)
        scale = self._scale

        q_prior = self.feat.compute_q_prior(canonical, self.kin.lower, self.kin.upper)
        q_prior[:2] = np.clip(self._estimate_thumb_prior(canonical), self.kin.lower[:2], self.kin.upper[:2])
        pico_curls = dict(pico_curls)
        pico_curls["thumb"] = float(np.clip(q_prior[1] / max(self.kin.upper[1], 1e-8), 0.0, 1.0))

        target_signature = self._build_target_signature(
            pico_tips, pico_spread, pico_curls, pico_pinch_ti, pico_pinch_tm)

        if self._has_valid_prev and self.prev_target_signature is not None:
            target_delta = float(np.linalg.norm(target_signature - self.prev_target_signature))
        else:
            target_delta = float("inf")

        w_pinch_eff = self.weights["w_pinch"]
        min_pinch = min(pico_pinch_ti, pico_pinch_tm)
        if min_pinch < PINCH_DISTANCE_THRESHOLD:
            w_pinch_eff *= PINCH_WEIGHT_BOOST

        if self._has_valid_prev and self.prev_q is not None:
            x0 = self.prev_q.copy()
            prev_q_ref = self.prev_q.copy()
            w_smooth_eff = 0.0 if target_delta < TARGET_SIGNATURE_SMOOTH_EPS else self.weights["w_smooth"]
        else:
            x0 = q_prior.copy()
            prev_q_ref = q_prior.copy()
            w_smooth_eff = 0.0

        if self._has_valid_prev and target_delta < TARGET_SIGNATURE_SMOOTH_EPS:
            self.prev_target_signature = target_signature.copy()
            return self._build_result(
                q_opt=self.prev_q.copy(),
                q_prior=q_prior,
                target_tips=pico_tips,
                target_dirs=pico_dirs,
                target_pinch={"thumb_index": pico_pinch_ti, "thumb_middle": pico_pinch_tm},
                target_spread=pico_spread,
                target_curls=pico_curls,
                scale=scale,
                solver_success=True,
                solver_message="target_hold_previous",
                iterations=0,
                fallback_used=False,
                w_pinch_eff=w_pinch_eff,
                w_smooth_eff=0.0,
                target_delta=target_delta,
                prev_q_ref=prev_q_ref,
            )

        bounds = list(zip(self.kin.lower, self.kin.upper))
        result = minimize(
            self._objective,
            x0=x0,
            args=(
                pico_tips,
                pico_dirs,
                pico_pinch_ti,
                pico_pinch_tm,
                pico_spread,
                pico_curls,
                scale,
                q_prior,
                prev_q_ref,
                w_pinch_eff,
                w_smooth_eff,
            ),
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": 15, "ftol": 1e-6},
        )

        fallback_used = False
        solver_success = bool(result.success)
        solver_message = str(result.message)
        iterations = int(getattr(result, "nit", 0))

        if solver_success:
            q_opt = np.clip(result.x, self.kin.lower, self.kin.upper)
            self.prev_q = q_opt.copy()
            self.prev_target_signature = target_signature.copy()
            self._has_valid_prev = True
        else:
            fallback_used = True
            if self._has_valid_prev and self.prev_q is not None:
                q_opt = self.prev_q.copy()
            else:
                q_opt = q_prior.copy()
                self.prev_q = q_opt.copy()
                self.prev_target_signature = target_signature.copy()
                self._has_valid_prev = True

        return self._build_result(
            q_opt=q_opt,
            q_prior=q_prior,
            target_tips=pico_tips,
            target_dirs=pico_dirs,
            target_pinch={"thumb_index": pico_pinch_ti, "thumb_middle": pico_pinch_tm},
            target_spread=pico_spread,
            target_curls=pico_curls,
            scale=scale,
            solver_success=solver_success,
            solver_message=solver_message,
            iterations=iterations,
            fallback_used=fallback_used,
            w_pinch_eff=w_pinch_eff,
            w_smooth_eff=w_smooth_eff,
            target_delta=target_delta,
            prev_q_ref=prev_q_ref,
        )

    def reset(self):
        self.prev_q = None
        self.prev_target_signature = None
        self._has_valid_prev = False
        self._scale = None


def create_retargeter(urdf_dir: str, side: str = "right",
                      weights: dict = None) -> OptimizationRetargeter:
    """Factory: create a ready-to-use OptimizationRetargeter."""
    kin = Revo2Kinematics(urdf_dir, side)
    return OptimizationRetargeter(kin, weights)
