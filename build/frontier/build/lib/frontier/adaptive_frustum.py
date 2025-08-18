#!/usr/bin/env python3
"""
Adaptive frustum near/far estimation from a single image bbox using
simple pinhole geometry and rule-based uncertainty.

- Assumes approximately known object dimensions (default 0.3m base, 0.7m height)
- Estimates range Z ≈ f * H / h_pixels (and width-based alternative)
- Expands to near/far with margins that grow for small bboxes
- Discretizes into 2-5 tiers based on bbox area ratio, to control band length
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

from yolo_msgs.msg import BoundingBox2D

from .calibration_loader import CameraCalibration


@dataclass
class AdaptiveFrustumConfig:
    enabled: bool = True
    object_height_m: float = 0.70
    object_width_m: float = 0.30
    near_min_m: float = 0.5
    far_max_m: float = 60.0
    # Fine mode (continuous) parameters
    margin_min: float = 0.15  # for largest boxes
    margin_max: float = 0.80  # for tiniest boxes
    # Area thresholds (fraction of image area) splitting to 2/3/4/5 tiers
    # area >= t0 -> 2 tiers (narrow); t1<=area<t0 -> 3; t2<=area<t1 -> 4; <t2 -> 5
    area_thresholds: Tuple[float, float, float] = (0.10, 0.05, 0.02)
    # Coarseness control
    mode: str = "fine"  # "fine" or "coarse"
    # Coarse mode uses 3 bins: large/medium/small
    coarse_thresholds: Tuple[float, float] = (0.06, 0.02)  # t_large, t_medium
    coarse_margins: Tuple[float, float, float] = (0.35, 0.70, 1.10)


class AdaptiveFrustumEstimator:
    """Estimate near/far distances for a frustum from a 2D bbox.

    The estimator uses camera intrinsics and nominal object dimensions to
    compute a coarse depth, then expands to near/far using a rule-based
    margin depending on bbox area fraction.
    """

    def __init__(self, calibration: CameraCalibration, config: Optional[AdaptiveFrustumConfig] = None):
        self.calibration = calibration
        self.cfg = config or AdaptiveFrustumConfig()

        # Cache intrinsics
        self.fx = float(self.calibration.camera_matrix[0, 0])
        self.fy = float(self.calibration.camera_matrix[1, 1])
        self.image_width = int(getattr(self.calibration, 'image_width', 640))
        self.image_height = int(getattr(self.calibration, 'image_height', 480))
        self.image_area = float(self.image_width * self.image_height)

    def compute_near_far(self, bbox: BoundingBox2D) -> Tuple[float, float]:
        """Return (near_m, far_m) distances along the viewing ray.

        Robustly combine height/width-based depth estimates and inflate a
        symmetric margin based on bbox area. Clamp to configured bounds.
        """
        if bbox.size.x <= 0 or bbox.size.y <= 0:
            # Degenerate; return conservative default band
            return self.cfg.near_min_m, min(self.cfg.far_max_m, max(self.cfg.near_min_m * 3.0, 5.0))

        # Depth from height and width (pinhole); guard divisions
        z_from_h = (self.fy * self.cfg.object_height_m) / max(1.0, float(bbox.size.y))
        z_from_w = (self.fx * self.cfg.object_width_m) / max(1.0, float(bbox.size.x))

        # Combine conservatively (use smaller i.e., nearer distance to avoid under-reaching)
        z_est = max(self.cfg.near_min_m, min(z_from_h, z_from_w))

        # Area-based uncertainty: smaller bbox -> larger margin
        area_px = float(bbox.size.x * bbox.size.y)
        area_ratio = np.clip(area_px / max(1.0, self.image_area), 0.0, 1.0)

        if self.cfg.mode == "coarse":
            # 3 coarse bins with fixed margins and tier scales
            t_large, t_medium = self.cfg.coarse_thresholds
            m_large, m_medium, m_small = self.cfg.coarse_margins
            if area_ratio >= t_large:
                tiers = 2
                margin = m_large
            elif area_ratio >= t_medium:
                tiers = 4
                margin = m_medium
            else:
                tiers = 5
                margin = m_small
            tier_scale = tiers / 5.0
        else:
            tiers = self._determine_tiers(area_ratio)
            margin = self._compute_margin(area_ratio)
            tier_scale = tiers / 5.0  # 2->0.4, 5->1.0

        band = max(0.1, z_est * margin * tier_scale)

        near = max(self.cfg.near_min_m, z_est - band)
        far = min(self.cfg.far_max_m, z_est + band)

        # Ensure a minimum band width
        MIN_BAND = 0.5
        if far - near < MIN_BAND:
            expand = (MIN_BAND - (far - near)) * 0.5
            near = max(self.cfg.near_min_m, near - expand)
            far = min(self.cfg.far_max_m, far + expand)

        return float(near), float(far)

    def _determine_tiers(self, area_ratio: float) -> int:
        t0, t1, t2 = self.cfg.area_thresholds
        if area_ratio >= t0:
            return 2
        if area_ratio >= t1:
            return 3
        if area_ratio >= t2:
            return 4
        return 5

    def _compute_margin(self, area_ratio: float) -> float:
        # Interpolate margin from [margin_max .. margin_min] as area grows
        # Use smoothstep for nicer behavior
        a = np.clip(area_ratio / max(1e-6, self.cfg.area_thresholds[0]), 0.0, 1.0)
        a_smooth = a * a * (3 - 2 * a)
        return float(self.cfg.margin_max + (self.cfg.margin_min - self.cfg.margin_max) * a_smooth)
