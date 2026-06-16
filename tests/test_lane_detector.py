'''Tests for apm/vision/lane_detector.py

Covers:
  - LaneLines dataclass: vanishing point, lane center, parallel-line guard
  - LaneDetector: hough_to_slope_intercept, resize_image, canny_with_mask, detect

Run with:
    pytest tests/test_lane_detector.py -v
    pytest tests/test_lane_detector.py -v --plot   # saves annotated images to tests/output/
'''

import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import tomlkit

from apm.config_handler import DEFAULT_CONFIG_PATH, SETTINGS_PATH
from apm.vision.lane_detector import LaneDetector, LaneLines
from apm.vision.frame_encoder import encode_lane_keeping_jpeg


def _load_detector_config() -> dict:
    path = SETTINGS_PATH if SETTINGS_PATH.exists() else DEFAULT_CONFIG_PATH
    return tomlkit.loads(path.read_text(encoding='utf-8'))['lane_detector']

_LABEL_FONT  = cv2.FONT_HERSHEY_SIMPLEX
_LABEL_COLOR = (200, 200, 200)


def _label(image: np.ndarray, text: str) -> np.ndarray:
    out = image.copy()
    pos = (10, out.shape[0] - 12)
    cv2.putText(out, text, pos, _LABEL_FONT, 0.7, (0, 0, 0), 3)
    cv2.putText(out, text, pos, _LABEL_FONT, 0.7, _LABEL_COLOR, 1)
    return out


def _to_bgr(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img


def _save_pipeline_steps(stem: str, resized: np.ndarray, canny: np.ndarray, annotated: np.ndarray):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for img, suffix in [
        (resized,   f'{stem}_1_resized.jpg'),
        (canny,     f'{stem}_2_canny.jpg'),
        (annotated, f'{stem}_3_detected.jpg'),
    ]:
        cv2.imwrite(str(OUTPUT_DIR / suffix), img)

FRONTCAM_DIR = Path('tests/input/img/lane_detector')
OUTPUT_DIR   = Path('tests/output')

LANE_IMAGES = sorted(
    p.name for p in FRONTCAM_DIR.glob('*')
    if p.suffix.lower() in ('.png', '.jpg', '.jpeg')
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load(filename: str) -> np.ndarray:
    path = FRONTCAM_DIR / filename
    img = cv2.imread(str(path))
    assert img is not None, f'Could not load {path}'
    return img


# ---------------------------------------------------------------------------
# LaneLines unit tests
# ---------------------------------------------------------------------------

class TestLaneLines:

    def _make(self, left_slope, left_intercept, right_slope, right_intercept, h=480, w=640):
        return LaneLines(
            left_slope=left_slope,
            left_intercept=left_intercept,
            right_slope=right_slope,
            right_intercept=right_intercept,
            image_height=h,
            image_width=w,
        )

    def test_vanishing_point_symmetric(self):
        # Two symmetric lines converging at x=320, y=0 for a 640-wide image
        # left:  y =  2x + 0  -> passes through (0,0) and (160, 320)
        # right: y = -2x + 1280 -> passes through (640,0) and (480, 320)
        lanes = self._make(left_slope=2.0, left_intercept=0.0,
                           right_slope=-2.0, right_intercept=1280.0, h=480)
        vx, vy = lanes.get_vanishing_point()
        assert abs(vx - 320.0) < 1e-6
        assert abs(vy - 640.0) < 1e-6

    def test_vanishing_point_parallel_raises(self):
        lanes = self._make(left_slope=1.0, left_intercept=0.0,
                           right_slope=1.0, right_intercept=100.0)
        with pytest.raises(ValueError, match='parallel'):
            lanes.get_vanishing_point()

    def test_lane_center_at_bottom_symmetric(self):
        # At image bottom (y=480): left line x = (480-0)/2 = 240, right x = (480-480)/(-2) = 0 ...
        # Use concrete numbers: left y=2x => x=y/2, right y=-2x+960 => x=(960-y)/2
        # At y=480: left_x=240, right_x=240 => center=240
        lanes = self._make(left_slope=2.0, left_intercept=0.0,
                           right_slope=-2.0, right_intercept=960.0, h=480)
        center = lanes.get_lane_center_at_bottom()
        assert abs(center - 240.0) < 1e-6

    def test_lane_center_near_horizontal_raises(self):
        lanes = self._make(left_slope=0.0, left_intercept=200.0,
                           right_slope=-2.0, right_intercept=960.0, h=480)
        with pytest.raises(ValueError, match='Left'):
            lanes.get_lane_center_at_bottom()


# ---------------------------------------------------------------------------
# LaneDetector unit tests
# ---------------------------------------------------------------------------

class TestHoughToSlopeIntercept:

    def setup_method(self):
        self.detector = LaneDetector()

    def test_vertical_line(self):
        # theta ≈ 0 => near-vertical line; sin≈0 guarded, should not raise
        a, b = self.detector.hough_to_slope_intercept(rho=100.0, theta=0.0)
        assert math.isfinite(a)
        assert math.isfinite(b)

    def test_horizontal_line(self):
        # theta = pi/2 => horizontal: y = rho
        a, b = self.detector.hough_to_slope_intercept(rho=200.0, theta=math.pi / 2)
        assert abs(a) < 1e-6       # slope ≈ 0
        assert abs(b - 200.0) < 1e-6

    def test_45_degree_line(self):
        # theta = pi/4 => slope = -cos/sin = -1, intercept = rho/sin = rho*sqrt(2)
        rho = 100.0
        a, b = self.detector.hough_to_slope_intercept(rho=rho, theta=math.pi / 4)
        assert abs(a - (-1.0)) < 1e-6
        assert abs(b - rho * math.sqrt(2)) < 1e-6


class TestPreprocess:

    def setup_method(self):
        self.detector = LaneDetector(cut_top=4/7, output_width=640, mask_polygons=None)

    def test_output_width(self):
        img = np.zeros((600, 800, 3), dtype=np.uint8)
        out = self.detector.resize_image(img)
        assert out.shape[1] == 640

    def test_cut_top_reduces_height(self):
        img = np.zeros((700, 700, 3), dtype=np.uint8)
        out = self.detector.resize_image(img)
        # After cutting 4/7 of 700 = 400 rows, 300 rows remain; resized to 640 wide
        # height = 300/700 * 640 ≈ 274
        expected_height = int(300 / 700 * 640)
        assert out.shape[0] == expected_height

    def test_resized_image_stored(self):
        img = np.zeros((600, 800, 3), dtype=np.uint8)
        self.detector.resize_image(img)
        assert self.detector.resized_image is not None

    def test_no_cut(self):
        detector = LaneDetector(cut_top=0.0, output_width=320, mask_polygons=None)
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        out = detector.resize_image(img)
        assert out.shape[1] == 320
        assert out.shape[0] == 240


class TestCannyWithMask:

    def test_returns_single_channel(self):
        detector = LaneDetector(mask_polygons=None)
        gray = np.random.randint(0, 255, (200, 200), dtype=np.uint8)
        edges = detector.canny_with_mask(gray)
        assert edges.ndim == 2

    def test_mask_zeroes_region(self):
        detector = LaneDetector(mask_polygons=[[[50, 50], [150, 50], [150, 150], [50, 150]]])
        # White image produces many edges at borders; the masked square should be zeroed
        gray = np.full((200, 200), 200, dtype=np.uint8)
        # Draw a bright square inside the mask region to create edges there
        gray[60:140, 60:140] = 50
        edges = detector.canny_with_mask(gray)
        # All pixels inside the mask polygon should be 0
        region = edges[60:140, 60:140]
        assert region.max() == 0, 'Mask did not suppress edges inside the polygon'


# ---------------------------------------------------------------------------
# Integration tests using real images
# ---------------------------------------------------------------------------

def test_lane_images_directory_not_empty():
    '''Fail early if no test images are present — add images or tune detection.'''
    assert LANE_IMAGES, (
        f'No images found in {FRONTCAM_DIR} — '
        'add .png/.jpg files to run lane detection integration tests'
    )


class TestDetectOnRealImages:

    def setup_method(self):
        self.detector = LaneDetector(cut_top=0.0, mask_polygons=None, hough_threshold=50)

    @pytest.mark.parametrize('filename', LANE_IMAGES)
    def test_detect_returns_lane_lines(self, filename):
        img = _load(filename)
        lanes = self.detector.detect(img)
        assert lanes is not None, f'Expected lane lines in {filename}, got None'
        assert math.isfinite(lanes.left_slope)
        assert math.isfinite(lanes.right_slope)

    
    @pytest.mark.parametrize('filename', LANE_IMAGES)
    def test_vanishing_point_above_midpoint(self, filename):
        '''Vanishing point should be in the upper half of the image (small y relative to height).'''
        img = _load(filename)
        lanes = self.detector.detect(img)
        if lanes is None:
            pytest.skip(f'No lanes detected in {filename}')
        _, vy = lanes.get_vanishing_point()
        assert vy < img.shape[0] / 2, (
            f'Vanishing point y={vy:.0f} is not in the upper half of the image (h={img.shape[0]})'
        )


    @pytest.mark.parametrize('filename', LANE_IMAGES)
    def test_lane_center_within_image_width(self, filename):
        img = _load(filename)
        lanes = self.detector.detect(img)
        if lanes is None:
            pytest.skip(f'No lanes detected in {filename}')
        try:
            center = lanes.get_lane_center_at_bottom()
        except ValueError:
            pytest.skip('Near-horizontal lane line')
        assert 0 <= center <= img.shape[1], (
            f'Lane center {center:.0f} is outside image width {img.shape[1]}'
        )

    @pytest.mark.parametrize('filename', LANE_IMAGES)
    def test_encode_lane_keeping_jpeg(self, filename):
        '''Smoke-test that encode_lane_keeping_jpeg runs without error and produces bytes.'''
        img = _load(filename)
        bgra = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
        lanes = self.detector.detect(img)
        result = encode_lane_keeping_jpeg(bgra, lane_lines=lanes)
        assert isinstance(result, bytes)
        assert len(result) > 0


class TestDetectWithPreprocess:
    '''Test the full resize_image -> detect pipeline used in production.'''

    def setup_method(self):
        cfg = _load_detector_config()
        self.detector = LaneDetector(
            cut_top=float(cfg['cut_top']),
            angle_threshold=float(cfg['angle_threshold']),
            hough_threshold=int(cfg['hough_threshold']),
            canny_sigma=float(cfg['canny_sigma']),
            mask_polygons=cfg['mask_polygons'],
        )

    @pytest.mark.parametrize('filename', LANE_IMAGES)
    def test_preprocess_then_detect(self, request, filename):
        img = _load(filename)
        resized = self.detector.resize_image(img)
        lanes = self.detector.detect(resized)
        # We don't assert lanes is not None here — resized crops out much of the scene
        # and the default mask may not fit every image — but no exception should be raised.
        if lanes is not None:
            assert math.isfinite(lanes.left_slope)
            assert math.isfinite(lanes.right_slope)

        if request.config.getoption('--plot', default=False):
            heading = None
            if lanes is not None:
                try:
                    heading = lanes.get_heading()
                except ValueError:
                    pass

            jpg = encode_lane_keeping_jpeg(
                cv2.cvtColor(resized, cv2.COLOR_BGR2BGRA),
                lane_lines=lanes,
                heading_angle=heading,
                mask_polygons=self.detector.mask_polygons,
            )
            annotated = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)

            stem = Path(filename).stem
            status = f'heading={heading:.1f}' if heading is not None else 'not detected'
            status = ''
            _save_pipeline_steps(
                stem,
                _label(resized,   f'1 resized  {resized.shape[1]}x{resized.shape[0]}'),
                _label(_to_bgr(self.detector.image_canny), f'2 canny+mask  {status}'),
                _label(annotated, f'3 detected  {status}'),
            )


class TestDetectEdgeCases:

    def test_blank_image_returns_none(self):
        detector = LaneDetector(mask_polygons=None)
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        assert detector.detect(blank) is None

    def test_horizontal_lines_filtered_out(self):
        '''Horizontal lines fall outside the angle_threshold and should not be detected as lanes.'''
        detector = LaneDetector(mask_polygons=None, hough_threshold=10)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw two strong horizontal lines — angle ≈ 90°, outside the ±60° from vertical threshold
        cv2.line(img, (0, 200), (639, 200), (255, 255, 255), 3)
        cv2.line(img, (0, 300), (639, 300), (255, 255, 255), 3)
        result = detector.detect(img)
        assert result is None
