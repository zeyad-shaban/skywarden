import numpy as np
import cv2
import math
import time


def imgmsg_to_cv2(msg):
    if msg.encoding == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif msg.encoding == "rgb8":  # it's this
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif msg.encoding == "mono8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")


def draw_sniper_scope(img, center=None, radius_ratio=0.28, outer_dim_alpha=0.45, ring_color=(0, 0, 255), ring_thickness=3, cross_color=(0, 0, 255), cross_thickness=2):
    """
    Draw a sniper-scope style overlay:
    - darkens everything outside the circle
    - draws a red ring
    - draws crosshair lines that get thicker toward the edge
    """
    h, w = img.shape[:2]
    if center is None:
        center = (w // 2, h // 2)

    cx, cy = center
    radius = int(min(w, h) * radius_ratio)

    # 1) Darken outside the circle
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, h), (0, 0, 0), thickness=-1)

    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, thickness=-1)

    # blend only outside the circle
    outside = cv2.bitwise_not(mask)
    img[outside > 0] = cv2.addWeighted(img[outside > 0], 1.0 - outer_dim_alpha, overlay[outside > 0], outer_dim_alpha, 0)

    # 2) Draw the outer ring
    cv2.circle(img, center, radius, ring_color, ring_thickness)

    # 3) Draw crosshair lines with a "thin near center, thicker outward" feel
    gap = int(radius * 0.08)  # small gap around the center
    inner_len = int(radius * 0.22)  # thin central part
    outer_len = radius - 2

    # Horizontal left side: thin near center, thicker farther out
    cv2.line(img, (cx - inner_len, cy), (cx - gap, cy), cross_color, 1)
    cv2.line(img, (cx - outer_len, cy), (cx - inner_len, cy), cross_color, 3)

    # Horizontal right side
    cv2.line(img, (cx + gap, cy), (cx + inner_len, cy), cross_color, 1)
    cv2.line(img, (cx + inner_len, cy), (cx + outer_len, cy), cross_color, 3)

    # Vertical top
    cv2.line(img, (cx, cy - inner_len), (cx, cy - gap), cross_color, 1)
    cv2.line(img, (cx, cy - outer_len), (cx, cy - inner_len), cross_color, 3)

    # Vertical bottom
    cv2.line(img, (cx, cy + gap), (cx, cy + inner_len), cross_color, 1)
    cv2.line(img, (cx, cy + inner_len), (cx, cy + outer_len), cross_color, 3)

    # 4) Small center dot
    cv2.circle(img, center, 2, cross_color, -1)

    return img


def _blend_rgba_like(base_bgr, overlay_bgr, alpha_mask):
    """
    Blend overlay_bgr onto base_bgr using alpha_mask in [0, 1].
    alpha_mask can be scalar, 2D, or 3D.
    """
    if alpha_mask.ndim == 2:
        alpha_mask = alpha_mask[..., None]

    alpha_mask = np.clip(alpha_mask, 0.0, 1.0).astype(np.float32)
    base = base_bgr.astype(np.float32)
    over = overlay_bgr.astype(np.float32)
    out = base * (1.0 - alpha_mask) + over * alpha_mask
    return np.clip(out, 0, 255).astype(np.uint8)


def draw_sniper_scope_better(
    img,
    center=None,
    radius_ratio=0.30,
    vignette_strength=0.72,
    feather_ratio=0.18,
    ring_color=(0, 0, 255),
    ring_thickness=3,
    inner_ring_thickness=1,
    cross_color=(0, 0, 255),
    cross_thin=1,
    cross_thick=3,
    gap_ratio=0.07,
    inner_len_ratio=0.22,
    outer_len_ratio=0.92,
    tick_len_ratio=0.06,
    add_pulse=False,
    pulse_speed=2.5,
):
    """
    A polished FPS-style sniper scope overlay.

    radius_ratio: size of the scope circle
    feather_ratio: softness of the edge
    vignette_strength: how dark the outside gets
    add_pulse=True: makes it feel more alive

    img: BGR image
    """
    h, w = img.shape[:2]
    if center is None:
        center = (w // 2, h // 2)

    cx, cy = center
    radius = int(min(w, h) * radius_ratio)
    feather = max(2, int(radius * feather_ratio))

    # Work on a copy
    out = img.copy()

    # Optional subtle pulse so it feels more like a live HUD
    pulse = 1.0
    if add_pulse:
        pulse = 1.0 + 0.035 * math.sin(time.time() * pulse_speed)

    r = max(2, int(radius * pulse))
    feather = max(2, int(feather * pulse))

    # ---------- 1) Soft vignette outside the scope ----------
    yy, xx = np.ogrid[:h, :w]
    dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)

    # 0 inside the clear circle, 1 far outside with feathered transition
    outside_alpha = np.clip((dist - (r - feather)) / feather, 0.0, 1.0)
    outside_alpha = (outside_alpha**1.35) * vignette_strength

    dark = np.zeros_like(out)
    out = _blend_rgba_like(out, dark, outside_alpha)

    # ---------- 2) Slight brightening inside the scope ----------
    inside_alpha = np.clip((r - dist) / feather, 0.0, 1.0)
    inside_boost = (inside_alpha * 0.08)[..., None]
    out = np.clip(out.astype(np.float32) * (1.0 + inside_boost), 0, 255).astype(np.uint8)

    # ---------- 3) Lens glow / reflection ----------
    glow = np.zeros_like(out)
    glow_center = (int(cx - r * 0.22), int(cy - r * 0.22))
    cv2.circle(glow, glow_center, int(r * 0.18), (255, 255, 255), -1, lineType=cv2.LINE_AA)
    glow = cv2.GaussianBlur(glow, (0, 0), sigmaX=max(1, r * 0.06), sigmaY=max(1, r * 0.06))
    glow_mask = np.clip((r - dist) / r, 0.0, 1.0) ** 2
    out = _blend_rgba_like(out, glow, glow_mask * 0.14)

    # ---------- 4) Ring with a shadow/sharp edge ----------
    # subtle dark under-ring for depth
    cv2.circle(out, center, r, (20, 20, 20), ring_thickness + 4, lineType=cv2.LINE_AA)
    # main red ring
    cv2.circle(out, center, r, ring_color, ring_thickness, lineType=cv2.LINE_AA)
    # thin inner highlight ring
    cv2.circle(out, center, max(1, r - 4), (255, 120, 120), inner_ring_thickness, lineType=cv2.LINE_AA)

    # ---------- 5) Crosshair segments ----------
    gap = max(2, int(r * gap_ratio))
    inner_len = max(4, int(r * inner_len_ratio))
    outer_len = max(8, int(r * outer_len_ratio))

    def draw_segment(p1, p2, thin=True):
        cv2.line(out, p1, p2, cross_color, cross_thin if thin else cross_thick, lineType=cv2.LINE_AA)

    # Horizontal
    draw_segment((cx - outer_len, cy), (cx - inner_len, cy), thin=False)
    draw_segment((cx - inner_len, cy), (cx - gap, cy), thin=True)
    draw_segment((cx + gap, cy), (cx + inner_len, cy), thin=True)
    draw_segment((cx + inner_len, cy), (cx + outer_len, cy), thin=False)

    # Vertical
    draw_segment((cx, cy - outer_len), (cx, cy - inner_len), thin=False)
    draw_segment((cx, cy - inner_len), (cx, cy - gap), thin=True)
    draw_segment((cx, cy + gap), (cx, cy + inner_len), thin=True)
    draw_segment((cx, cy + inner_len), (cx, cy + outer_len), thin=False)

    # ---------- 6) Tick marks around the ring ----------
    tick_len = max(3, int(r * tick_len_ratio))
    tick_in = r - 2
    tick_out = r + tick_len

    ticks = [
        ((cx, cy - tick_out), (cx, cy - tick_in)),  # top
        ((cx + tick_in, cy), (cx + tick_out, cy)),  # right
        ((cx, cy + tick_in), (cx, cy + tick_out)),  # bottom
        ((cx - tick_out, cy), (cx - tick_in, cy)),  # left
    ]
    for p1, p2 in ticks:
        cv2.line(out, p1, p2, (255, 80, 80), 2, lineType=cv2.LINE_AA)

    # ---------- 7) Center dot ----------
    cv2.circle(out, center, 3, (255, 255, 255), -1, lineType=cv2.LINE_AA)
    cv2.circle(out, center, 6, (0, 0, 0), 1, lineType=cv2.LINE_AA)

    return out


def draw_target_crosshair(img, target_cx: int, target_cy: int):
    cv2.circle(img, (target_cx, target_cy), 1, (0, 0, 255), 2)
