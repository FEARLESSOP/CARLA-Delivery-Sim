"""Small HUD elements for the drone camera viewers (drone_viewer.py camera wall,
drone_window.py): a bubble-level tilt indicator.

The dot moves the way the drone leans, seen from above with the nose up: nose down
(leaning forwards) moves it up, right side down moves it right. The ring is the
largest tilt drawn (25 deg, the motion layer's limit); the dot is green when the
drone is nearly level (< 3 deg), amber up to 15 deg, red beyond. The pitch and
roll values are written next to it.

ドローンカメラ表示用の小さな HUD：水準器風の傾き表示。点は機体が傾く方向へ動く（機首下げで上、
右下がりで右）。ほぼ水平（3 度未満）で緑、15 度まで黄、それ以上で赤。
"""
import math

import pygame

RING_DEG = 25.0
LEVEL_DEG, STRONG_DEG = 3.0, 15.0
GREEN, AMBER, RED = (120, 255, 140), (255, 196, 64), (255, 80, 80)
GRID, BACK, TEXT = (170, 180, 195), (0, 0, 0, 140), (235, 235, 235)


def tilt_color(pitch, roll):
    t = math.hypot(pitch, roll)
    return GREEN if t < LEVEL_DEG else AMBER if t < STRONG_DEG else RED


def draw_tilt(surface, center, radius, pitch, roll, font=None, label=True):
    """Draw the indicator at ``center`` (pixels). ``pitch`` > 0 nose up, ``roll`` > 0
    right side down (CARLA, degrees). / 傾き表示を描く。"""
    cx, cy = int(center[0]), int(center[1])
    r = int(radius)
    back = pygame.Surface((2 * r + 6, 2 * r + 6), pygame.SRCALPHA)
    pygame.draw.circle(back, BACK, (r + 3, r + 3), r + 3)
    surface.blit(back, (cx - r - 3, cy - r - 3))
    pygame.draw.circle(surface, GRID, (cx, cy), r, 1)
    pygame.draw.circle(surface, GRID, (cx, cy), max(2, int(r * LEVEL_DEG / RING_DEG)), 1)   # "level" zone
    pygame.draw.line(surface, GRID, (cx - r, cy), (cx + r, cy), 1)
    pygame.draw.line(surface, GRID, (cx, cy - r), (cx, cy + r), 1)
    # nose down (pitch < 0) -> up; right side down (roll > 0) -> right / 機首下げで上、右下がりで右
    dx, dy = roll / RING_DEG, pitch / RING_DEG
    n = math.hypot(dx, dy)
    if n > 1.0:
        dx, dy = dx / n, dy / n
    dot = (cx + int(dx * (r - 4)), cy + int(dy * (r - 4)))
    col = tilt_color(pitch, roll)
    pygame.draw.line(surface, col, (cx, cy), dot, 2)
    pygame.draw.circle(surface, col, dot, max(3, r // 5))
    if label and font is not None:
        txt = "P%+.0f R%+.0f" % (pitch, roll)
        img = font.render(txt, True, TEXT)
        rect = img.get_rect(midtop=(cx, cy + r + 4))
        bg = pygame.Surface((rect.w + 6, rect.h + 2), pygame.SRCALPHA)
        bg.fill(BACK)
        surface.blit(bg, (rect.x - 3, rect.y - 1))
        surface.blit(img, rect)
