"""Rotierbares 3D-Modell des G1 mit anklickbaren Gelenken.

Bedienung:
* Linke Maustaste ziehen  — Modell drehen
* Mausrad                 — Zoom
* Rechte/mittlere Taste   — Ansicht verschieben
* Linksklick auf Gelenk   — Gelenk auswählen (Menü erscheint rechts)

Die Gelenkauswahl erfolgt über Color-Picking: Die Szene wird unsichtbar
mit einer Kennfarbe pro Gelenk gerendert und das Pixel unter dem
Mauszeiger ausgelesen.
"""

from __future__ import annotations

import math

import numpy as np
from OpenGL.GL import *  # noqa: F401,F403
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QSurfaceFormat
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from ..model.joints import (
    GROUP_LEFT_ARM,
    GROUP_LEFT_LEG,
    GROUP_RIGHT_ARM,
    GROUP_RIGHT_LEG,
    GROUP_WAIST,
    JOINTS,
    NUM_JOINTS,
)
from ..model.kinematics import G1Kinematics

GROUP_COLORS = {
    GROUP_LEFT_LEG: (0.30, 0.55, 0.95),
    GROUP_RIGHT_LEG: (0.25, 0.80, 0.60),
    GROUP_WAIST: (0.95, 0.75, 0.20),
    GROUP_LEFT_ARM: (0.65, 0.45, 0.95),
    GROUP_RIGHT_ARM: (0.95, 0.45, 0.55),
}
SELECT_COLOR = (1.0, 0.45, 0.05)
BONE_COLOR = (0.75, 0.78, 0.82)
JOINT_RADIUS = 0.032
PICK_RADIUS = 0.045  # etwas größer, damit Gelenke leichter zu treffen sind


def _unit_sphere(stacks: int = 10, slices: int = 14):
    """Dreiecksliste einer Einheitskugel (Vertex == Normale)."""
    tris = []
    for i in range(stacks):
        t0 = math.pi * i / stacks
        t1 = math.pi * (i + 1) / stacks
        for j in range(slices):
            p0 = 2 * math.pi * j / slices
            p1 = 2 * math.pi * (j + 1) / slices
            v = lambda t, p: (math.sin(t) * math.cos(p), math.sin(t) * math.sin(p), math.cos(t))
            a, b, c, d = v(t0, p0), v(t1, p0), v(t1, p1), v(t0, p1)
            tris += [a, b, c, a, c, d]
    return np.array(tris, dtype=np.float32)


class RobotView(QOpenGLWidget):
    jointClicked = Signal(int)      # Motorindex
    selectionCleared = Signal()

    def __init__(self, parent=None) -> None:
        fmt = QSurfaceFormat()
        fmt.setSamples(4)
        fmt.setDepthBufferSize(24)
        QSurfaceFormat.setDefaultFormat(fmt)
        super().__init__(parent)
        self.kin = G1Kinematics()
        self.q = np.zeros(NUM_JOINTS)
        self.selected: int | None = None
        self.yaw = 40.0
        self.pitch = -20.0
        self.dist = 2.2
        self.target = np.array([0.0, 0.0, -0.15])
        self._last_pos = None
        self._dragged = False
        self._sphere = _unit_sphere()
        self.setMinimumSize(480, 480)
        self.setFocusPolicy(Qt.StrongFocus)

    # ------------------------------------------------------------- API
    def set_pose(self, q: np.ndarray) -> None:
        if not np.array_equal(q, self.q):
            self.q = np.asarray(q, dtype=float).copy()
            self.update()

    def set_selected(self, index: int | None) -> None:
        if index != self.selected:
            self.selected = index
            self.update()

    # ------------------------------------------------------ GL-Grundlagen
    def initializeGL(self) -> None:
        glClearColor(0.13, 0.15, 0.18, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_MULTISAMPLE)
        glEnable(GL_NORMALIZE)  # Normalen trotz glScalef korrekt halten
        glLightfv(GL_LIGHT0, GL_POSITION, (0.5, 0.3, 1.0, 0.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.9, 0.9, 0.9, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.35, 0.35, 0.35, 1.0))
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    def resizeGL(self, w: int, h: int) -> None:
        glViewport(0, 0, max(w, 1), max(h, 1))

    def _apply_camera(self) -> None:
        w, h = max(self.width(), 1), max(self.height(), 1)
        aspect = w / h
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        near, far, fov = 0.05, 50.0, 45.0
        top = near * math.tan(math.radians(fov) / 2)
        glFrustum(-top * aspect, top * aspect, -top, top, near, far)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -self.dist)
        glRotatef(self.pitch - 90.0, 1.0, 0.0, 0.0)   # z-Achse nach oben
        glRotatef(self.yaw, 0.0, 0.0, 1.0)
        glTranslatef(-self.target[0], -self.target[1], -self.target[2])

    # ------------------------------------------------------------ Rendern
    def paintGL(self) -> None:
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._apply_camera()
        transforms = self.kin.forward(self.q)
        pos = {name: t[:3, 3] for name, t in transforms.items()}

        self._draw_ground()
        self._draw_bones(pos)
        self._draw_joints(pos)
        if self.selected is not None:
            self._draw_axis(transforms, pos)

    def _draw_ground(self) -> None:
        glDisable(GL_LIGHTING)
        glLineWidth(1.0)
        glColor3f(0.28, 0.30, 0.34)
        glBegin(GL_LINES)
        z = -0.83  # Standhöhe (Füße) bei Nullstellung
        for i in range(-8, 9):
            glVertex3f(i * 0.125, -1.0, z)
            glVertex3f(i * 0.125, 1.0, z)
            glVertex3f(-1.0, i * 0.125, z)
            glVertex3f(1.0, i * 0.125, z)
        glEnd()

    def _draw_bones(self, pos) -> None:
        glDisable(GL_LIGHTING)
        glLineWidth(5.0)
        glColor3f(*BONE_COLOR)
        glBegin(GL_LINES)
        for parent, child in self.kin.segments():
            glVertex3f(*pos[parent])
            glVertex3f(*pos[child])
        glEnd()
        # Kopf als Kugel andeuten
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glColor3f(0.55, 0.58, 0.62)
        self._sphere_at(pos["head"], 0.055)

    def _draw_joints(self, pos) -> None:
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        for spec in JOINTS:
            p = pos[self.kin.joint_nodes[spec.index]]
            if spec.index == self.selected:
                glColor3f(*SELECT_COLOR)
                r = JOINT_RADIUS * 1.35
            else:
                glColor3f(*GROUP_COLORS[spec.group])
                r = JOINT_RADIUS
            self._sphere_at(p, r)

    def _draw_axis(self, transforms, pos) -> None:
        """Drehachse des ausgewählten Gelenks anzeigen."""
        axis = self.kin.axis_world(transforms, self.selected)
        p = pos[self.kin.joint_nodes[self.selected]]
        a, b = p - axis * 0.12, p + axis * 0.12
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)
        glColor3f(1.0, 0.9, 0.2)
        glBegin(GL_LINES)
        glVertex3f(*a)
        glVertex3f(*b)
        glEnd()

    def _sphere_at(self, p, radius: float) -> None:
        glPushMatrix()
        glTranslatef(*p)
        glScalef(radius, radius, radius)
        glBegin(GL_TRIANGLES)
        for v in self._sphere:
            glNormal3f(*v)
            glVertex3f(*v)
        glEnd()
        glPopMatrix()

    # ------------------------------------------------------------ Picking
    def _pick(self, x: int, y: int) -> int | None:
        from PySide6.QtOpenGL import QOpenGLFramebufferObject, QOpenGLFramebufferObjectFormat

        self.makeCurrent()
        ratio = self.devicePixelRatio()
        w, h = int(self.width() * ratio), int(self.height() * ratio)
        fmt = QOpenGLFramebufferObjectFormat()
        fmt.setAttachment(QOpenGLFramebufferObject.CombinedDepthStencil)
        fbo = QOpenGLFramebufferObject(w, h, fmt)  # ohne Multisampling => lesbar
        fbo.bind()
        glViewport(0, 0, w, h)
        glDisable(GL_MULTISAMPLE)
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._apply_camera()
        glDisable(GL_LIGHTING)
        pos = self.kin.positions(self.q)
        for spec in JOINTS:
            glColor3ub(spec.index + 1, 0, 0)
            self._sphere_at(pos[self.kin.joint_nodes[spec.index]], PICK_RADIUS)
        glFlush()
        px = int(x * ratio)
        py = h - int(y * ratio) - 1
        buf = glReadPixels(px, py, 1, 1, GL_RGB, GL_UNSIGNED_BYTE)
        fbo.release()
        glClearColor(0.13, 0.15, 0.18, 1.0)
        glEnable(GL_MULTISAMPLE)
        self.doneCurrent()
        self.update()
        red = buf[0] if isinstance(buf[0], int) else int(np.asarray(buf).flat[0])
        idx = red - 1
        return idx if 0 <= idx < NUM_JOINTS else None

    # -------------------------------------------------------- Mausbedienung
    def mousePressEvent(self, ev) -> None:
        self._last_pos = ev.position()
        self._dragged = False

    def mouseMoveEvent(self, ev) -> None:
        if self._last_pos is None:
            return
        d = ev.position() - self._last_pos
        if abs(d.x()) + abs(d.y()) > 2:
            self._dragged = True
        self._last_pos = ev.position()
        if ev.buttons() & Qt.LeftButton:
            self.yaw += d.x() * 0.5
            self.pitch = max(-89.0, min(89.0, self.pitch + d.y() * 0.5))
            self.update()
        elif ev.buttons() & (Qt.RightButton | Qt.MiddleButton):
            # Verschieben in der Bildebene
            s = self.dist * 0.0015
            yaw = math.radians(self.yaw)
            right = np.array([math.cos(yaw), -math.sin(yaw), 0.0])
            self.target -= right * d.x() * s
            self.target[2] += d.y() * s
            self.update()

    def mouseReleaseEvent(self, ev) -> None:
        if ev.button() == Qt.LeftButton and not self._dragged:
            idx = self._pick(int(ev.position().x()), int(ev.position().y()))
            if idx is not None:
                self.selected = idx
                self.jointClicked.emit(idx)
            else:
                self.selected = None
                self.selectionCleared.emit()
            self.update()
        self._last_pos = None

    def wheelEvent(self, ev) -> None:
        self.dist = max(0.5, min(8.0, self.dist * (0.9 if ev.angleDelta().y() > 0 else 1.1)))
        self.update()
