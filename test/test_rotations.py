import math

import numpy as np
import pytest

from unicore_rtk_driver.utils import rotations

Rotation = pytest.importorskip('scipy.spatial.transform').Rotation

ANGLES = [(0, 0, 0), (0.1, -0.2, 0.3), (math.pi, 0, math.pi / 2), (0.4, 1.2, -2.9),
          (-3.0, 0.7, 3.1), (0, math.pi / 2 - 1e-6, 1.0)]


@pytest.mark.parametrize('rpy', ANGLES)
def test_against_scipy(rpy):
    q = rotations.quaternion_from_euler(*rpy)
    ref = Rotation.from_euler('xyz', rpy)  # extrinsic xyz == tf 'sxyz'
    assert np.allclose(abs(np.dot(q, ref.as_quat())), 1.0)
    m = rotations.quaternion_matrix(q)
    assert np.allclose(m[:3, :3], ref.as_matrix())
    q2 = rotations.quaternion_from_matrix(m)
    assert np.allclose(abs(np.dot(q2, q)), 1.0)
    r, p, y = rotations.euler_from_matrix(m)
    assert np.allclose(Rotation.from_euler('xyz', [r, p, y]).as_matrix(), ref.as_matrix(), atol=1e-6)
