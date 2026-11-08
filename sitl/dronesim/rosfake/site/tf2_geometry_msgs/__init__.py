"""tf2_geometry_msgs.do_transform_* on the dronesim tf2 implementation."""
from dronesim.rosfake.tf2_impl import q_mul, q_rotate


def _tf(transform):
    tr, ro = transform.transform.translation, transform.transform.rotation
    return (tr.x, tr.y, tr.z), (ro.x, ro.y, ro.z, ro.w)


def do_transform_point(point, transform):
    from geometry_msgs.msg import PointStamped
    t, q = _tf(transform)
    p = q_rotate(q, (point.point.x, point.point.y, point.point.z))
    out = PointStamped()
    out.header = transform.header
    out.point.x, out.point.y, out.point.z = (float(p[i] + t[i]) for i in range(3))
    return out


def do_transform_vector3(vector, transform):
    from geometry_msgs.msg import Vector3Stamped
    _t, q = _tf(transform)
    v = q_rotate(q, (vector.vector.x, vector.vector.y, vector.vector.z))
    out = Vector3Stamped()
    out.header = transform.header
    out.vector.x, out.vector.y, out.vector.z = (float(x) for x in v)
    return out


def _pose(pose, transform):
    from geometry_msgs.msg import Pose
    t, q = _tf(transform)
    p = q_rotate(q, (pose.position.x, pose.position.y, pose.position.z))
    o = q_mul(q, (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    out = Pose()
    out.position.x, out.position.y, out.position.z = (float(p[i] + t[i]) for i in range(3))
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = (float(x) for x in o)
    return out


def do_transform_pose(pose, transform):
    return _pose(pose, transform)


def do_transform_pose_stamped(pose, transform):
    from geometry_msgs.msg import PoseStamped
    out = PoseStamped()
    out.header = transform.header
    out.pose = _pose(pose.pose, transform)
    return out


def do_transform(obj, transform):
    name = type(obj).__name__
    if name == 'PointStamped':
        return do_transform_point(obj, transform)
    if name == 'Vector3Stamped':
        return do_transform_vector3(obj, transform)
    if name == 'PoseStamped':
        return do_transform_pose_stamped(obj, transform)
    raise TypeError(f'tf2_geometry_msgs: no conversion registered for {name}')
