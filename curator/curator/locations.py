import tf_transformations
from geometry_msgs.msg import Pose

def to_pose(x, y, z, rx, ry, rz):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z

    q = tf_transformations.quaternion_from_euler(rx, ry, rz)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    
    return p

class ExhibitLocation:
    def __init__(self, tag, art):
        self.tag = tag
        self.art = art

pool = [
        ExhibitLocation(
            tag = to_pose(-0.636005, 4.45525, 0.25, 0, 0, 0.64501),
            art = to_pose(-0.420341, 4.618723, 0.25, 0, 0, 0.64501)
            ),
        ExhibitLocation(
            tag = to_pose(-3.304133, 4.826403, 0.25, 0, 0, 1.210165),
            art = to_pose(-3.198024, 5.110230, 0.25, 0, 0, 1.210165)
            ),
        ExhibitLocation(
            tag = to_pose(-1.129375, 7.109408, 0.25, 0 ,0, 0.4534463),
            art = to_pose(-0.850112, 7.236442, 0.25, 0 ,0, 0.4534463)
            ),
        ExhibitLocation(
            tag = to_pose(-0.882744, 4.301681, 0.25, 0, 0, 0.645007),
            art = to_pose(-1.124689, 4.121682, 0.25, 0, 0, 0.645007)
            ),
        ExhibitLocation(
            tag = to_pose(0.046312, -0.458784, 0.25, 0, 0, 0),
            art = to_pose(-0.270073, -0.453808, 0.25, 0, 0, 0)
            ),
        ]
