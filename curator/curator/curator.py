import rclpy
import random
import os
from ament_index_python.packages import get_package_share_directory
from curator import locations, art
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node

model_dir = os.path.join(get_package_share_directory("tb3_gazebo"), "models")

class Curator(Node):
    def __init__(self):
        super().__init__("curator")
        self.declare_parameter("exhibit_size", 1)

        self.spawner = self.create_client(SpawnEntity, "/spawn_entity")
        self.spawner.wait_for_service()


    def spawn(self, model, location):
        f = open(os.path.join(model_dir, model, "model.sdf"), 'r')

        entity_xml = f.read()
        req = SpawnEntity.Request()
        req.name = model
        req.xml = entity_xml
        req.initial_pose = location

        future = self.spawner.call_async(req)

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self)

    def setup_gallery(self):
        k = self.get_parameter("exhibit_size").get_parameter_value().integer_value

        location_picks = random.sample(locations.pool, k)
        art_picks = random.sample(art.pool, k)
        tag_picks = random.sample(range(30), k)

        debug_file = open("curator_debug.txt", "w")

        for i in range(k):
            debug_file.write(str(tag_picks[i]) + ": " + art_picks[i] + "\n")

            image = art_picks[i]
            tag = "tag" + str(tag_picks[i])
            location = location_picks[i]

            self.spawn(image, location.art)
            self.spawn(tag, location.tag)

        debug_file.close()

def main():
    rclpy.init()

    c = Curator()
    c.setup_gallery()

if __name__ == '__main__':
    main()
