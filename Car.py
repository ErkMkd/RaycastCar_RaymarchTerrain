# -*-coding:Utf-8 -*

import harfang as hg
from math import radians, sqrt

# ========================================================================================================
#               Car class
# ========================================================================================================

class Car:
    def __init__(self, name, plus, scene, start_position: hg.Vector3, start_rotation=hg.Vector3.Zero):
        self.name = name
        self.chassis_node = scene.GetNode("car_body")
        self.thrust = scene.GetNode("thrust")
        self.wheels = self.get_wheels(plus, scene)
        self.chassis_rigid, self.collisions_boxes = self.get_collisions(scene)

        self.local_rays = self.get_rays()
        self.ray_dir = None
        self.wheels_ray = self.wheels[0].GetObject().GetLocalMinMax().mx.y
        self.ray_max_dist = self.wheels_ray + 0.2

        self.wheels_rot_speed = [0] * 4
        self.ground_hits = [False] * 4
        self.ground_impacts = [None] * 4

        self.mass = 0
        self.density = 0
        self.spring_friction = 0
        self.tires_reaction = 0
        self.tires_adhesion = 0
        self.front_angle = 0
        self.front_angle_max = 45

        self.setup()

        self.start_position = start_position
        self.start_rotation = start_rotation
        self.chassis_node.GetTransform().SetPosition(start_position)

        plus.UpdateScene(scene, plus.UpdateClock())

    def setup(self, mass=1500, spring_friction=0.5, tires_reaction=26, tires_adhesion=0.04, linear_damping=0.5,
              angular_damping=0.6):
        self.chassis_rigid.SetAngularDamping(angular_damping)
        self.chassis_rigid.SetLinearDamping(linear_damping)
        self.spring_friction = spring_friction
        self.tires_reaction = tires_reaction
        self.tires_adhesion = tires_adhesion
        self.set_mass(mass)

    def set_mass(self, mass):
        volumes = []
        total_volume = 0
        for colbox in self.collisions_boxes:
            dimensions = colbox.GetDimensions()
            volume = dimensions.x * dimensions.y * dimensions.z
            total_volume += volume
            volumes.append(volume)
        self.density = mass / total_volume / 1000
        for i in range(len(self.collisions_boxes)):
            self.collisions_boxes[i].SetMass(volumes[i] / total_volume * mass)
        self.mass = mass

    def set_density(self, density=0.3):
        self.density = density
        self.mass = 0
        for colbox in self.collisions_boxes:
            dimensions = colbox.GetDimensions()
            box_mass = dimensions.x * dimensions.y * dimensions.z * self.density * 1000
            colbox.SetMass(box_mass)
            self.mass += box_mass

    def get_collisions(self, scene):
        rigid = hg.RigidBody()
        rigid.SetType(hg.RigidBodyDynamic)
        self.chassis_node.AddComponent(rigid)
        collisions_nodes = scene.GetNodes("col_shape")
        collisions_boxes = []
        for col_shape in collisions_nodes:
            colbox = hg.BoxCollision()
            collisions_boxes.append(colbox)
            obj = col_shape.GetObject()
            bounds = obj.GetLocalMinMax()
            dimensions = bounds.mx - bounds.mn
            pos = col_shape.GetTransform().GetPosition() + bounds.mn + dimensions * 0.5
            colbox.SetDimensions(dimensions)
            colbox.SetMatrix(hg.Matrix4.TranslationMatrix(pos))
            self.chassis_node.AddComponent(colbox)
            scene.RemoveNode(col_shape)
        return rigid, collisions_boxes

    def reset(self, pos=None):
        if pos is None:
            pos=self.start_position
        self.chassis_rigid.ResetWorld(hg.Matrix4.TransformationMatrix(pos, self.start_rotation))

    def get_rays(self):
        rays = []
        for wheel in self.wheels:
            rays.append(wheel.GetTransform().GetPosition())
        return rays

    def get_wheels(self, plus, scene):
        wheels = []
        for n in range(4):
            wheel = scene.GetNode("wheel_" + str(n))
            wheels.append(wheel)
        return wheels

    def turn(self, angle):
        self.front_angle = max(min(self.front_angle + angle, self.front_angle_max), -self.front_angle_max)
        self.thrust.GetTransform().SetRotation(hg.Vector3(0, radians(self.front_angle), 0))

    def accelerate(self, value):
        f = 0
        for i in range(2):
            if self.ground_hits[i]:
                f += 0.5
        pos = self.thrust.GetTransform().GetWorld().GetTranslation()
        dir = self.thrust.GetTransform().GetWorld().GetZ()
        self.chassis_rigid.ApplyForce(dir * self.mass * f * value, pos)

    def brake(self, value):
        f = 0
        for i in range(4):
            if self.ground_hits[i]:
                f += 0.25
        v = self.chassis_rigid.GetLinearVelocity()
        value *= min(v.Len(), 1)
        self.chassis_rigid.ApplyLinearForce(v.Normalized() * self.mass * f * -value)

    def update_kinetic(self, scene, terrain, dts):
        self.chassis_rigid.SetIsSleeping(False)
        self.ray_dir = self.chassis_node.GetTransform().GetWorld().GetY().Reversed()
        for i in range(4):
            self.update_wheel_physic(i, scene, terrain, dts)

    def update_wheel_physic(self, id, scene, terrain, dts):
        wheel = self.wheels[id]
        mat = self.chassis_node.GetTransform().GetWorld()  # Ray position in World space
        ray_pos = mat * self.local_rays[id]
        self.ground_hits[id], self.ground_impacts[id] = terrain.raycast(ray_pos,self.ray_dir,self.ray_max_dist)
        #scene.GetPhysicSystem().Raycast(ray_pos, self.ray_dir, 0x255,self.ray_max_dist)

        if self.ground_hits[id]:
            v = scene.GetPhysicSystem().GetRigidBodyVelocity(self.chassis_rigid, ray_pos).Reversed()
            hit_distance = (self.ground_impacts[id].GetPosition() - ray_pos).Len()

            # Spring bounce:
            v_dot_ground_n = hg.Dot(self.ground_impacts[id].GetNormal(), v)
            if v_dot_ground_n > 0:
                v_bounce = self.ground_impacts[id].GetNormal() * v_dot_ground_n
                self.chassis_rigid.ApplyImpulse(v_bounce * self.spring_friction, ray_pos)

            # Tire/Ground reaction:
            wheel_reaction = sqrt(abs(self.ray_max_dist - hit_distance)) * self.tires_reaction
            self.chassis_rigid.ApplyForce(self.ground_impacts[id].GetNormal() * wheel_reaction * self.mass / 4, ray_pos)

            # Wheel lateral friction:
            x_axis = wheel.GetTransform().GetWorld().GetX()
            proj = hg.Dot(x_axis, v)
            v_lat = x_axis * proj
            self.chassis_rigid.ApplyImpulse(v_lat * self.tires_adhesion, ray_pos)

            # Adjust wheel on the ground.
            wheel_p = wheel.GetTransform().GetPosition()
            wheel_p.y = self.local_rays[id].y - hit_distance + self.wheels_ray
            wheel.GetTransform().SetPosition(wheel_p)

            # Wheel rotation:
            z_axis = hg.Cross(x_axis, self.ray_dir).Normalized()
            vlin = hg.Dot(z_axis, v)  # Linear speed (along Z axis)
            self.wheels_rot_speed[id] = (vlin / self.wheels_ray)
        else:
            self.wheels_rot_speed[id] *= 0.95  # Wheel slow-down

        rot = wheel.GetTransform().GetRotation()
        rot.x += self.wheels_rot_speed[id] * dts
        if id == 0 or id == 1:
            rot.y = radians(self.front_angle)
        wheel.GetTransform().SetRotation(rot)

    def get_parent_node(self):
        return self.chassis_node

    def get_position(self):
        return self.chassis_node.GetTransform().GetPosition()

# ==================================================================================================
#                       Camera follow
# ==================================================================================================

follow_distance = 0
follow_altitude = 0
fps_ref = 0
target_point = hg.Vector3(0, 0, 0)
target_node = None
wall_jump_k = 0
v_wall_jump = None


def setup_camera_follow(targetNode: hg.Node, targetPoint: hg.Vector3, distance=20, altitude=1.7, jump_force=20, fps=60):
    global target_point, target_node, follow_distance, follow_altitude, fps_ref, v_wall_jump, wall_jump_k
    target_point = targetPoint
    target_node = targetNode
    follow_distance = distance
    follow_altitude = altitude
    v_wall_jump = hg.Vector3(0, 0, 0)
    fps_ref = fps
    wall_jump_k = jump_force


def RangeAdjust(value, oldmin, oldmax, newmin, newmax):
    return (value - oldmin) / (oldmax - oldmin) * (newmax - newmin) + newmin


def update_target_point(dts, inertia=0.1):
    global target_point
    v = target_node.GetTransform().GetPosition() - target_point
    target_point += v * pow(inertia, fps_ref * dts)


def update_follow_translation(camera: hg.Node, scene, terrain, dts, inertia=0.1):
    global v_wall_jump
    trans = camera.GetTransform()
    camera_pos = camera.GetTransform().GetPosition()
    target_pos = target_node.GetTransform().GetPosition()

    # Wall
    v = target_pos - camera_pos
    target_dir = v.Normalized()
    target_dist = v.Len()

    hit, impact = terrain.raycast(trans.GetPosition(), target_dir, target_dist)
    if hit: #and impact.GetNode() != target_node:
        v_wall_jump.y += wall_jump_k * dts
    else:
        v_wall_jump *= pow(0.95,fps_ref*dts)

    y_alt = follow_altitude - (camera_pos.y - target_pos.y)
    v_wall_jump.y += y_alt * dts

    camera_pos += v_wall_jump * dts
    v_trans = target_dir * (target_dist - follow_distance)

    camera.GetTransform().SetPosition(camera_pos + v_trans * pow(inertia , fps_ref * dts))


def update_follow_direction(camera: hg.Node):
    v = target_point - camera.GetTransform().GetPosition()
    camera.GetTransform().SetRotationMatrix(
        camera.GetTransform().GetWorld().GetRotationMatrix().LookAt(v, hg.Vector3.Up))


def update_camera_follow(camera: hg.Node, scene, terrain, dts):
    global target_point, target_node
    update_target_point(dts)
    update_follow_direction(camera)
    update_follow_translation(camera, scene, terrain, dts)