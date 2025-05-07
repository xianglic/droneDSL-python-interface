import asyncio
from json import JSONDecodeError
import sys
import json
import numpy as np
import math
import time
from ..transition_defs.TimerTransition import TimerTransition
from interface.Task import Task
import logging
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class TrackTask(Task):

    def __init__(self, control, data, task_id, trigger_event_queue, task_args):
        super().__init__(control, data, task_id, trigger_event_queue, task_args)
        self.image_res = (1280, 720)
        self.pixel_center = (self.image_res[0] / 2, self.image_res[1] / 2)
        self.HFOV = 69
        self.VFOV = 43
        self.target_lost_duration = 10
        self.leash_length = 15.0

    def create_transition(self):
        args = {
            'task_id': self.task_id,
            'trans_active': self.trans_active,
            'trans_active_lock': self.trans_active_lock,
            'trigger_event_queue': self.trigger_event_queue
        }

        if ("timeout" in self.transitions_attributes):
            timer = TimerTransition(args, self.transitions_attributes["timeout"])
            timer.daemon = True
            timer.start()

    ''' Helper Functions '''
    def target_bearing(self, origin, destination):
        lat1, lon1 = origin
        lat2, lon2 = destination

        rlat1 = math.radians(lat1)
        rlat2 = math.radians(lat2)
        rlon1 = math.radians(lon1)
        rlon2 = math.radians(lon2)
        dlon = math.radians(lon2-lon1)

        b = math.atan2(math.sin(dlon)*math.cos(rlat2),math.cos(rlat1)*math.sin(rlat2)\
                -math.sin(rlat1)*math.cos(rlat2)*math.cos(dlon))
        bd = math.degrees(b)
        br, bn = divmod(bd+360, 360)

        return bn

    def find_intersection(self, target_dir, target_insct):
        plane_pt = np.array([0, 0, 0])
        plane_norm = np.array([0, 0, 1])

        if plane_norm.dot(target_dir).all() == 0:
            return None

        t = (plane_norm.dot(plane_pt) - plane_norm.dot(target_insct)) / plane_norm.dot(target_dir)
        return target_insct + (t * target_dir)

    async def estimate_distance(self, yaw, pitch):
        telemetry = await self.data.get_telemetry()
        alt = telemetry["global_position"]["relative_altitude"]
        gimbal = telemetry["gimbal_pose"]["pitch"]

        vf = [0, 1, 0]
        r = R.from_euler('ZYX', [yaw, 0, pitch + gimbal], degrees=True)
        target_dir = r.as_matrix().dot(vf)
        target_vec = self.find_intersection(target_dir, np.array([0, 0, alt]))

        logger.info(f"[TrackTask]: Distance estimation: {np.linalg.norm(target_vec)}")
        leash_vec = self.leash_length * (target_vec / np.linalg.norm(target_vec))
        logger.info(f"[TrackTask]: Error vector length: {np.linalg.norm(leash_vec - target_vec)}")
        return leash_vec - target_vec

    async def error(self, box):
        target_x_pix = self.image_res[0] - int(((box[3] - box[1]) / 2.0) + box[1])
        target_y_pix = self.image_res[1] - int(((box[2] - box[0]) / 2.0) + box[0])
        target_yaw_angle = ((target_x_pix - self.pixel_center[0]) / self.pixel_center[0]) * (self.HFOV / 2)
        target_pitch_angle = ((target_y_pix - self.pixel_center[1]) / self.pixel_center[1]) * (self.VFOV / 2)
        target_bottom_pitch_angle = (((self.image_res[1] - box[2]) - self.pixel_center[1]) \
                / self.pixel_center[1]) * (self.VFOV / 2)

        yaw_error = -1 * target_yaw_angle
        gimbal_error = target_pitch_angle
        follow_error = await self.estimate_distance(target_yaw_angle, target_bottom_pitch_angle)

        return (follow_error, yaw_error, gimbal_error)

    def clamp(self, value, minimum, maximum):
        return np.clip(value, minimum, maximum)

    async def actuate(self, follow_vel, yaw_vel,\
            gimbal_offset, orbit_speed, descent_speed):
        telemetry = await self.data.get_telemetry()
        prev_gimbal = telemetry["gimbal_pose"]["pitch"]
        await self.control.set_velocity_body(follow_vel, orbit_speed, -1 * descent_speed, yaw_vel)
        #await self.control.set_gimbal_pose(gimbal_offset + prev_gimbal)

    ''' Main Logic '''
    @Task.call_after_exit
    async def run(self):
        # init the data
        model = self.task_attributes["model"]
        lower_bound = self.task_attributes["lower_bound"]
        upper_bound = self.task_attributes["upper_bound"]
        await self.control.configure_compute(model, lower_bound, upper_bound)

        target = self.task_attributes["class"]
        altitude = 10
        descent_speed = 5
        orbit_speed = 2
        follow_speed = 2
        yaw_speed = 1
        gimbal_offset = 0

        self.create_transition()
        last_seen = None
        descended = False
        while True:
            logger.info("Awaiting compute result")
            response = await self.data.get_compute_result("openscout-object")
            result = response.cpt.result
            if len(result) == 0:
                continue

            detections = result[0].generic_result
            logger.info(f"{detections=}")
            try:
                detections = json.loads(detections)
            except Exception as e:
                logger.error(e)
                raise
            if last_seen is not None and \
                    int(time.time() - last_seen)  > self.target_lost_duration:
                # If we have not found the target in N seconds trigger the done transition
                logger.info(f"Breaking; {self.target_lost_duration=} {last_seen=} {time.time()=}")
                break
            telemetry = await self.data.get_telemetry()
            global_pos = telemetry["global_position"]
            if global_pos["relative_altitude"] <= altitude:
                descended = True

            box = None
            # Return the first instance found of the target class
            for det in detections:
                #if det["class"] == 'bench':# and det["hsv_filter"]:
                #    box = det["box"]
                #    last_seen = time.time()
                #    break
                logger.info(f"Now following {det['class']}")
                box = det["box"]
                last_seen = time.time()
                break

            # Found an instance of target, start tracking!
            if box is not None:
                try:
                    follow_error, yaw_error, gimbal_error = await self.error(box)
                except Exception as e:
                    logger.error(f"Failed to calculate error, reason: {e}")
                try:
                    follow_vel = self.clamp(follow_error, -1 * follow_speed, follow_speed)
                    yaw_vel = self.clamp(yaw_error, -1 * yaw_speed, yaw_speed)
                except Exception as e:
                    logger.error(f"Failed to clamp, reason: {e}")
                try:
                    if descended:
                        await self.actuate(0.0, yaw_vel, gimbal_offset, 0.0, descent_speed)
                    else:
                        await self.actuate(follow_vel, yaw_vel, gimbal_offset, orbit_speed, 0.0)
                except Exception as e:
                    logger.error(f"Failed to actuate, reason: {e}")
                logger.info("Successfully actuated")

            await asyncio.sleep(0.05)

