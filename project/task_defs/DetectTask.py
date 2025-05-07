
from ..transition_defs.ObjectDetectionTransition import ObjectDetectionTransition
from ..transition_defs.TimerTransition import TimerTransition
from ..transition_defs.HSVDetectionTransition import HSVDetectionTransition
from interface.Task import Task
import asyncio
import ast
import logging
from gabriel_protocol import gabriel_pb2


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class DetectTask(Task):

    def __init__(self, control, data, task_id, trigger_event_queue, task_args):
        super().__init__(control, data, task_id, trigger_event_queue, task_args)
       
        
    def create_transition(self):
        
        logger.info(f"**************Detect Task {self.task_id}: create transition! **************\n")
        logger.info(self.transitions_attributes)
        args = {
            'task_id': self.task_id,
            'trans_active': self.trans_active,
            'trans_active_lock': self.trans_active_lock,
            'trigger_event_queue': self.trigger_event_queue
        }
        
        # triggered event
        if ("timeout" in self.transitions_attributes):
            logger.info(f"**************Detect Task {self.task_id}:  timer transition! **************\n")
            timer = TimerTransition(args, self.transitions_attributes["timeout"])
            timer.daemon = True
            timer.start()
            
        if ("object_detection" in self.transitions_attributes):
            logger.info(f"**************Detect Task {self.task_id}:  object detection transition! **************\n")
            self.data.clearResults("openscout-object")
            object_trans = ObjectDetectionTransition(args, self.transitions_attributes["object_detection"], self.data)
            object_trans.daemon = True
            object_trans.start()

        if ("hsv_detection" in self.transitions_attributes):
            logger.info(f"**************Detect Task {self.task_id}:  hsv detection transition! **************\n")
            self.data.clearResults("openscout-object")
            hsv = HSVDetectionTransition(args, self.transitions_attributes["hsv_detection"], self.data)
            hsv.daemon = True
            hsv.start()
    
    @Task.call_after_exit
    async def run(self):
        # init the data
        logger.info("test, for pullin3")
        model = self.task_attributes["model"]
        lower_bound = self.task_attributes["lower_bound"]
        upper_bound = self.task_attributes["upper_bound"]
        self.control.configure_compute(model, lower_bound, upper_bound)
        self.create_transition()
        # try:
        logger.info(f"**************Detect Task {self.task_id}: hi this is detect task {self.task_id}**************\n")
        coords = ast.literal_eval(self.task_attributes["coords"])
        await self.control.setGimbalPose(0.0, float(self.task_attributes["gimbal_pitch"]), 0.0)
        for dest in coords:
            lng = dest["lng"]
            lat = dest["lat"]
            alt = dest["alt"]
            logger.info(f"**************Detect Task {self.task_id}: Move **************\n")
            logger.info(f"**************Detect Task {self.task_id}: move to {lat}, {lng}, {alt}**************\n")
            await self.control.moveTo(lat, lng, alt)
            await asyncio.sleep(1)

        logger.info(f"**************Detect Task {self.task_id}: Done**************\n")


