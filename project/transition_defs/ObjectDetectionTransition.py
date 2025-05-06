from json import JSONDecodeError
import json
import logging
import time
from venv import logger
from interface.Transition import Transition
from gabriel_protocol import gabriel_pb2

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class ObjectDetectionTransition(Transition):
    def __init__(self, args, target, data):
        super().__init__(args)
        self.stop_signal = False
        self.target =target
        self.data = data
        
    def stop(self):
        self.stop_signal = True
    
    def run(self):
        self._register()
        time.sleep(4)
        self.data.clear_compute_result("openscout-object")
        while not self.stop_signal:
            # get result
            result = self.data.get_compute_result("openscout-object")
            if (result != None):
                logger.info(f"**************Transition:  Task {self.task_id}: detected payload! {result}**************\n")
                try:
                    # Decode the payload from bytes to string
                    json_string = result

                    # Parse the JSON string
                    json_data = json.loads(json_string)

                    # Access the 'class' attribute
                    class_attribute = json_data[0]['class']  # Adjust the indexing based on your JSON structure
                    logger.info(class_attribute)

                    if (class_attribute== self.target):
                            logger.info(f"**************Transition: Task {self.task_id}: detect condition met! {class_attribute}**************\n")
                            self._trigger_event("object_detection")
                            break
                except JSONDecodeError as e:
                    logger.error(f'Error decoding json: {json_string}')
                except Exception as e:
                    logger.info(e)
        # print("object stopping...\n")          
        self._unregister()
  
    