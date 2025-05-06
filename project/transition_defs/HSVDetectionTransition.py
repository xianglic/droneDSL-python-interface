from json import JSONDecodeError
import json
import logging
from venv import logger
from interface.Transition import Transition

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class HSVDetectionTransition(Transition):
    def __init__(self, args, target,  data):
        super().__init__(args)
        self.stop_signal = False
        self.target = target
        self.data = data
        
    def stop(self):
        self.stop_signal = True
    
    def run(self):
        self._register()
        while not self.stop_signal:
            result = self.data.get_compute_result("openscout-object")
            if (result != None):
                try:
                    json_string = result
                    json_data = json.loads(json_string)
                    for item in json_data:
                        class_attribute = item['class']
                        hsv_filter = item['hsv_filter']
                        if (class_attribute == self.target and hsv_filter):
                                logger.info(f"**************Transition: Task {self.task_id}: detect condition met! {class_attribute}**************\n")
                                self._trigger_event("hsv_detection")
                                break
                except JSONDecodeError as e:
                    logger.error(f'Error decoding json: {json_string}')
                except Exception as e:
                    logger.info(e)
      
        self._unregister()
  
    
