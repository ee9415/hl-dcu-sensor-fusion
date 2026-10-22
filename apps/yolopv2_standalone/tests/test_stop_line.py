import unittest
from types import SimpleNamespace
import numpy as np
from lane_boundaries import BoundaryTracker,projection_for
from stop_line import StopLineTracker,draw_stop_line


class StopLineTests(unittest.TestCase):
    def setUp(self):
        tracker=BoundaryTracker()
        mask=np.zeros((81,241),bool)
        self.bounds=tracker.update_bev(np.linspace(0,25,81),np.linspace(-6,6,241),
                                      mask,mask,0,projection_for((180,320)))
        self.detector=StopLineTracker()
        self.blank=np.full((180,320,3),50,np.uint8)

    def bar(self):
        image=self.blank.copy();image[145:148,20:300]=230
        return image

    def test_confirm_hold_expire_and_render(self):
        for n in range(3):
            estimate=self.detector.update(self.bar(),self.bounds,n*.1)
            self.assertEqual(estimate.state,'candidate' if n<2 else 'detected')
        self.assertGreater(estimate.confidence,.7)
        image=self.blank.copy();draw_stop_line(image,estimate)
        self.assertTrue(np.any(np.all(image==(255,0,255),axis=2)))
        held=self.detector.update(self.blank,self.bounds,.3)
        self.assertEqual(held.state,'estimated')
        self.assertLess(held.confidence,estimate.confidence)
        expired=self.detector.update(self.blank,self.bounds,.7)
        self.assertEqual(expired.state,'unavailable')
        self.assertFalse(expired.telemetry()['stop_line_valid'])

    def test_crosswalk_vertical_paint_and_objects_are_not_confirmed(self):
        crosswalk=self.blank.copy()
        for y in (130,145,160):crosswalk[y:y+3,20:300]=230
        vertical=self.blank.copy();vertical[:,100:103]=230
        for image,boxes in ((crosswalk,()),(vertical,()),(self.bar(),
                            (SimpleNamespace(xmin=0,ymin=0,xmax=1,ymax=1),))):
            detector=StopLineTracker()
            for n in range(5):
                estimate=detector.update(image,self.bounds,n*.1,boxes)
                self.assertFalse(estimate.telemetry()['stop_line_valid'])

    def test_time_reset_clears_old_stop(self):
        for n in range(3):self.detector.update(self.bar(),self.bounds,n*.1)
        result=self.detector.update(self.blank,self.bounds,0)
        self.assertEqual(result.state,'unavailable')


if __name__=='__main__':unittest.main()
