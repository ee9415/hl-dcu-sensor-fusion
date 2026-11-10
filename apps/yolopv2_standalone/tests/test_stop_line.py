"""stop_line.StopLineTracker(정지선 검출/추적) 테스트."""
import unittest
from types import SimpleNamespace
import numpy as np
from lane_boundaries import BoundaryTracker,projection_for
from stop_line import StopLineTracker,draw_stop_line


class StopLineTests(unittest.TestCase):
    def setUp(self):
        """빈 BEV 마스크로 기본 경계선 추정값을 만들고, 회색 배경 이미지를 준비한다."""
        tracker=BoundaryTracker()
        mask=np.zeros((81,241),bool)
        self.bounds=tracker.update_bev(np.linspace(0,25,81),np.linspace(-6,6,241),
                                      mask,mask,0,projection_for((180,320)))
        self.detector=StopLineTracker()
        self.blank=np.full((180,320,3),50,np.uint8)

    def bar(self):
        """y=145~147에 가로 흰 띠(정지선 후보)를 그린 이미지."""
        image=self.blank.copy();image[145:148,20:300]=230
        return image

    def test_confirm_hold_expire_and_render(self):
        """3프레임 연속 관측 시 candidate → detected로 확정되고 그려진다. 사라지면 잠시 estimated로 유지 후 unavailable로 만료된다."""
        for n in range(3):
            estimate=self.detector.update(self.bar(),self.bounds,n*.1)
            self.assertEqual(estimate.state,'candidate' if n<2 else 'detected')
        self.assertGreater(estimate.confidence,.7)
        image=self.blank.copy();draw_stop_line(image,estimate)
        # 정지선은 마젠타색으로 그려진다.
        self.assertTrue(np.any(np.all(image==(255,0,255),axis=2)))
        held=self.detector.update(self.blank,self.bounds,.3)
        self.assertEqual(held.state,'estimated')
        self.assertLess(held.confidence,estimate.confidence)
        expired=self.detector.update(self.blank,self.bounds,.7)
        self.assertEqual(expired.state,'unavailable')
        self.assertFalse(expired.telemetry()['stop_line_valid'])

    def test_crosswalk_vertical_paint_and_objects_are_not_confirmed(self):
        """횡단보도(여러 줄), 세로 페인트, 객체 박스와 겹치는 띠는 정지선으로 확정하지 않는다."""
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
        """타임스탬프가 뒤로 가면(재생 리셋) 이전 정지선 상태를 지운다."""
        for n in range(3):self.detector.update(self.bar(),self.bounds,n*.1)
        result=self.detector.update(self.blank,self.bounds,0)
        self.assertEqual(result.state,'unavailable')


if __name__=='__main__':unittest.main()
