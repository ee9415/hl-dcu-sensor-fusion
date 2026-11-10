"""Host-node integration without a camera or the RVC4 DepthAI runtime."""
from datetime import timedelta
import importlib
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np


class Frame:
    """depthai ImgFrame/NNData 등을 흉내 내는 가짜 프레임. 1920x1080 RGB/깊이를 0으로 돌려준다."""
    Type = types.SimpleNamespace(NV12=0, BGR888i=1)

    def __init__(self):
        self.data = None
        self.timestamp = timedelta(seconds=10)
        self.sequence = 7

    def setCvFrame(self, data, kind):
        self.data = data

    def getCvFrame(self):
        return np.zeros((1080, 1920, 3), np.uint8)

    def getFrame(self):
        return np.zeros((1080, 1920), np.uint16)

    def getTransformation(self):
        return None

    def getTimestamp(self):
        return self.timestamp

    def getSequenceNum(self):
        return self.sequence

    def setTimestamp(self, value):
        self.timestamp = value

    def setSequenceNum(self, value):
        self.sequence = value


class BoundaryPipelineTests(unittest.TestCase):
    def test_temporal_overlay_survives_display_changes_and_resets_on_source_epoch(self):
        """표시 설정(경계선 숨김, raw_rgb)이 바뀌어도 경계선 추적은 그대로 유지되고, source epoch가 바뀔 때만 초기화된다."""
        class Host:
            def createOutput(self, **kwargs):
                return MagicMock()
        dai = types.SimpleNamespace(node=types.SimpleNamespace(HostNode=Host,ThreadedHostNode=Host),
            Node=MagicMock(),DatatypeEnum=MagicMock(),ImgFrame=Frame,NNData=Frame,
            ImgDetection=Frame,ImgDetections=Frame,SpatialImgDetections=Frame,
            LengthUnit=MagicMock(),Clock=types.SimpleNamespace(now=lambda:timedelta(seconds=10)))
        with patch.dict(sys.modules,{'depthai':dai}):
            node = importlib.import_module('yolopv2_node')
        road = np.full((180,320),255,np.uint8)
        lane = np.zeros_like(road)
        for y in range(82,176):
            for x in (int(160-(y-65)*.9),int(160+(y-65)*.9)):
                lane[y,x-1:x+2] = 255
        with (patch.object(node,'TelemetryServer') as server,
              patch.object(node,'decode_heads'),
              patch.object(node,'non_max_suppression',return_value=[]),
              patch.object(node,'segmentation_masks',return_value=(road,lane))):
            processor = node.Yolopv2Postprocess()
            processor._tensor = MagicMock()
            processor.video_input = MagicMock()
            metadata = dict(input_source='pc',source_epoch=1,processing_mode='full',processing_epoch=0)
            processor.video_input.metadata.return_value = metadata
            reference = node.Yolopv2Postprocess()
            reference._tensor = MagicMock()
            reference.video_input = MagicMock()
            reference.video_input.metadata.return_value = metadata.copy()
            for i in range(10):
                frame = Frame();frame.timestamp = timedelta(seconds=10+i*.05);frame.sequence = i
                processor.process(frame,frame,frame)
                reference.process(frame,frame,frame)
            # 10프레임 정상 처리 후 왼쪽 2차식을 저장해 둔다.
            # 미검출 쪽은 간격 보정으로 움직일 수 있다. 표시 설정이 다른
            # 대조 처리기와 추적 결과가 같아야 한다.
            lane[:,:160] = 0
            metadata.update(boundary_visible=False,lane_overlay_mode='raw_rgb')
            frame.timestamp = timedelta(seconds=10.5)
            processor.process(frame,frame,frame)
            self.assertTrue(processor._lane_quadratics.left.is_virtual)
            reference.process(frame,frame,frame)
            np.testing.assert_allclose(processor._lane_quadratics.left.coefficients,
                                       reference._lane_quadratics.left.coefficients,atol=1e-7)
            metadata.update(boundary_visible=True,lane_overlay_mode='normal')
            frame.timestamp = timedelta(seconds=10.55)
            processor.process(frame,frame,frame)
            self.assertTrue(processor._lane_quadratics.left.is_virtual)
            self.assertTrue(server.return_value.publish.call_args.args[0]['left_boundary_valid'])
            # 입력 소스가 바뀌면(epoch 증가) 추적 이력을 버린다.
            metadata['source_epoch'] = 2
            frame.timestamp = timedelta(seconds=10.6)
            processor.process(frame,frame,frame)
            self.assertIsNone(processor._lane_quadratics.left)
            self.assertFalse(server.return_value.publish.call_args.args[0]['left_boundary_valid'])

    def setUp(self):
        # These assertions specify the legacy/off rendering and telemetry.
        """기본적으로 가상 가이던스를 끈 상태(기존 렌더링/텔레메트리 기준)로 테스트한다."""
        from virtual_guidance import GuidanceConfig
        patcher = patch.object(GuidanceConfig, 'load', return_value=GuidanceConfig(enabled=False))
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_guidance_enabled_publishes_separate_path_and_basic_bypasses_it(self):
        """가이던스를 켜면 기존 중심선과 별도로 경로를 발행한다.

        - 경계선 표시를 꺼도 추적기·경로는 그대로이고 화면에서만 숨긴다.
        - 미리보기 임계값은 RGB 결과만 바꾸고 인지 마스크/경로에는 영향이 없다.
        - basic 모드에서는 가이던스 계산 자체를 건너뛴다.
        """
        from virtual_guidance import GuidanceConfig
        node = sys.modules.get('yolopv2_node')
        if node is None:
            class Host:
                def createOutput(self, **kwargs):
                    return MagicMock()
            dai=types.SimpleNamespace(node=types.SimpleNamespace(HostNode=Host,ThreadedHostNode=Host),
                Node=MagicMock(),DatatypeEnum=MagicMock(),ImgFrame=Frame,NNData=Frame,
                ImgDetection=Frame,ImgDetections=Frame,SpatialImgDetections=Frame,
                LengthUnit=MagicMock(),Clock=types.SimpleNamespace(now=lambda:timedelta(seconds=10)))
            with patch.dict(sys.modules,{'depthai':dai}):
                node=importlib.import_module('yolopv2_node')
        road = np.full((180,320),255,np.uint8)
        with patch.object(node.GuidanceConfig,'load',return_value=GuidanceConfig(enabled=True)), \
             patch.object(node,'TelemetryServer') as server, \
             patch.object(node,'decode_heads'), \
             patch.object(node,'non_max_suppression',return_value=[]), \
             patch.object(node,'segmentation_masks',return_value=(road,np.zeros_like(road))):
            processor=node.Yolopv2Postprocess()
            processor._tensor=MagicMock()
            processor.process(Frame(),Frame(),Frame())
            payload=server.return_value.publish.call_args.args[0]
            self.assertTrue(payload['guidance_valid'], payload['guidance_reason'])
            self.assertEqual(payload['guidance_source'],'road')
            self.assertFalse(payload['centerline_valid'])
            self.assertIsNone(payload['coefficients'])
            # No lane mask means no skeleton, even when guidance is valid.
            self.assertFalse(np.any(np.all(processor.result.send.call_args.args[0].data==(255,255,0),axis=2)))
            processor.video_input=MagicMock()
            processor.video_input.metadata.return_value=dict(input_source='camera',source_epoch=1,
                processing_mode='full',processing_epoch=0,boundary_visible=True)
            processor.process(Frame(),Frame(),Frame())
            tracker=processor._boundary_tracker
            guidance=processor._guidance
            before=server.return_value.publish.call_args.args[0]
            processor.video_input.metadata.return_value['boundary_visible']=False
            processor.process(Frame(),Frame(),Frame())
            hidden=processor.result.send.call_args.args[0].data
            self.assertIs(processor._boundary_tracker,tracker)
            self.assertIs(processor._guidance,guidance)
            after=server.return_value.publish.call_args.args[0]
            self.assertEqual(before['guidance_path_image'],after['guidance_path_image'])
            self.assertFalse(after['boundary_visible'])
            self.assertFalse(np.any(np.all(hidden==(0,150,255),axis=2)))
            self.assertFalse(np.any(np.all(hidden==(255,255,0),axis=2)))
            # Preview settings alter the RGB result only, not perception masks
            # or the existing path/trackers. All callbacks use the same input.
            # 미리보기 임계값 0.2 / 0.9 두 번 처리해서 결과를 비교한다.
            preview_paths=[];mask_frames=[];selected=[]
            processor._tensor=MagicMock(return_value=np.full((1,1,320,320),.7,np.float32))
            for threshold in (.2,.9):
                processor.video_input.metadata.return_value.update(lane_overlay_mode='lane_mask',lane_overlay_threshold=threshold)
                processor.process(Frame(),Frame(),Frame())
                p=server.return_value.publish.call_args.args[0]
                preview_paths.append(p['guidance_path_image'])
                mask_frames.append(processor.lane.send.call_args.args[0].data.copy())
                selected.append(p['lane_preview_selected_pixels'])
                self.assertIs(processor._boundary_tracker,tracker)
                self.assertEqual(p['lane_perception_threshold'],.5)
                self.assertIsNone(p['lane_overlay_error'])
            self.assertEqual(preview_paths[0],preview_paths[1])
            np.testing.assert_array_equal(mask_frames[0],mask_frames[1])
            self.assertEqual(selected,[320*180,0])
            processor.video_input=MagicMock()
            processor.video_input.metadata.return_value=dict(input_source='camera',source_epoch=1,
                processing_mode='basic',processing_epoch=1)
            with patch.object(processor._guidance,'update',side_effect=AssertionError('guidance ran in basic')):
                processor.process(Frame(),Frame(),Frame())
            self.assertEqual(server.return_value.publish.call_args.args[0]['guidance_reason'],'basic_mode')

    def test_result_bev_and_telemetry_use_boundary_pair(self):
        """결과 영상, BEV, 텔레메트리가 같은 경계선 쌍을 사용하는지 통합 확인한다.

        - 윤곽 표시/경계선 숨김 조합별 그리기 색상
        - 한쪽 차선만 있을 때 반대쪽을 만들어 내지 않음
        - PC 입력이면 깊이 거리를 쓰지 않고 추적을 초기화
        - basic 모드는 무거운 알고리즘을 아예 실행하지 않음
        """
        class Host:
            def createOutput(self, **kwargs):
                return MagicMock()

        dai = types.SimpleNamespace(node=types.SimpleNamespace(HostNode=Host, ThreadedHostNode=Host),
                                    Node=MagicMock(), DatatypeEnum=MagicMock(), ImgFrame=Frame,
                                    ImgDetections=Frame, SpatialImgDetections=Frame,
                                    LengthUnit=MagicMock(),
                                    Clock=types.SimpleNamespace(now=lambda: timedelta(seconds=10)))
        # Annotations are evaluated during import in yolopv2_node.py.
        dai.NNData = type('NNData', (), {})
        dai.ImgDetection = type('ImgDetection', (), {})
        with patch.dict(sys.modules, {'depthai': dai}):
            node = importlib.import_module('yolopv2_node')
        road = np.full((180, 320), 255, np.uint8)
        from lane_boundaries import projection_for, transform, draw_polyline
        mask_rgb = np.zeros((180,320,3),np.uint8)
        for y in range(78,176):
            for x in (160-(y-65)*.9,160+(y-65)*.9):
                x=int(x)
                mask_rgb[y,x-1:x+2]=230
        lane = mask_rgb[:,:,0]
        source = np.repeat(np.repeat(mask_rgb,6,axis=0),6,axis=1)
        with patch.object(node, 'TelemetryServer') as server, \
             patch.object(node, 'decode_heads'), \
             patch.object(node, 'non_max_suppression', return_value=[]), \
             patch.object(node, 'segmentation_masks', return_value=(road, lane)), \
             patch.object(Frame, 'getCvFrame', return_value=source):
            processor = node.Yolopv2Postprocess()
            processor._tensor = MagicMock()
            processor.process(Frame(), Frame(), Frame())
            payload = server.return_value.publish.call_args.args[0]
            self.assertTrue(payload['boundary_valid'])
            self.assertFalse(payload['centerline_valid'])
            self.assertIsNone(payload['coefficients'])
            self.assertEqual(payload['turn_mode'], 'disabled')
            self.assertEqual(payload['frame_seq'], 7)
            self.assertIsNone(payload['corridor_width_m'])
            self.assertIsNone(payload['normal_width_error_dev'])
            self.assertEqual(payload['boundary_model'],'persistent_skeleton')
            self.assertGreater(len(payload['left_boundary_image']),12)
            self.assertGreater(len(payload['right_boundary_image']),12)
            result = processor.result.send.call_args.args[0]
            bev = processor.bev.send.call_args.args[0]
            self.assertEqual(result.data.shape, (540,960,3))
            self.assertEqual(bev.data.shape, (480,640,3))
            self.assertEqual(result.getSequenceNum(), 7)
            self.assertEqual(bev.getTimestamp(), timedelta(seconds=10))
            self.assertTrue(np.any(np.all(result.data==(255,0,255),axis=2)))
            self.assertFalse(np.any(np.all(result.data==(0,255,255),axis=2)))
            self.assertFalse(np.any(np.all(result.data==(255,255,0),axis=2)))
            self.assertFalse(np.any(np.all(bev.data==(0,255,255),axis=2)))
            self.assertFalse(np.any(np.all(bev.data==(255,255,0),axis=2)))
            processor.video_input=MagicMock()
            processor.video_input.metadata.return_value=dict(input_source='camera',source_epoch=0,
                processing_mode='full',processing_epoch=0,boundary_visible=True,contour_visible=True)
            processor.process(Frame(),Frame(),Frame())
            self.assertTrue(np.any(np.all(processor.result.send.call_args.args[0].data==(0,255,255),axis=2)))
            tracker=processor._boundary_tracker
            processor.video_input.metadata.return_value['boundary_visible']=False
            processor.process(Frame(),Frame(),Frame())
            contour_only=processor.result.send.call_args.args[0].data
            self.assertTrue(np.any(np.all(contour_only==(0,255,255),axis=2)))
            self.assertFalse(np.any(np.all(contour_only==(255,0,255),axis=2)))
            self.assertFalse(np.any(np.all(contour_only==(255,255,0),axis=2)))
            self.assertIs(processor._boundary_tracker,tracker)
            # lane 마스크가 비어 있으면(RGB만 있음) 경계선 쌍은 유효하지 않다.
            processor.video_input=None
            with patch.object(node,'segmentation_masks',return_value=(road,np.zeros_like(lane))):
                rgb_only=node.Yolopv2Postprocess()
                rgb_only._tensor=MagicMock()
                rgb_only.process(Frame(),Frame(),Frame())
            self.assertFalse(server.return_value.publish.call_args.args[0]['boundary_pair_valid'])
            # Full mode now tracks actual mask contours, never RGB-only paint
            # or an unobserved side fabricated from a default image width.
            partial_lane=lane.copy()
            partial_lane[:,160:]=0
            partial_source=source.copy()
            partial_source[:,960:]=0
            with patch.object(node,'segmentation_masks',return_value=(road,partial_lane)), \
                 patch.object(Frame,'getCvFrame',return_value=partial_source):
                partial=node.Yolopv2Postprocess()
                partial._tensor=MagicMock()
                partial.process(Frame(),Frame(),Frame())
            payload=server.return_value.publish.call_args.args[0]
            self.assertTrue(payload['boundary_valid'])
            self.assertTrue(payload['left_boundary_valid'])
            self.assertFalse(payload['right_boundary_valid'])
            self.assertFalse(payload['boundary_pair_valid'])
            self.assertFalse(payload['centerline_valid'])
            result=partial.result.send.call_args.args[0]
            self.assertTrue(np.any(np.all(result.data==(0,220,255),axis=2)))
            self.assertEqual(payload['right_boundary_state'],'unavailable')
            self.assertFalse(np.any(np.all(result.data==(114,81,0),axis=2)))
            # Even if a real-looking stereo image reaches the callback, PC
            # provenance must suppress its distances and reset old tracking.
            old_tracker=processor._boundary_tracker
            processor.video_input=MagicMock()
            processor.video_input.metadata.return_value=dict(input_source='pc',
                source_epoch=2,source_frame_index=20,source_media_time_s=2.)
            with patch.object(Frame,'getFrame',return_value=np.full((180,320),2000,np.uint16)), \
                 patch.object(node,'non_max_suppression',return_value=np.array([[80,100,180,220,.9,0]])), \
                 patch.object(node,'estimate_spatial_from_depth',wraps=node.estimate_spatial_from_depth) as spatial:
                processor.process(Frame(),Frame(),Frame())
            self.assertIsNot(old_tracker,processor._boundary_tracker)
            self.assertFalse(spatial.call_args.args[0].any())
            payload=server.return_value.publish.call_args.args[0]
            self.assertFalse(payload['depth_available'])
            self.assertEqual(payload['source_frame_index'],20)
            self.assertFalse(payload['objects'][0]['depth_valid'])
            # Basic must bypass the expensive algorithms, not just hide lines.
            processor.video_input.metadata.return_value=dict(input_source='pc',source_epoch=2,
                processing_mode='basic',processing_epoch=1)
            with patch.object(node.ImageBoundaryTracker,'update_contours',side_effect=AssertionError('boundary ran')), \
                 patch.object(node.StopLineTracker,'update',side_effect=AssertionError('stop ran')), \
                 patch.object(node,'render_boundary_bev',side_effect=AssertionError('BEV ran')):
                processor.process(Frame(),Frame(),Frame())
            basic_tracker=processor._boundary_tracker
            payload=server.return_value.publish.call_args.args[0]
            self.assertEqual(payload['processing_mode'],'basic')
            self.assertTrue(np.any(processor.bev.send.call_args.args[0].data[100:]))
            self.assertEqual(payload['boundary_state'],'disabled')
            self.assertEqual(payload['stop_line_state'],'disabled')
            self.assertFalse(payload['boundary_valid'])
            # Missing raw scores must retain the RGB/road overlay, never fake
            # confidence by painting the binary mask a uniform cyan.
            self.assertFalse(np.any(np.all(processor.result.send.call_args.args[0].data==(184,220,184),axis=2)))
            processor.video_input.metadata.return_value=dict(input_source='pc',source_epoch=2,
                processing_mode='full',processing_epoch=2)
            processor.process(Frame(),Frame(),Frame())
            self.assertIsNot(basic_tracker,processor._boundary_tracker)
            self.assertEqual(server.return_value.publish.call_args.args[0]['processing_mode'],'full')
