"""TOP midpoint endpoint, fixed start and a single shared projected path."""
import unittest
from types import SimpleNamespace
import numpy as np
from virtual_guidance import GuidanceConfig
from top_midpoint_guidance import displayed_midpoint_guidance,_map

def pair(left=50.,right=230.,states=('observed','observed')):
    y=np.linspace(70,176,54)
    return SimpleNamespace(image_left=np.column_stack((left+.12*y,y)),
        image_right=np.column_stack((right+.16*y,y)),left_support=np.ones(len(y),bool),
        right_support=np.ones(len(y),bool),left_state=states[0],right_state=states[1])

class TopMidpointTests(unittest.TestCase):
    def setUp(self):
        self.p=np.array([[.5,0,0],[0,.45,0],[0,0,1.]])
        self.cfg=GuidanceConfig(enabled=True)
        self.valid=np.ones((480,640),bool)

    def estimate(self,b=None,p=None,valid=None):
        return displayed_midpoint_guidance(b or pair(),self.p if p is None else p,self.cfg,
            self.valid if valid is None else valid)

    def test_fixed_start_exact_endpoint_and_forward_start_tangent(self):
        r=self.estimate();self.assertTrue(r.valid,r.reason)
        np.testing.assert_allclose(r.path[0],[319*.5,179*.995],atol=1e-10)
        np.testing.assert_allclose(r.target_top,r.target_edges_top.mean(axis=0),atol=1e-9)
        np.testing.assert_allclose(r.path_top[-1],r.target_top,atol=1e-10)
        np.testing.assert_allclose(np.polyval(r.top_coefficients,r.path_top[:,1]),r.path_top[:,0],atol=1e-9)
        self.assertAlmostEqual(2*r.top_coefficients[0]*r.start_top[1]+r.top_coefficients[1],0.)
        np.testing.assert_allclose(_map(r.path_top,self.p),r.path,atol=1e-9)

    def test_midpoint_is_in_top_space_with_asymmetric_perspective(self):
        p=np.array([[.5,.02,0],[0,.45,0],[.00015,0,1.]])
        r=self.estimate(p=p);self.assertTrue(r.valid,r.reason)
        np.testing.assert_allclose(r.target_top,r.target_edges_top.mean(axis=0),atol=1e-8)
        image_start=_map(np.array([r.start_top]),p)[0]
        np.testing.assert_allclose(image_start,[159.5,178.105],atol=1e-9)
        for side,x in zip(('left','right'),r.target_edges_top):
            q=_map(np.array([x]),p)[0]
            slope=.12 if side=='left' else .16;offset=50 if side=='left' else 230
            self.assertAlmostEqual(q[0],offset+slope*q[1],places=7)

    def test_inferred_displayed_side_is_used_instead_of_another_width(self):
        r=self.estimate(pair(states=('inferred','observed')))
        self.assertFalse(r.valid);self.assertIsNotNone(r.visible_path)
        np.testing.assert_allclose(r.target_top,r.target_edges_top.mean(axis=0),atol=1e-9)
        shifted=self.estimate(pair(left=20,states=('inferred','observed')))
        self.assertLess(shifted.target_top[0],r.target_top[0])
        np.testing.assert_allclose(shifted.start_top,r.start_top)

    def test_no_fabricated_target_without_two_visible_edges(self):
        b=pair();b.image_left[:,0]=np.nan
        self.assertIsNone(self.estimate(b).visible_path)
        self.assertIsNone(self.estimate(valid=np.zeros_like(self.valid)).visible_path)

    def test_target_projection_telemetry_does_not_claim_image_quadratic(self):
        r=self.estimate();d=r.telemetry()
        self.assertEqual(d['guidance_path_geometry'],'top_quadratic')
        self.assertEqual(d['guidance_polynomial'],'projected_top_quadratic')
        self.assertEqual(d['guidance_coefficients'],[])
        np.testing.assert_allclose(d['guidance_target_top'],r.target_edges_top.mean(axis=0))
