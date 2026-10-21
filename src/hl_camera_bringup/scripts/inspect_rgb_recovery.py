import argparse,sys
from pathlib import Path
sys.path.insert(0,str(Path('apps/yolopv2_standalone').resolve()))
import numpy as np
from PIL import Image
from lane_boundaries import BoundaryTracker,white_lane_evidence,draw_boundaries
parser=argparse.ArgumentParser(description='RGB-only diagnostic with an all-road mask, not live segmentation validation.')
parser.add_argument('image',type=Path)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
p=args.output
p.mkdir(parents=True,exist_ok=True)
rgb=np.array(Image.open(args.image).convert('RGB'))
road=np.ones((180,320),np.uint8)*255
lane=np.zeros_like(road)
t=BoundaryTracker()
e=t.update(road,lane,0,rgb=rgb)
print(e.state,e.left_state,e.right_state)
paint=white_lane_evidence(rgb,lane)
Image.fromarray(paint).save(p/'rgb_evidence.png')
out=rgb[:,:,::-1].copy()
draw_boundaries(out,e,(320,180))
Image.fromarray(out[:,:,::-1]).save(p/'offline.jpg')
