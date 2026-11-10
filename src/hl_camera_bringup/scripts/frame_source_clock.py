"""Read the camera's bitmap source clock from an unscaled received frame."""
from pathlib import Path
import ast,re
import numpy as np
class SourceClock:
    def __init__(self):
        root=Path(__file__).resolve().parents[3]
        module=ast.parse((root/'apps/yolopv2_standalone/yolopv2_node.py').read_text())
        node=next(n.value for n in module.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FONT_5X7' for t in n.targets))
        font={ast.literal_eval(k):(('00000',)*7 if ast.literal_eval(k)==' ' else ast.literal_eval(v)) for k,v in zip(node.keys,node.values) if ast.literal_eval(k) in '0123456789.S '}
        self.templates={c:np.array([[v=='1' for v in row] for row in glyph],float) for c,glyph in font.items()}
    def read(self,image,y):
        chars=[];scores=[]
        for i in range(6):
            x=10+i*12;crop=image[y:y+14,x:x+10].astype(float)
            if crop.shape!=(14,10,3):return None
            yellow=(crop[:,:,2]>100)&(crop[:,:,1]>90)&(crop[:,:,0]<.65*np.minimum(crop[:,:,1],crop[:,:,2]))
            cells=yellow.reshape(7,2,5,2).mean(axis=(1,3))
            score,c=min((float(np.mean(abs(cells-template))),c) for c,template in self.templates.items())
            chars.append(c);scores.append(score)
        match=re.match(r'^(\d+\.\d)S',''.join(chars))
        if not match:return None
        score=float(np.mean(scores[:match.end()]))
        return (float(match.group(1)),score) if score<.15 else None
