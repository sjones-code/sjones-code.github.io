# Run from the repo root: python3 shrink_images.py
# Shrinks oversized images IN PLACE, keeping the same filenames, so the site loads fast.
import os
from PIL import Image
for r,_,fs in os.walk('assets'):
    for f in fs:
        p=os.path.join(r,f)
        if not f.lower().endswith(('.png','.jpg','.jpeg')) or os.path.getsize(p)<800_000: continue
        im=Image.open(p); cap=512 if f=='bubble.png' else 1800
        if max(im.size)>cap: im.thumbnail((cap,cap))
        if f.lower().endswith('.png'): im.save(p,optimize=True)
        else: im.convert('RGB').save(p,quality=82,optimize=True)
        print('shrunk',p,round(os.path.getsize(p)/1e6,2),'MB')
