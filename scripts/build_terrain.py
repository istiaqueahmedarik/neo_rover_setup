#!/usr/bin/env python3
"""Download public USGS lidar-derived DEM; preserve metadata and exact metric crop."""
import hashlib, json
from pathlib import Path
import numpy as np
import requests
import rasterio
from rasterio.windows import from_bounds
from rasterio.enums import Resampling
from pyproj import Transformer
from PIL import Image
ROOT = Path(__file__).resolve().parents[1]
a = ROOT / 'assets'
a.mkdir(exist_ok=True)
if not (a/'usgs_catalog.json').exists():
    r=requests.get('https://tnmaccess.nationalmap.gov/api/v1/products',params={'bbox':'-110.793,38.406,-110.791,38.408','datasets':'Digital Elevation Model (DEM) 1 meter','max':10,'outputFormat':'JSON'},timeout=60);r.raise_for_status()
    (a/'usgs_catalog.json').write_text(json.dumps(r.json(),indent=2))
item=next(x for x in json.loads((a/'usgs_catalog.json').read_text())['items'] if '2018' in x['title'])
raw=a/'usgs_source.tif'
if not raw.exists():
    print('Downloading',item['downloadURL'],flush=True)
    import subprocess, sys
    raw.with_suffix('.part').touch(exist_ok=True)
    subprocess.run([sys.executable,str(ROOT/'scripts/resume_download.py')],check=True)
# Center is near the MDRS habitat. Local x east, y north, z relative to crop minimum.
with rasterio.open(raw) as src:
    cx,cy=Transformer.from_crs('EPSG:4326',src.crs,always_xy=True).transform(-110.7919,38.4073)
    # Pixel centers run exactly from -1000 to +1000 at 2 m spacing.
    bounds=(cx-1001,cy-1001,cx+1001,cy+1001)
    window=from_bounds(*bounds,transform=src.transform)
    if window.col_off<0 or window.row_off<0 or window.col_off+window.width>src.width or window.row_off+window.height>src.height:
        raise RuntimeError('Crop extends beyond source tile')
    z=src.read(1,window=window,out_shape=(1001,1001),resampling=Resampling.bilinear)
    if not np.isfinite(z).all() or (z==src.nodata).any(): raise RuntimeError('Invalid DEM')
    crs=str(src.crs)
# Row 0 is south; GeoTIFF row 0 is north.
z=np.flipud(z).copy(); minimum=float(z.min()); z-=minimum
np.save(a/'terrain.npy',z)
# Pick the least-sloped patch within 60 m of the actual map center.
gy,gx=np.gradient(z,2.0); slope=gx*gx+gy*gy
best=min((float(slope[j-2:j+3,i-2:i+3].max()),j,i) for j in range(470,531) for i in range(470,531))
_,j,i=best
meta={'source':item,'source_sha256':hashlib.sha256(raw.read_bytes()).hexdigest(),'crs':crs,'center_easting':cx,'center_northing':cy,'center_lon_lat':[-110.7919,38.4073],'size_m':[2000,2000],'grid_spacing_m':2,'grid_shape':[1001,1001],'absolute_min_elevation_m':minimum,'relief_m':float(z.max()),'spawn_xyz':[float(i*2-1000),float(j*2-1000),float(z[j,i]+0.48)],'source_resolution_m':1,'note':'Real lidar-derived bare-earth DEM resampled to 2 m; no raw lidar points. Texture and decorative rocks are synthetic. No vertical exaggeration.'}
(a/'terrain.json').write_text(json.dumps(meta,indent=2))
# Repeatable desert material; terrain elevations themselves remain unmodified.
rng=np.random.default_rng(17)
# Synthetic sediment colors and fixed hill shading reveal the measured landforms.
gy,gx=np.gradient(z,2.0)
normal=np.sqrt(1+gx*gx+gy*gy)
shade=np.clip((.8-.9*gx+.55*gy)/normal,.32,1.25)
t=z/z.max()
base=np.array([194,151,105])[None,None,:]*(1-t[...,None])+np.array([151,93,62])[None,None,:]*t[...,None]
bands=1+.055*np.sin(z*1.1)+.025*np.sin(z*3.1)
noise=rng.normal(0,2,(*z.shape,1))
rgb=np.clip(base*(.55+.55*shade[...,None])*bands[...,None]+noise,0,255).astype('uint8')
Image.fromarray(np.flipud(rgb)).save(a/'sand.png')
Image.fromarray(np.uint8((z/z.max())*255)).transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(a/'height_preview.png')
print(json.dumps({k:v for k,v in meta.items() if k!='source'},indent=2))
