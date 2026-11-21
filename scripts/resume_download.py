"""Resume an interrupted source download with validated HTTP range requests."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json,requests,time
A=Path(__file__).resolve().parents[1]/'assets'
u=next(x for x in json.loads((A/'usgs_catalog.json').read_text())['items'] if '2018' in x['title'])['downloadURL']
p=A/'usgs_source.part'
start=p.stat().st_size
r=requests.get(u,headers={'Range':'bytes=0-0'},timeout=30);r.raise_for_status()
assert r.status_code==206
size=int(r.headers['Content-Range'].split('/')[-1])
ranges=[(i,min(i+8*1024*1024,size)-1) for i in range(start,size,8*1024*1024)]
def fetch(pair):
    lo,hi=pair;f=A/f'download_{lo}.chunk'
    for attempt in range(4):
        try:
            # Unique query prevents intermediate caches reusing a different range response.
            with requests.get(u+f'?part={lo}',headers={'Range':f'bytes={lo}-{hi}'},stream=True,timeout=(20,60)) as r:
                r.raise_for_status();assert r.status_code==206 and r.headers['Content-Range'].startswith(f'bytes {lo}-{hi}/'),r.headers
                with f.open('wb') as out:
                    for block in r.iter_content(1024*1024):out.write(block)
            assert f.stat().st_size==hi-lo+1
            print('Fetched',lo,hi,flush=True);return f
        except Exception:
            if attempt==3:raise
            time.sleep(1)
with ThreadPoolExecutor(max_workers=6) as pool:
    files=list(pool.map(fetch,ranges))
with p.open('ab') as out:
    for f in files:
        with f.open('rb') as inp:
            while block:=inp.read(1024*1024):out.write(block)
        f.unlink()
assert p.stat().st_size==size
p.rename(A/'usgs_source.tif')
print('Downloaded',size,flush=True)
