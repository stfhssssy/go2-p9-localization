#!/usr/bin/env python3
"""Apply only the checked equivalent centroid export optimization to this image."""
import argparse,hashlib,json
from pathlib import Path
from test_mapping_points import OLD,NEW
p=argparse.ArgumentParser();p.add_argument('--provenance',required=True);a=p.parse_args()
out=Path(a.provenance);out.mkdir(parents=True,exist_ok=True)
paths=[Path('/opt/p9_ws/src/go2_p9/go2_p9/mapping_core.py'),Path('/opt/p9_ws/install/go2_p9/lib/python3.12/site-packages/go2_p9/mapping_core.py')]
records=[];seen=set()
for path in paths:
 path=path.resolve()
 if path in seen:continue
 seen.add(path);before=path.read_text()
 if before.count(OLD)==1:
  after=before.replace(OLD,NEW,1)
 elif before.count(NEW)==1:after=before
 else:raise SystemExit('Unexpected mapping implementation: '+str(path))
 compile(after,str(path),'exec')
 if before!=after:
  (out/f'mapping_core_{len(records)}.before.py').write_text(before)
  path.write_text(after)
 records.append(dict(path=str(path),changed=before!=after,before_sha256=hashlib.sha256(before.encode()).hexdigest(),after_sha256=hashlib.sha256(after.encode()).hexdigest()))
(out/'mapping-patch.json').write_text(json.dumps(records,indent=2)+'\n')
print(json.dumps(records,indent=2))
