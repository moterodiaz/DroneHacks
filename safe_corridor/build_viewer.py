"""Plan both courses and bake them into one self-contained viewer page.

    py build_viewer.py      -> out/safe_corridor.html
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
COURSES = {'taped': 'routes/taped_2026-11-08.yaml', 'example': 'routes/example.yaml'}

data = {}
for key, route in COURSES.items():
    out = os.path.join(HERE, 'out', key)
    subprocess.run([sys.executable, os.path.join(HERE, 'corridor.py'), os.path.join(HERE, route), '--out', out], check=True)
    with open(out + '_view.json') as f:
        data[key] = json.load(f)
with open(os.path.join(HERE, 'viewer_template.html'), encoding='utf-8') as f:
    page = f.read().replace('/*DATA*/', json.dumps(data, separators=(',', ':')))
with open(os.path.join(HERE, 'out', 'safe_corridor.html'), 'w', encoding='utf-8') as f:
    f.write(page)
print(f"out/safe_corridor.html: {len(page) / 1e6:.1f} MB")
