"""Build the APAC 2026 edition of the engineering journal from the repository's own docs.

Part I is rendered from docs/apac_2026_vehicle.md, docs/apac_2026_software.md and
src/apac-2026/README.md (plus a cover and a one-page snapshot), printed to PDF by
headless Microsoft Edge; Part II is the Nationals journal (rev 3) unchanged.
Run from anywhere:  python docs/journal/build_apac_journal.py
Needs: Python 3 with markdown and pymupdf; Microsoft Edge (Windows) or set EDGE; the state diagrams are drawn by mermaid.ink.
"""
import os, re, sys, subprocess, tempfile, html
from pathlib import Path
import markdown
import fitz  # pymupdf

ROOT = Path(__file__).resolve().parents[2]
EDGE = os.environ.get('EDGE', r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe')
GH = 'https://github.com/teddriveomo/wro-2026-future-engineers/blob/main/'
OUT = ROOT / 'docs' / 'engineering_journal_apac_2026.pdf'
NATIONALS = ROOT / 'docs' / 'engineering_journal_final.pdf'
WORK = Path(os.environ.get('JOURNAL_WORK', tempfile.gettempdir())) / 'ted_journal_apac'
WORK.mkdir(parents=True, exist_ok=True)

def rewrite_links(md, doc_dir):
    def fix(m):
        bang, text, target = m.group(1), m.group(2), m.group(3)
        if re.match(r'^(https?:|mailto:|#)', target):
            return m.group(0)
        path, _, frag = target.partition('#')
        rel = os.path.normpath(os.path.join(doc_dir, path.replace('%20', ' '))).replace('\\', '/')
        if bang:
            return '%s[%s](%s)' % (bang, text, (ROOT / rel).as_uri())
        url = GH + rel.replace(' ', '%20') + ('#' + frag if frag else '')
        return '[%s](%s)' % (text, url)
    return re.sub(r'(!?)\[([^\]]*)\]\(([^)\s]+)\)', fix, md)

def sections(md):
    """Split a doc into (title, body) at '## ' headings; drop the H1 line."""
    md = re.sub(r'\A# [^\n]*\n', '', md)
    parts = re.split(r'(?m)^## ', md)
    head, out = parts[0], []
    for p in parts[1:]:
        title, _, body = p.partition('\n')
        out.append((title.strip(), body))
    return head, out

def diagram(src_html):
    """Mermaid source -> PNG from mermaid.ink (cached); a link to the repository if that fails."""
    import base64, hashlib, urllib.request
    src_text = html.unescape(src_html).strip()
    png = WORK / ('diagram_%s.png' % hashlib.md5(src_text.encode('utf-8')).hexdigest()[:12])
    if not png.exists():
        url = 'https://mermaid.ink/img/' + base64.urlsafe_b64encode(src_text.encode('utf-8')).decode('ascii') + '?type=png&bgColor=!white&width=1600'
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'ted-journal-build'})
            data = urllib.request.urlopen(req, timeout=60).read()
            if data[:4] != b'\x89PNG': raise ValueError('not a PNG')
            png.write_bytes(data)
        except Exception as e:
            print('diagram fetch failed:', e)
            return '<p class="note">State diagram: see <a href="%sdocs/apac_2026_software.md">docs/apac_2026_software.md</a>, where GitHub renders it.</p>' % GH
    return '<img class="diagram" src="%s">' % png.as_uri()

def render(md):
    h = markdown.markdown(md, extensions=['extra', 'sane_lists'])
    h = re.sub(r'<pre><code class="language-mermaid">(.*?)</code></pre>', lambda m: diagram(m.group(1)), h, flags=re.S)
    return h

def sub(title, body):  # a doc section as an H3 block inside a chapter
    return '<h3>%s</h3>\n%s' % (html.escape(title), render(body))

veh = rewrite_links((ROOT / 'docs/apac_2026_vehicle.md').read_text(encoding='utf-8'), 'docs')
sw = rewrite_links((ROOT / 'docs/apac_2026_software.md').read_text(encoding='utf-8'), 'docs')
src = rewrite_links((ROOT / 'src/apac-2026/README.md').read_text(encoding='utf-8'), 'src/apac-2026')
vhead, vsec = sections(veh); shead, ssec = sections(sw); rhead, rsec = sections(src)
print('vehicle sections:', [t for t, _ in vsec]); print('software sections:', [t for t, _ in ssec]); print('src sections:', [t for t, _ in rsec])

def num(t):
    m = re.match(r'(\d+)\.', t); return int(m.group(1)) if m else None
mob = [s for s in vsec if num(s[0]) in (1, 2, 3, 4)]
pwr = [s for s in vsec if num(s[0]) in (5, 6, 8)]
dec = [s for s in vsec if num(s[0]) == 7]
skipped = [t for t, _ in vsec if num(t) not in (1, 2, 3, 4, 5, 6, 7, 8)]
print('vehicle sections left out of the journal (open-items list stays in the repo):', skipped)

views = [('front', 'Front'), ('rear', 'Rear'), ('left', 'Left'), ('right', 'Right'), ('top', 'Top'), ('bottom', 'Bottom (laid on its side)')]
grid = '<div class="grid">' + ''.join(
    '<figure><img src="%s"><figcaption>%s</figcaption></figure>' % ((ROOT / 'v-photos/apac-2026-09-22' / ('apac-%s.jpg' % k)).as_uri(), c)
    for k, c in views) + '</div>'

CSS = """
@page { size: A4; margin: 15mm 14mm 17mm 14mm; }
body { font-family: 'Segoe UI', Arial, sans-serif; font-size: 9.6pt; line-height: 1.42; color: #1d1d1f; }
.band { font-size: 7.5pt; letter-spacing: .14em; color: #6b6b6b; border-bottom: 1px solid #d9d9d9; padding-bottom: 3px; margin-bottom: 10px; }
h1 { font-size: 26pt; margin: 6px 0 2px; letter-spacing: -.01em; }
h2 { font-size: 16pt; margin: 0 0 4px; } h2 .n { color: #c0392b; margin-right: 8px; }
h2 + .lede { color: #555; margin-top: 0; font-style: italic; }
h3 { font-size: 11.5pt; margin: 14px 0 4px; border-left: 3px solid #c0392b; padding-left: 7px; }
h4 { font-size: 10pt; margin: 10px 0 3px; }
h3, h4 { break-after: avoid-page; page-break-after: avoid; }
.views { break-inside: avoid; page-break-inside: avoid; }
section { page-break-before: always; }
table { width: 100%; border-collapse: collapse; margin: 6px 0 10px; font-size: 8.3pt; page-break-inside: auto; }
th, td { border: 1px solid #cfcfcf; padding: 3px 5px; vertical-align: top; text-align: left; }
th { background: #f2f2f2; } tr { page-break-inside: avoid; }
code { font-family: Consolas, monospace; font-size: 8.2pt; background: #f4f4f4; padding: 0 2px; word-break: break-word; }
pre { background: #f7f7f7; padding: 6px; font-size: 8pt; white-space: pre-wrap; }
img.diagram { display: block; max-width: 100%; max-height: 225mm; margin: 6px auto; break-inside: avoid; page-break-inside: avoid; }
img { max-width: 100%; }
p img { display: block; max-height: 88mm; margin: 6px auto; border: 1px solid #ddd; }
a { color: #1a4f8b; text-decoration: none; }
.cover { text-align: left; } .cover .photo { width: 100%; max-height: 118mm; object-fit: cover; border-radius: 4px; margin: 10px 0; }
.meta td { border: none; padding: 2px 8px 2px 0; font-size: 9.6pt; } .meta td:first-child { color: #6b6b6b; width: 32mm; }
.grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px; margin: 8px 0; }
.grid figure { margin: 0; page-break-inside: avoid; } .grid img { width: 100%; height: 44mm; object-fit: cover; border: 1px solid #ddd; }
figcaption { font-size: 7.8pt; color: #555; text-align: center; }
.kv td:first-child { width: 34mm; font-weight: 600; }
.note { font-size: 8.4pt; color: #555; }
"""
BAND = '<div class="band">TEAM TED DRIVE &nbsp;|&nbsp; WRO FUTURE ENGINEERS 2026 &nbsp;|&nbsp; ENGINEERING JOURNAL, APAC EDITION</div>'
cover = """
<div class="cover">%s
<div style="font-size:9pt;color:#c0392b;letter-spacing:.12em;margin-top:16px">WRO FUTURE ENGINEERS 2026 &middot; APAC</div>
<h1>Engineering Journal</h1>
<div style="font-size:13pt;color:#444">APAC 2026 edition (rev 4) &middot; Team TED Drive</div>
<img class="photo" src="%s">
<table class="meta">
<tr><td>Team</td><td>Ethan Fernandes (software) &middot; Tejas Sirikonda (mechanical) &middot; Diaan (electronics)</td></tr>
<tr><td>Coach</td><td>Amey Chavan (OMOTEC)</td></tr>
<tr><td>Competition</td><td>WRO Future Engineers 2026, APAC, Hyderabad, September 2026</td></tr>
<tr><td>Repository</td><td><a href="https://github.com/teddriveomo/wro-2026-future-engineers">github.com/teddriveomo/wro-2026-future-engineers</a> (release 0.5.0 and later)</td></tr>
<tr><td>Prepared</td><td>23 September 2026, against WRO 2026 General Rules, Appendix C</td></tr>
</table>
<p class="note">Part I describes the vehicle rebuilt for APAC and is generated from the repository's own documents, so the journal and the repository say the same thing. Part II is the India Nationals journal (rev 3, 24 August 2026), kept unchanged as the record of the first vehicle.</p>
</div>""" % (BAND, (ROOT / 't-photos/team_photo_official.jpg').as_uri())

snapshot = """<section>%s<h2><span class="n">01</span>Snapshot</h2><p class="lede">The APAC vehicle in one page; every figure is from Part I and the repository.</p>
<table class="kv">
<tr><td>Vehicle</td><td>Rebuilt in September 2026: a new 84-step LEGO Technic frame (332 g bare), a steering column that turns the whole front axle 60&deg; each way, the camera on a 24.8 cm tower pointing 12&deg; down, six WS2812 LEDs on the front frame. The horizontal side rollers of the rebuild were removed on 22 September.</td></tr>
<tr><td>Mass</td><td>862 g race-ready with the Raspberry Pi 5 (measured 16 September, before the rollers came off).</td></tr>
<tr><td>Drive</td><td>MEX 6 V motor through a TB6612FNG driver to the rear wheels: 422 rpm free-running at the wheel and 0.89 m/s average over 3 m from a standing start (measured 21 September).</td></tr>
<tr><td>Power</td><td>3S 2200 mAh pack: 1.34 A idle, 1.93-2.02 A driving, 2.77 A worst case (each branch measured, 21 September).</td></tr>
<tr><td>Compute</td><td>Raspberry Pi 5 (camera, detection, decisions) and ESP32 (drive motor, servo, three TF-Luna, LEDs), linked by USB serial.</td></tr>
<tr><td>Software</td><td><code>main.py</code> dated 18 September: Lab-colour pillar detection with tape-line corner context, a state machine that recentres between the walls after every pillar and corner, two pillars per straight, and a CSV log of every run. Parking at the end: <code>parallel_parking.py</code>, the team's choice for APAC.</td></tr>
<tr><td>Evidence</td><td>Bench measurements of drive, power and camera geometry (&sect;02, &sect;03); detection shown on the mat (&sect;04); code stored byte-for-byte with MD5 checksums (&sect;06).</td></tr>
</table>
<h3>Contents</h3>
<table><tr><th>&sect;</th><th>Part I: APAC 2026 vehicle</th><th>Appendix C</th></tr>
<tr><td>01</td><td>Snapshot</td><td></td></tr>
<tr><td>02</td><td>Mobility: frame, drive, steering, camera placement, six views</td><td>C1</td></tr>
<tr><td>03</td><td>Power and sensing: LEDs, measured power budget, failure points, ESP32 pins</td><td>C2</td></tr>
<tr><td>04</td><td>Software and obstacle strategy: control loop, states, recovery, detection, parking, validation</td><td>C3</td></tr>
<tr><td>05</td><td>Engineering decisions: every change with its reason, evidence and cost</td><td>C4</td></tr>
<tr><td>06</td><td>Reproducibility: files, serial link, settings, provenance and checksums</td><td>C5</td></tr>
<tr><td>II</td><td>India Nationals journal, rev 3 (the first vehicle)</td><td>C1-C5</td></tr></table>
</section>""" % BAND

def chapter(n, title, lede, body):
    return '<section>%s<h2><span class="n">%s</span>%s</h2><p class="lede">%s</p>%s</section>' % (BAND, n, html.escape(title), lede, body)

ch2 = chapter('02', 'Mobility', 'How the car is built and moves (Appendix C, criterion 1).', render(vhead) + ''.join(sub(t, b) for t, b in mob) + '<div class="views"><h3>The six views</h3>' + grid + '</div>')
ch3 = chapter('03', 'Power and sensing', 'What powers the car, what it senses with, and what can fail (criterion 2).', ''.join(sub(t, b) for t, b in pwr))
ch4 = chapter('04', 'Software and obstacle strategy', 'How main.py drives the Obstacle Challenge and how the car parks (criterion 3).', render(shead) + ''.join(sub(t, b) for t, b in ssec))
ch5 = chapter('05', 'Engineering decisions', 'Each change as a decision: what it replaced, why, the evidence, and the cost (criterion 4).', ''.join(sub(t, b) for t, b in dec))
ch6 = chapter('06', 'Reproducibility', 'How to rebuild and run the software exactly as committed (criterion 5).', render(rhead) + ''.join(sub(t, b) for t, b in rsec))

doc = """<!DOCTYPE html><html><head><meta charset="utf-8"><title>TED Drive - WRO FE 2026 Engineering Journal (APAC edition)</title>
<style>%s</style>
</head><body>%s%s%s%s%s%s%s</body></html>""" % (CSS, cover, snapshot, ch2, ch3, ch4, ch5, ch6)
page = WORK / 'journal_apac.html'; page.write_text(doc, encoding='utf-8')
part1 = WORK / 'journal_apac_part1.pdf'
if part1.exists(): part1.unlink()
cmd = [EDGE, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--run-all-compositor-stages-before-draw',
       '--virtual-time-budget=30000', '--user-data-dir=' + str(WORK / 'edge-profile'), '--print-to-pdf=' + str(part1), page.as_uri()]
subprocess.run(cmd, check=False, timeout=240, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
assert part1.exists() and part1.stat().st_size > 50000, 'Edge did not produce the PDF'

p1 = fitz.open(str(part1)); n1 = p1.page_count
text = [pg.get_text() for pg in p1]
leftover = [i + 1 for i, pg in enumerate(p1) if i > 0 and len(re.sub(r'\s+', '', text[i])) < 160 and not pg.get_images()]
print('part I pages', n1, '| near-empty pages without images:', leftover or 'none')
for i, pg in enumerate(p1):
    if i == 0: continue
    r = pg.rect
    pg.insert_text((r.width / 2 - 150, r.height - 22), 'TED Drive \u00b7 WRO FE 2026 Engineering Journal, APAC edition \u00b7 Part I \u00b7 %d / %d' % (i + 1, n1), fontsize=7, color=(0.45, 0.45, 0.45))
nat = fitz.open(str(NATIONALS))
div = fitz.open(); w, h = p1[0].rect.width, p1[0].rect.height; d = div.new_page(width=w, height=h)
d.insert_text((56, 300), 'Part II', fontsize=30, color=(0.75, 0.22, 0.17))
d.insert_text((56, 336), 'India Nationals journal, rev 3 (24 August 2026)', fontsize=14)
d.insert_text((56, 360), 'The first vehicle, kept unchanged. Part I describes the car that replaced it.', fontsize=10, color=(0.3, 0.3, 0.3))
out = fitz.open(); out.insert_pdf(p1); out.insert_pdf(div); out.insert_pdf(nat)
def first(pat):
    pat = r'\s+'.join(pat.split(' '))
    for i, t in enumerate(text):
        if re.search(pat, t): return i + 1
    raise SystemExit('bookmark target not found: ' + pat)
toc = [[1, 'Cover', 1], [1, 'Part I - APAC 2026 vehicle', 2],
       [2, '01 Snapshot', first('The APAC vehicle in one page')], [2, '02 Mobility', first('How the car is built and moves')],
       [2, '03 Power and sensing', first('What powers the car')], [2, '04 Software and obstacle strategy', first('How main.py drives the Obstacle')],
       [2, '05 Engineering decisions', first('Each change as a decision')], [2, '06 Reproducibility', first('How to rebuild and run')],
       [1, 'Part II - India Nationals journal (rev 3)', n1 + 1]]
out.set_toc(toc)
out.set_metadata({'title': 'TED Drive - WRO FE 2026 Engineering Journal (APAC 2026 edition, rev 4)', 'author': 'Team TED Drive',
                  'subject': 'WRO Future Engineers 2026, APAC', 'creator': 'docs/journal/build_apac_journal.py'})
out.save(str(OUT), garbage=3, deflate=True)
print('wrote', OUT, 'pages', out.page_count, 'bytes', OUT.stat().st_size, '| toc', [(t[1], t[2]) for t in toc])
qa = sorted(set([first('The six views'), first('2. The run') + 1]))
for pno in qa:
    pix = out[pno - 1].get_pixmap(matrix=fitz.Matrix(0.9, 0.9)); pix.save(str(WORK / ('qa_p%02d.png' % pno)))
print('qa pages', qa, 'in', WORK)
