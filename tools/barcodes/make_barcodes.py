"""Make a printable PDF of the Code 39 track barcodes ("*X*", 141 mm).

Sizes match subsystem 3's decoder (app_program/subsystem3/code39_wide.h):
narrow 3.0 mm, wide 9.0 mm (3:1), so "*X*" = 20 narrow + 9 wide = 141 mm.
Patterns are the same table as app_program/subsystem3/code39.c
(9 elements, bar first, 1 = wide).

GP28, the barcode sensor, sits about 45 mm to the CAR'S LEFT of the line
(line_sensor.c), so each barcode is drawn with a guide showing where the
track line's centre goes.

No libraries needed: writes the PDF by hand.
Run:  python tools/barcodes/make_barcodes.py
Out:  tools/barcodes/IS14_test_barcodes.pdf
"""

import os

MM = 72.0 / 25.4                      # points per mm
PAGE_W, PAGE_H = 210.0, 297.0         # A4, mm
NARROW, WIDE = 3.0, 9.0               # mm
BAR_H = 40.0                          # bar length across the travel direction, mm
QUIET = 20.0                          # white before and after, mm
GP28_OFFSET = 45.0                    # line centre to GP28, mm

PATTERNS = {
    '*': 0b010010100,
    'A': 0b100001001, 'B': 0b001001001, 'C': 0b101001000, 'D': 0b000011001,
    'E': 0b100011000, 'F': 0b001011000, 'G': 0b000001101, 'H': 0b100001100,
    'I': 0b001001100, 'J': 0b000011100, 'K': 0b100000011, 'L': 0b001000011,
    'M': 0b101000010, 'N': 0b000010011, 'O': 0b100010010, 'P': 0b001010010,
    'Q': 0b000000111, 'R': 0b100000110, 'S': 0b001000110, 'T': 0b000010110,
    'U': 0b110000001, 'V': 0b011000001, 'W': 0b111000000, 'X': 0b010010001,
    'Y': 0b110010000, 'Z': 0b011010000,
}

MAIN = [('A', 'LEFT'), ('B', 'RIGHT'), ('C', 'FORWARD (straight on)'), ('D', 'U-TURN')]


def elements(text):
    """(width_mm, is_bar) for the whole symbol, with the narrow gap between characters."""
    out = []
    for i, ch in enumerate(text):
        pat = PATTERNS[ch]
        for k in range(9):
            wide = (pat >> (8 - k)) & 1
            out.append((WIDE if wide else NARROW, k % 2 == 0))
        if i != len(text) - 1:
            out.append((NARROW, False))
    return out


class Page:
    def __init__(self):
        self.ops = []

    def _y(self, y_mm):                # y measured from the TOP of the page
        return (PAGE_H - y_mm) * MM

    def rect(self, x, y_top, w, h, gray=0.0):
        self.ops.append('%.3f g %.2f %.2f %.2f %.2f re f' %
                        (gray, x * MM, self._y(y_top + h), w * MM, h * MM))

    def line(self, x1, y1, x2, y2, width=0.3, dash=None, gray=0.0):
        d = '[%s] 0 d' % ' '.join('%.2f' % (v * MM) for v in dash) if dash else '[] 0 d'
        self.ops.append('%.3f G %s %.2f w %.2f %.2f m %.2f %.2f l S' %
                        (gray, d, width * MM, x1 * MM, self._y(y1), x2 * MM, self._y(y2)))

    def text(self, x, y, s, size=10, bold=False):
        s = s.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')
        self.ops.append('0 g BT /%s %.1f Tf %.2f %.2f Td (%s) Tj ET' %
                        ('F2' if bold else 'F1', size, x * MM, self._y(y), s))

    def stream(self):
        return '\n'.join(self.ops).encode('latin-1')


def barcode_block(p, y, letter, meaning):
    """One barcode with its labels; y = top of the block, mm. Block is about 88 mm tall."""
    total = sum(w for w, _ in elements('*' + letter + '*'))
    x0 = (PAGE_W - total) / 2.0
    title = 'Barcode *%s*   ->   %s' % (letter, meaning)
    p.text(x0 - QUIET, y + 5, title, size=14, bold=True)
    box = {'LEFT': 'Left', 'RIGHT': 'Right', 'FORWARD': 'Forward', 'U-TURN': 'U-turn'}.get(meaning.split(' ')[0])
    how = ('Dashboard map: %s in the %s box.' % (letter, box)) if box else \
          'Map it on the dashboard only if the demo uses this letter.'
    p.text(x0 - QUIET, y + 10.5, '%s   %.0f mm long, narrow %.0f mm, wide %.0f mm.'
           % (how, total, NARROW, WIDE), size=8.5)
    top = y + 15
    # light frame = quiet zone, cut along it
    p.line(x0 - QUIET, top - 2, x0 + total + QUIET, top - 2, 0.2, dash=(1.5, 1.5), gray=0.6)
    p.line(x0 - QUIET, top + BAR_H + 2, x0 + total + QUIET, top + BAR_H + 2, 0.2, dash=(1.5, 1.5), gray=0.6)
    x = x0
    for w, bar in elements('*' + letter + '*'):
        if bar:
            p.rect(x, top, w, BAR_H)
        x += w
    p.text(x0 + total / 2 - 2, top + BAR_H + 7, letter, size=12, bold=True)
    # where the track line goes: GP28 (bar centre) is 45 mm to the car's left of the line
    ly = top + BAR_H / 2 + GP28_OFFSET
    p.line(x0 - QUIET, ly, x0 + total + QUIET, ly, 0.5, dash=(4, 2), gray=0.45)
    p.text(x0 - QUIET, ly + 4.5,
           'Centre of the black track line goes along this dashed line (45 mm from the bar centre).', size=8)
    p.text(x0 - QUIET, ly + 8.5,
           'Car drives left -> right on this page, so the bars are on the CAR\'S LEFT (GP28 side).', size=8)


def header_page(p):
    p.text(15, 18, 'IS14 test barcodes (Code 39, subsystem 3)', size=16, bold=True)
    lines = [
        'Print on A4 at ACTUAL SIZE / 100 % (turn OFF "Fit to page"). Check: the ruler below must be 100 mm.',
        'Matte paper, black ink. Cut along the grey dotted lines and tape flat (no tape over the bars).',
        'Place it beside the track line, bars at right angles to the driving direction, at least 10 cm',
        'before the junction, on the car\'s LEFT. The car can read it in either direction.',
        'Before the run, while the car is READY, send the map on the dashboard: Forward C, U-turn D, Left A, Right B.',
        'Without a map the car ignores the barcode. Pages 3 onwards: spare letters E..Z.',
    ]
    for i, s in enumerate(lines):
        p.text(15, 27 + i * 5, s, size=9)
    # 100 mm ruler
    ry = 62
    p.rect(15, ry, 100, 1.2)
    for k in range(11):
        h = 5 if k % 5 == 0 else 3
        p.rect(15 + k * 10 - 0.2, ry - h, 0.4, h)
    p.text(118, ry + 1, '100 mm', size=9)


def build():
    pages = []
    p = Page()
    header_page(p)
    barcode_block(p, 72, *MAIN[0])
    barcode_block(p, 172, *MAIN[1])
    pages.append(p)
    p = Page()
    barcode_block(p, 15, *MAIN[2])
    barcode_block(p, 110, *MAIN[3])
    pages.append(p)
    spare = [c for c in 'EFGHIJKLMNOPQRSTUVWXYZ']
    for i in range(0, len(spare), 3):
        p = Page()
        for j, c in enumerate(spare[i:i + 3]):
            barcode_block(p, 10 + j * 95, c, 'spare letter')
        pages.append(p)
    return pages


def write_pdf(pages, path):
    objs = []                                  # index 0 -> object 1

    def add(b):
        objs.append(b)
        return len(objs)

    cat = add(None)
    pages_id = add(None)
    f1 = add(b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>')
    f2 = add(b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>')
    kids = []
    for pg in pages:
        data = pg.stream()
        c = add(b'<< /Length %d >>\nstream\n' % len(data) + data + b'\nendstream')
        kids.append(add(('<< /Type /Page /Parent %d 0 R /MediaBox [0 0 %.2f %.2f] '
                         '/Resources << /Font << /F1 %d 0 R /F2 %d 0 R >> >> /Contents %d 0 R >>'
                         % (pages_id, PAGE_W * MM, PAGE_H * MM, f1, f2, c)).encode()))
    objs[cat - 1] = ('<< /Type /Catalog /Pages %d 0 R >>' % pages_id).encode()
    objs[pages_id - 1] = ('<< /Type /Pages /Kids [%s] /Count %d >>'
                          % (' '.join('%d 0 R' % k for k in kids), len(kids))).encode()
    out = bytearray(b'%PDF-1.4\n')
    offs = []
    for i, o in enumerate(objs, 1):
        offs.append(len(out))
        out += b'%d 0 obj\n' % i + o + b'\nendobj\n'
    xref = len(out)
    out += b'xref\n0 %d\n0000000000 65535 f \n' % (len(objs) + 1)
    for o in offs:
        out += b'%010d 00000 n \n' % o
    out += b'trailer\n<< /Size %d /Root %d 0 R >>\nstartxref\n%d\n%%%%EOF\n' % (len(objs) + 1, cat, xref)
    with open(path, 'wb') as fh:
        fh.write(out)


if __name__ == '__main__':
    assert abs(sum(w for w, _ in elements('*A*')) - 141.0) < 1e-6
    here = os.path.dirname(os.path.abspath(__file__))
    write_pdf(build(), os.path.join(here, 'IS14_test_barcodes.pdf'))
    print('wrote', os.path.join(here, 'IS14_test_barcodes.pdf'))
