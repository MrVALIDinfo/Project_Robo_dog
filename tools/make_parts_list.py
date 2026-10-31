"""Final purchase list (PDF). Run: python tools/make_parts_list.py -> docs/parts_list_final.pdf"""
import os
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

pdfmetrics.registerFont(TTFont('DV', '/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf'))
pdfmetrics.registerFont(TTFont('DVB', '/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf'))

SECTIONS = [
    ('1. Computing & sensors', [
        ('Raspberry Pi 3B', '1', ''),
        ('microSD card, 16-32 GB', '1', ''),
        ('Raspberry Pi Camera v2', '1', ''),
        ('Camera ribbon cable, 15-pin', '1', '30 cm (route is ~22 cm)'),
        ('MPU-6050 (IMU)', '1', ''),
        ('LCD 16x2 with I2C backpack', '1', 'Address 0x27'),
        ('I2C level shifter, BSS138, 4-ch', '1', 'LCD backpack pulls SDA/SCL toward 5 V; Pi GPIO is 3.3 V only'),
        ('Heatsink for Pi CPU', '1', 'Skip if already installed'),
        ('Dupont jumper wires F-F', '1 set', ''),
    ]),
    ('2. Drive', [
        ('PCA9685 servo driver, 16-ch', '1', ''),
        ('Servo PDI-HV5523MG', '12', ''),
        ('Servo extension 3-pin, 20 cm', '4', 'Knee servos'),
        ('Servo extension 3-pin, 10 cm', '2', 'Right hip servos. Other 6 servos: own cable is enough'),
    ]),
    ('3. Power', [
        ('LiPo 2S 4000-5000 mAh, shorty hardcase', '1', 'Must fit bay: max 127 x 48 x 25.5 mm (check height and width). '
                                         'Standard 138 mm packs do not fit. E.g. Gens Ace 2S 5000 mAh shorty, 95 x 46.8 x 25 mm'),
        ('UBEC 5 V / 5 A, 2S input', '1', 'E.g. Hobbywing UBEC 5A (2-8S, 5 A cont.). Set output to 5.0 V before connecting the Pi'),
        ('Rocker switch Switch Components RF1-1A-DC-2-B', '1', '16 A at 12 V DC, panel cut-out 19.2 x 13 mm. 20 A rockers do not fit'),
        ('Blade fuse 15 A (ATC) + inline holder, 14 AWG', '1', 'Not 20 A: switch is rated 16 A'),
        ('XT60 connector pair', '1', 'Solder to the battery if it comes with bullets or T-plug'),
        ('XT60 Y-splitter', '1', 'Servos + UBEC'),
        ('Capacitor 1000-2200 uF, 16 V or higher', '1', 'On servo power line'),
        ('Silicone wire 14-16 AWG, red + black', '1 m each', ''),
        ('2S LiPo balance charger', '1', ''),
        ('LiPo low-voltage buzzer', '1', ''),
    ]),
    ('4. Voice', [
        ('USB microphone', '1', ''),
        ('USB speaker', '1', ''),
    ]),
    ('5. Optional: battery voltage', [
        ('ADS1115 ADC', '1', ''),
        ('Resistors 20 kOhm + 10 kOhm', '1 + 1', 'Divider: 8.4 V -> 2.8 V'),
    ]),
    ('6. Fasteners & bearings (lengths measured on our _cls parts)', [
        ('Bolt M5x15', '8', 'Covers to shoulder bulkheads'),
        ('Nut M5', '8', 'Hex pockets in bulkheads (not in KDY BOM)'),
        ('Bolt M4x20', '40', 'Servo mounting: shoulder, hip, knee'),
        ('Bolt M4x25', '8', 'Knee servo, 20 mm holes in the _cls shin'),
        ('Nut M4', '48', ''),
        ('Bolt M3x25', '24', 'Bulkheads (12); thigh + cover (12, check at assembly)'),
        ('Bolt M3x20', '12', 'Side plates to bulkheads'),
        ('Nut M3', '36', 'In nut slots'),
        ('Bolt M3x10 + nut M3', '8 + 8', 'mike4192 shoulder reinforcement'),
        ('Bearing F625zz', '12', 'Frame 8 + shoulder reinforcement 4'),
        ('Screw M2x6 + nut M2', '4 + 4', 'Camera mount'),
    ]),
    ('7. Consumables', [
        ('Self-tapping screw 2 x 6-8 mm', '8', 'Pi (4) and PCA9685 (4) to their printed platforms (holes 1.5 mm)'),
        ('Double-sided foam tape', '1 roll', 'Pi and PCA9685 platforms to plate; MPU-6050 to plate'),
        ('Hot glue sticks + glue gun', '1', 'Servo horns into printed parts (KDY0523 instructions)'),
        ('Zip ties, small', '1 pack', 'Servo cables'),
    ]),
]

out = os.path.join(os.path.dirname(__file__), '..', 'docs', 'parts_list_final.pdf')
doc = SimpleDocTemplate(out, pagesize=A4, leftMargin=14 * mm, rightMargin=14 * mm,
                        topMargin=6 * mm, bottomMargin=5 * mm,
                        title='Spot Micro - final parts list', author='Project_Robo_dog')
H = ParagraphStyle('h', fontName='DVB', fontSize=15, leading=19)
S = ParagraphStyle('s', fontName='DV', fontSize=8.5, leading=11, textColor=colors.HexColor('#555555'))
SEC = ParagraphStyle('sec', fontName='DVB', fontSize=10, leading=12, spaceBefore=3, spaceAfter=1)
C = ParagraphStyle('c', fontName='DV', fontSize=7.6, leading=9)

story = [Paragraph('Spot Micro - final parts list', H),
         Paragraph('31 Oct 2026. Replaces the 23 Oct list. Checked against the KDY0523 _cls frame in the 3D model. '
                   '3D-printed parts: see print/README.md.', S), Spacer(1, 4)]
W = [7 * mm, 80 * mm, 16 * mm, 79 * mm]
for title, rows in SECTIONS:
    story.append(Paragraph(title, SEC))
    data = [['', 'Item', 'Qty', 'Notes']] + [['☐', Paragraph(a, C), b, Paragraph(c, C)] for a, b, c in rows]
    t = Table(data, colWidths=W, repeatRows=1)
    t.setStyle(TableStyle([
        ('FONT', (0, 0), (-1, 0), 'DVB', 8), ('FONT', (0, 1), (-1, -1), 'DV', 8),
        ('FONTSIZE', (0, 1), (0, -1), 9),
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#e8e8e8')),
        ('LINEBELOW', (0, 0), (-1, -1), 0.3, colors.HexColor('#bbbbbb')),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'), ('ALIGN', (2, 0), (2, -1), 'CENTER'),
        ('TOPPADDING', (0, 0), (-1, -1), 0.6), ('BOTTOMPADDING', (0, 0), (-1, -1), 0.6),
    ]))
    story.append(t)
WIRING = [
    ('Power (14-16 AWG)', [
        ('LiPo 2S', 'XT60', 'Battery plug'),
        ('XT60 (+)', 'Fuse 15 A -> switch -> Y-splitter (+)', 'Switch in the rear cover'),
        ('XT60 (-)', 'Y-splitter (-)', ''),
        ('Y-splitter branch 1', 'PCA9685 V+ / GND terminal', 'Servo power 6.4-8.4 V; capacitor across this terminal'),
        ('Y-splitter branch 2', 'UBEC input', ''),
        ('LiPo balance lead', 'Low-voltage buzzer', ''),
    ]),
    ('5 V (UBEC output)', [
        ('UBEC 5 V / GND', 'Raspberry Pi pin 2, 4 (5 V) / pin 6 (GND)', 'Output jumper at 5.0 V'),
        ('UBEC 5 V / GND', 'LCD backpack VCC / GND, level shifter HV', ''),
    ]),
    ('I2C bus, 3.3 V (Dupont)', [
        ('Pi pin 1 (3.3 V)', 'PCA9685 VCC, MPU-6050 VCC, ADS1115 VDD, level shifter LV', 'PCA9685 logic at 3.3 V, not 5 V'),
        ('Pi pin 3 SDA / pin 5 SCL', 'PCA9685 (0x40), MPU-6050 (0x68), ADS1115 (0x48)', 'No address conflicts'),
        ('Pi pin 3 / 5 via level shifter', 'LCD SDA / SCL (0x27)', ''),
        ('Pi pin 9 (GND)', 'PCA9685 GND, all module GND', 'Common ground'),
    ]),
    ('Signals', [
        ('PCA9685 channels', '12 servos (3-pin plugs on the header)', 'Knee servos via 20 cm, right hips via 10 cm extensions'),
        ('Battery (+) after switch', 'Divider 20k/10k -> ADS1115 A0', 'Optional, max 2.8 V at A0'),
        ('Camera v2', 'Pi CSI connector, 30 cm ribbon', ''),
        ('USB mic, USB speaker', 'Pi USB ports', ''),
    ]),
]
story += [PageBreak(), Paragraph('Wiring', H),
          Paragraph('Same as the mike4192 build (2S direct to the PCA9685 servo terminal, UBEC 5 V for the Pi). '
                    'Differences: PCA9685 logic at 3.3 V and a level shifter for the 5 V LCD, so the Pi I2C pins never see 5 V. '
                    'The mike4192 software drives only the PCA9685 and the LCD; MPU-6050, camera, ADS1115 and voice need our own code.', S),
          Spacer(1, 4)]
WW = [52 * mm, 72 * mm, 58 * mm]
for title, rows in WIRING:
    story.append(Paragraph(title, SEC))
    data = [['From', 'To', 'Notes']] + [[Paragraph(a, C), Paragraph(b, C), Paragraph(c, C)] for a, b, c in rows]
    t = Table(data, colWidths=WW)
    t.setStyle(TableStyle([
        ('FONT', (0, 0), (-1, 0), 'DVB', 8), ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#e8e8e8')),
        ('LINEBELOW', (0, 0), (-1, -1), 0.3, colors.HexColor('#bbbbbb')), ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('TOPPADDING', (0, 0), (-1, -1), 2), ('BOTTOMPADDING', (0, 0), (-1, -1), 2),
    ]))
    story.append(t)
doc.build(story)
print(os.path.abspath(out))
