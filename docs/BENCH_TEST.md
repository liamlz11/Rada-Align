# בדיקות ספסל ל-ESP32 (משימה 6)

**לא לצרוב ולא להזיז מנועים בלי אישור מפורש של ליאם.** המסמך הזה מתאר מה לעשות אחרי האישור.

כל הפינים, הגבולות והמהירויות ב-`firmware/include/config.h` הם הנחות. לפני צריבה חובה לעדכן אותם לחיווט האמיתי.

## לפני הצריבה

1. לעדכן ב-`firmware/include/config.h`: מפת פינים, `HAS_LIMIT_SWITCHES`, `ENABLE_ACTIVE_LOW`, גבולות רכים, מהירויות.
2. לעדכן ב-`firmware/platformio.ini` את `board` ללוח האמיתי.
3. לעדכן ב-`config.yaml` תחת `hw:` את `serial_port` (מנהל ההתקנים > Ports).
4. לבנות (בלי צריבה), מתוך תיקיית `firmware`:

```bash
pio run
```

## פקודת הצריבה (רק אחרי אישור)

מתוך `firmware`, עם הלוח מחובר ב-USB ו**המנועים מנותקים**:

```bash
pio run -t upload
```

מסוף סריאלי (115200):

```bash
pio device monitor
```

## שלב א': מנועים מנותקים, דרייברים בלי מתח מנוע

| # | פעולה | תוצאה צפויה |
| --- | --- | --- |
| 1 | לפתוח מסוף אחרי אתחול | `OK RADA-ALIGN READY` |
| 2 | `PING` | `OK` |
| 3 | `STATUS` | `OK pos=0,0,0,0 moving=0000 limits=0000 estop=0 wd=0 homed=0000` |
| 4 | `LED ON`, `LED OFF`, `LED BLINK 5` | `OK` בכל אחת; ה-LED נדלק, נכבה, מהבהב 5 הרץ |
| 5 | `MOVE X 100` מיד אחרי `PING` | `OK 100` |
| 6 | לחכות 2 שניות בלי `PING`, ואז `MOVE X 100` | `ERR WATCHDOG` |
| 7 | `STATUS` | `wd=1` |
| 8 | `PING` ואז `MOVE X 999999` | `ERR SOFT_LIMIT` |
| 9 | `MOVE Q 10` | `ERR BAD_AXIS` |
| 10 | `MOVE X abc` | `ERR BAD_NUMBER` |
| 11 | `FOO` | `ERR UNKNOWN_COMMAND` |
| 12 | ללחוץ על כפתור החירום, ואז `STATUS` | `estop=1` |
| 13 | `PING` כשהכפתור לחוץ | `ERR ESTOP` |
| 14 | לשחרר את הכפתור, `PING`, `STATUS` | `OK`, ואז `estop=0` |
| 15 | לנתק את חוט כפתור החירום | `estop=1` (חוט קרוע עוצר) |
| 16 | סקופ או נורית על פין STEP של X בזמן `MOVE X 2000` | פולסים; נפסקים מיד ב-`STOP` |
| 17 | `HOME X` (בלי מפסקי גבול) | `OK ZEROED_HERE`, ו-`STATUS` מראה `pos=0` ו-`homed=1000` |

אם שורה כלשהי נכשלת: לעצור, לא לחבר מנועים.

## שלב ב': מנוע אחד, בלי עומס, רק אחרי ששלב א' עבר ואחרי אישור

1. לוודא שהגבלת הזרם בדרייבר מכוונת למנוע.
2. לחבר מנוע אחד (X), לא מחובר למכניקה.
3. יד על כפתור החירום.
4. `PING`, ואז `SPEED X 200 400` (איטי), ואז `MOVE X 200`. המנוע מסתובב סיבוב אחד בערך ונעצר.
5. `MOVE X -200`. חוזר לכיוון ההפוך.
6. תוך כדי `MOVE X 5000` ללחוץ חירום: המנוע נעצר מיד ומשתחרר.
7. תוך כדי `MOVE X 5000` להפסיק לשלוח `PING`: נעצר אחרי שנייה.
8. לחזור על 4–7 לכל ציר בנפרד.

## שלב ג': דרך Python (אחרי שלב ב')

מ-Windows, עם ה-venv פעיל:

```bash
python -c "from rada_align.config import load_config; from rada_align.hw.esp32_actuators import ESP32Actuators; a = ESP32Actuators(load_config()); print(a.status()); a.close()"
```

אם ציר נע בכיוון ההפוך מהמוגדר ב-`rada_align/interfaces.py` (‎+x ימינה מנקודת המבט של ה-horn, ‎+z למעלה, ‎+pan ימינה, ‎+tilt למעלה), להפוך את הסימן שלו ב-`hw.direction` ב-`config.yaml`.

## מה לא נבדק

הקושחה נבנתה אבל לא רצה על לוח. הדרייבר ב-Python נבדק רק מול פורט מדומה שמחקה את הפרוטוקול.
