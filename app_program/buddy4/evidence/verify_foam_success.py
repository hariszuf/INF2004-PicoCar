"""Verify immutable 86-column hardware captures; never rewrite original bytes.

Run without arguments to check the saved manifest, or --write-manifest to
regenerate only the derived JSON. This parses recorded outputs, not a firmware
simulation or proof of contact timing. Operator confirmation is explicit.
"""
import csv
import hashlib
import json
import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MANIFEST = ROOT / '2026-11-02' / 'foam_success_manifest.json'
CASES = (
    ('Pasted text(20261102-074240).txt', 722, 5, 9160237, True),
    ('Pasted text(20261102-074448).txt', 669, 2, 10880237, False),
)


def analyse(name, expected_rows, expected_bad, expected_event, confirmed):
    path = ROOT / name
    raw = path.read_bytes()
    rows, controls, sent = [], [], []
    rejected = 0
    for line in raw.decode('utf-8-sig').splitlines():
        data = line.split(' -> ', 1)[-1]
        control = re.search(r'# CONTROL,time_us=(\d+),event=([^,]+)', data)
        if control:
            controls.append((int(control[1]), control[2]))
        send = re.search(r'(\d\d:\d\d:\d\d:\d{3}) ---- Sent utf8 encoded message: "S"', line)
        if send:
            sent.append(send[1])
        if not re.match(r'^\d+,', data):
            continue
        fields = next(csv.reader([data]))
        try:
            assert len(fields) == 86
            for i, value in enumerate(fields):
                if i not in (6, 61, 78):
                    float(value)
        except (ValueError, AssertionError):
            rejected += 1
            continue
        rows.append((line.split(' -> ', 1)[0], fields))
    assert len(rows) == expected_rows and rejected == expected_bad, name
    assert all(r[3:7] == ['1', '1', '1', 'GRAVITY_LIKE'] and
               r[7] == '0' and r[11] == '0' for _, r in rows), name
    counts = [int(r[82]) for _, r in rows]
    assert counts[0] == 0 and counts[-1] == 1
    assert sum(b > a for a, b in zip(counts, counts[1:])) == 1
    assert all(a <= b for a, b in zip(counts, counts[1:]))
    pulses = [(host, r) for host, r in rows if r[81] == '1']
    assert len(pulses) == 2 and {int(r[84]) for _, r in pulses} == {expected_event}
    assert all(r[34] == '1' and r[35] == '0' and r[45:48] == ['1', '0', '0'] and
               float(r[76]) == float(r[77]) == 3 and
               float(r[42]) <= .5 and float(r[43]) <= .5 for _, r in pulses)
    host, first = pulses[0]
    index = next(i for i, (_, r) in enumerate(rows) if r is first)
    assert any(r[80] == '1' for _, r in rows[:index])
    stop = next(t for t, event in controls if event == 'SERIAL_STOP' and t > expected_event)
    stopped_host, stopped = next((h, r) for h, r in rows[index:] if r[34] == '0')
    assert stopped[78] == 'SERIAL_STOP' and stopped[45:47] == ['3', '1']
    assert all(r[66] == '0' and r[65] == '0' for _, r in rows)
    return {
        'file': name,
        'sha256': hashlib.sha256(raw).hexdigest(),
        'bytes': len(raw),
        'complete_rows': len(rows),
        'rejected_numeric_lines': rejected,
        'driving_rows': sum(r[34] == '1' for _, r in rows),
        'quality_counts': dict(Counter(r[6] for _, r in rows)),
        'operator_confirmed_physical_foam_contact': confirmed,
        'independently_timestamped_contact': False,
        'obstruction_count_change': [counts[0], counts[-1]],
        'distinct_obstruction_events': 1,
        'event_flag_rows': len(pulses),
        'first_event_host_timestamp': host,
        'event_observation_us': expected_event,
        'slow_ms_at_event': int(first[85]),
        'event_left_right_cm_s': [float(first[42]), float(first[43])],
        'serial_stop_control_us': stop,
        'event_to_stop_control_us': stop - expected_event,
        'first_stopped_csv_host_timestamp': stopped_host,
        'first_stopped_csv_time_us': int(stopped[1]),
        'event_to_first_stopped_csv_us': int(stopped[1]) - expected_event,
        'host_S_send_timestamp': sent[0],
        'new_possible_impact_events': 0,
        'firmware_identity': 'Updated SDK integration firmware, operator-reported; no binary hash in capture',
    }


def main():
    result = [analyse(*case) for case in CASES]
    if sys.argv[1:] == ['--write-manifest']:
        MANIFEST.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    else:
        assert not sys.argv[1:], 'Usage: verify_foam_success.py [--write-manifest]'
        assert result == json.loads(MANIFEST.read_text(encoding='utf-8')), 'Evidence/manifest mismatch'
    print(json.dumps(result, indent=2))
    print('Verified two distinct recorded possible-obstruction events; originals unchanged.')


if __name__ == '__main__':
    main()
