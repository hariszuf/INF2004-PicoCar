"""Check actual C printf schemas: fixed-column rows, formats and argument count."""
from pathlib import Path
import re

def calls(text):
    for match in re.finditer(r'printf\s*\(', text):
        pos=match.end(); start=pos; depth=1; quote=False; escaped=False
        while depth:
            c=text[pos]
            if quote:
                if escaped: escaped=False
                elif c=='\\': escaped=True
                elif c=='"': quote=False
            elif c=='"': quote=True
            elif c=='(': depth+=1
            elif c==')': depth-=1
            pos+=1
        body=text[start:pos-1]
        # Split arguments while respecting casts, conditionals and strings.
        parts=[]; begin=0; depth=0; quote=False; escaped=False
        for i,c in enumerate(body):
            if quote:
                if escaped: escaped=False
                elif c=='\\': escaped=True
                elif c=='"': quote=False
            elif c=='"': quote=True
            elif c=='(': depth+=1
            elif c==')': depth-=1
            elif c==',' and depth==0:
                parts.append(body[begin:i]); begin=i+1
        parts.append(body[begin:])
        fmt=''.join(re.findall(r'"((?:\\.|[^"\\])*)"',parts[0]))
        yield fmt,parts[1:]

for filename,expected in [('buddy4/main.c',39),('integration_test/main.c',79)]:
    text=(Path(__file__).parent.parent/filename).read_text()
    header=None; row=None; summary=None
    for fmt,args in calls(text):
        conversions=re.findall(r'%(?!%)[-+ #0]*\d*(?:\.\d+)?(?:hh|ll|[hljztL])?[diuoxXfFeEgGaAcsp]',fmt)
        assert len(conversions)==len(args),(filename,fmt,len(conversions),len(args))
        if fmt.startswith('seq,'): header=fmt.removesuffix(r'\n').split(',')
        if fmt.startswith('%lu,%llu,'): row=fmt.removesuffix(r'\n').split(',')
        if fmt.startswith('# SIDE_EVENT_END'): summary=fmt
    assert header and row and summary
    assert len(header)==len(row)==expected,(filename,len(header),len(row))
    assert len(header)==len(set(header))
    assert header[4:10]==['tilt_valid','acceleration_valid','measurement_quality',
        'clipped_axes','clipped_samples','unsuitable_samples']
    assert row[4:10]==['%d','%d','%s','%u','%lu','%lu']
    if filename.startswith('integration_test'):
        assert header[-18:-15]==['encoder_motion_state','encoder_motion_valid','encoder_motion_sample_us']
        assert row[-18:-15]==['%s','%d','%llu']
        assert header[-15:-7]==['possible_impact_valid','possible_impact_event',
            'possible_impact_count','possible_impact_sample_us','possible_impact_event_us',
            'possible_impact_magnitude_g','possible_impact_baseline_g','possible_impact_deviation_g']
        assert row[-15:-7]==['%d','%d','%lu','%llu','%llu','%.4f','%.4f','%.4f']
        assert header[-7:-3]==['encoder_yaw_rate_estimate_rad_s','encoder_yaw_rate_estimate_deg_s',
            'encoder_yaw_rate_estimate_valid','encoder_yaw_rate_estimate_sample_us']
        assert row[-7:-3]==['%.5f','%.4f','%d','%llu']
        assert header[-3:]==['selected_target_left_cm_s','selected_target_right_cm_s','control_reason']
        assert row[-3:]==['%.3f','%.3f','%s']
        i=header.index('stop_fault')
        assert header[i+1:i+5]==['terrain_mode','terrain_speed_scale','left_terrain_duty','right_terrain_duty']
        assert row[i+1:i+5]==['%d','%.3f','%.3f','%.3f']
        controls=[fmt for fmt,args in calls(text) if fmt.startswith('# CONTROL')]
        assert len(controls)==1
        for field in ['mode=','event=','selected_target_left_cm_s=','selected_target_right_cm_s=','turn_test_limit_us=']:
            assert field in controls[0]
    for name in ['median_candidate_valid','median_candidate_roll_deg','median_candidate_height_cm']:
        assert name in header and name+'=' in summary
    print(f'{filename}: {expected} columns; printf argument counts and candidate summaries passed')
