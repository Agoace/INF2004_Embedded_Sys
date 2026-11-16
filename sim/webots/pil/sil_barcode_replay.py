"""Old/new window component replay on identical native acquisition publications.

No physics, pose-to-controller input, or invented control permission. Control
values are captured from actual jlive publications, then held identical for the
component comparison. The feedback path is NOT being replayed in that comparison.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
from .sil_junction_barcode import BarcodeController,build_barcode_library

REASONS=dict(enumerate(('BW_WAIT','BW_OK','BW_CALIBRATION','BW_LOSS','BW_MOTION',
    'BW_START','BW_WIDTH','BW_GAP','BW_EXTENT','BW_TAIL','BW_NOT_LETTER')))
REASONS.update(dict(enumerate(('BP_CONTROL','BP_DISABLED','BP_CONTEXT','BP_CUT',
    'BP_CALIBRATION','BP_RECORD_AGE','BP_MOTOR_AGE','BP_CLOCK'),100)))

def read_rows(project):
    with (Path(project)/'sil-result/samples.csv').open(newline='')as f:
        reader=csv.reader(f);next(reader);return [list(map(float,r))for r in reader]


def gp28_x_mm(row):
    return row[12]*1000+80*math.cos(row[15])-77*math.sin(row[15])


def motion_truth(event,rows):
    """Net barcode-world-X progress over the ORIGINAL native motor bracket."""
    if event['reason']!=4:return None
    lookup={int(r[0])*1000:r for r in rows}
    old,new=event['prior_motor_us'],event['motor_us']
    delta=lambda a,b:((a-b+2**31)%2**32)-2**31
    dl,dr=delta(event['left'],event['prior_left']),delta(event['right'],event['prior_right'])
    result=dict(prior_motor_us=old,motor_us=new,delta_left=dl,delta_right=dr,
        zero_count_update=dl==0 and dr==0,motor_faults=event['motor_faults'],
        truth_scope='diagnostic flat GP28 mount projection on barcode world X; no controller input')
    if event['prior_valid']and old in lookup and new in lookup:
        dx=gp28_x_mm(lookup[new])-gp28_x_mm(lookup[old])
        result.update(gp28_projected_dx_mm=dx,net_truth_reversal=dx < -1e-7,
            net_truth_zero=abs(dx)<=1e-7,sign_tolerance_mm=1e-7)
    else:result.update(gp28_projected_dx_mm=None,net_truth_reversal=None,unavailable='missing original pose bracket')
    return result


def attempts(details,rows):
    result={}
    for d in details:
        key=(d['sim_ms'],d['reason'])
        item=result.setdefault(key,dict(d,window_event=False,producer_event=False))
        item['window_event']|=d['kind']==1;item['producer_event']|=d['kind']==2
        item['reason_name']=REASONS.get(d['reason'],'UNKNOWN')
        item['letter_name']=chr(d['letter'])if d['letter']else None
        item['motion_truth']=motion_truth(d,rows)
    return list(result.values())


def validate_inputs(inputs,rows):
    if len(inputs)!=len(rows):raise ValueError('acquisition stream length mismatch')
    snapshots={int(r[0])*1000:r for r in rows}
    for i,r in zip(inputs,rows):
        if len(i)!=18 or [i[0],i[9]]!=[int(r[0]),int(r[36])]:raise ValueError('ADC/time acquisition mismatch')
        # Held10ms publication, NOT the1ms encoder input; preserve its stamp.
        bracket=snapshots.get(i[3])
        expected=[int(bracket[22]),int(bracket[23])]if bracket else[0,0]if i[3]==0 else None
        if [i[4],i[5]]!=expected:raise ValueError('original motor publication mismatch')


def compare(project,output,*,capture_reference=False):
    project=Path(project);output=Path(output)
    if output.exists():raise ValueError('new comparison output required')
    output.mkdir(parents=True)
    rows=read_rows(project);config=json.loads((project/'sil-manifest.json').read_text())
    controllers={};builds={}
    for label,rev in (('old','cbcb557'),('new','52244f2')):
        library=build_barcode_library(output/label,window_revision=rev)
        c=BarcodeController(library);c.calibrate(config['calibration_rows'])
        report=c.barcode_calibrate(config['barcode_calibration_rows'],config['barcode_sigma_along_mm'])
        if report!=config['barcode_calibration']:raise ValueError('actual calibration replay mismatch')
        controllers[label]=c;builds[label]=json.loads((output/label/'sil-build.json').read_text())
    if builds['old']['sources_sha256'].keys()!=builds['new']['sources_sha256'].keys():raise ValueError('source set mismatch')
    different=[k for k,v in builds['old']['sources_sha256'].items()if builds['new']['sources_sha256'].get(k)!=v]
    if set(different)!={'app_program/subsystem3/barcode_window.c','app_program/subsystem3/barcode_window.h'}:
        raise ValueError('comparison differs beyond the two window source files')
    if capture_reference:
        c=controllers['old']
        for r in rows:
            value=list(c.update(int(r[1]),int(r[2]),int(r[0]),int(r[22]),int(r[23]),int(r[36])).values())
            if value[:21]!=r[3:12]+r[24:36]:raise ValueError('recorded native vehicle/motor invariants mismatch')
        inputs=c.inputs();reference_details=c.details()
        reference='old C native publication replay; vehicle/motor outputs MATCH original prefix'
    else:
        inputs=json.loads((project/'sil-result/barcode-inputs.json').read_text())
        reference_details=json.loads((project/'sil-result/barcode-details.json').read_text())
        reference='actual captured native jlive publication stream'
    validate_inputs(inputs,rows)
    (output/'inputs.json').write_text(json.dumps(inputs,separators=(',',':'))+'\n')
    raw_results={label:c.component_replay(inputs)for label,c in controllers.items()}
    if raw_results['old'if capture_reference else'new']!=reference_details:
        raise ValueError('component does not reproduce original native event stream')
    results={label:attempts(events,rows)for label,events in raw_results.items()}
    result=dict(scope='C component replay, NOT repeated physics; recorded control/ACK stream fixed, not generated by each variant',
        recorded_native_component_match=True,reference=reference,source_project=str(project),original_status=json.loads((project/'sil-result/summary.json').read_text())['status'],
        samples=len(rows),inputs_sha256=hashlib.sha256((output/'inputs.json').read_bytes()).hexdigest(),
        source_differences=different,results=results)
    (output/'comparison.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    return result
