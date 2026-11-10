"""Capture actual comparison decisions and audit them without model replay."""
import csv
import hashlib
import json
import math
from pathlib import Path
import queue
import threading

import numpy as np

from .contract import SAMPLE_HZ


LIVE_RAMP_PROTOCOL = 'ramp_detection_live_v1'
DETECTOR = {'force_on_n':3.0, 'force_off_n':1.2, 'contact_hold_ms':20.0, 'free_hold_ms':20.0}


def _json_value(value):
    if isinstance(value, np.ndarray): return _json_value(value.tolist())
    if isinstance(value, np.generic): return _json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, dict): return {k:_json_value(v) for k,v in value.items()}
    if isinstance(value, (tuple,list)): return [_json_value(v) for v in value]
    return value


def _sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024), b''): digest.update(chunk)
    return digest.hexdigest()


class LiveTraceWriter:
    """Bounded queue keeps filesystem writes out of the inference callback."""
    def __init__(self, directory, engine, queue_size=4096):
        for detector in engine.detectors.values():
            if (detector.force_on_n,detector.force_off_n,detector.contact_hold_s,detector.free_hold_s)!=(3.,1.2,.020,.020):
                raise ValueError('live protocol requires the fixed3.0/1.2N20/20ms detector')
        self.directory=Path(directory)
        self.path=self.directory/'live_decisions.jsonl'
        self.stream=self.path.open('x',encoding='utf-8')
        self.phase='prepare'
        self.context={'type':'context', 'schema':LIVE_RAMP_PROTOCOL, 'models':engine.names,
                      'model_artifacts':{n:{'model_sha256':p.model_sha256,'metadata_sha256':p.metadata_sha256}
                                         for n,p in engine.predictors.items()},
                      'detector':dict(DETECTOR),
                      'reference_baseline_n':_json_value(engine.reference_baseline),
                      'initial_detectors':{n:{'contact':d.contact,'pending_since_s':d.pending_since_s}
                                           for n,d in engine.detectors.items()}}
        try:
            self.stream.write(json.dumps(_json_value(self.context),allow_nan=False)+'\n')
        except Exception:
            self.stream.close(); raise
        self.queue=queue.Queue(maxsize=queue_size)
        self.stop=threading.Event()
        self.submitted=self.written=self.dropped=0
        self.error=None
        self.thread=threading.Thread(target=self._run,name='comparison_trace_writer',daemon=True)
        self.thread.start()

    def submit(self, row, inputs, completed_s, duration_s):
        self.submitted+=1
        packet={'type':'tick','index':self.submitted,'phase':self.phase,'completed_s':completed_s,
                'duration_s':duration_s,'row':row,'inputs':inputs}
        try: self.queue.put_nowait(packet)
        except queue.Full: self.dropped+=1

    def _run(self):
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try: packet=self.queue.get(timeout=.1)
                except queue.Empty: continue
                try:
                    if self.error is None:
                        self.stream.write(json.dumps(_json_value(packet),allow_nan=False)+'\n')
                        self.written+=1
                except Exception as exc: self.error=str(exc)
                finally: self.queue.task_done()
        finally:
            try: self.stream.close()
            except Exception as exc: self.error=str(exc)

    def close(self):
        self.stop.set(); self.thread.join(timeout=5.)
        info={'schema':LIVE_RAMP_PROTOCOL,'submitted_rows':self.submitted,'written_rows':self.written,
              'dropped_rows':self.dropped,'write_error':self.error,'writer_stopped':not self.thread.is_alive()}
        info['complete']=bool(info['writer_stopped'] and self.error is None and self.dropped==0 and
                              self.written==self.submitted and self.written>0)
        info['sha256']=_sha(self.path) if info['writer_stopped'] else None
        (self.directory/'live_trace_info.json').write_text(json.dumps(info,indent=2)+'\n')
        return info


def audit_detector_trace(entries, context):
    """Independent threshold/hold calculation; does not call SchmittContactDetector."""
    times=np.array([e['row']['t'] for e in entries],dtype=float)
    gaps=int(np.sum(np.diff(times)>.020))
    clock_ok=bool(len(times)>0 and np.isfinite(times).all() and np.all(np.diff(times)>0))
    indices=[e['index'] for e in entries]
    complete_indices=indices==list(range(1,len(entries)+1))
    result={'definition':'actual logged GUI comparison decisions; threshold audit at tick-start ROS time',
            'clock_increases':clock_ok,'indices_contiguous':complete_indices,'gaps_over_20ms':gaps,
            'max_tick_interval_ms':float(np.max(np.diff(times))*1000) if len(times)>1 else None,
            'inference_deadline_exceedances':sum(e['duration_s']>1/SAMPLE_HZ for e in entries), 'models':{}}
    for name in context['models']:
        seed=context['initial_detectors'][name]
        active=bool(seed['contact']); pending=seed['pending_since_s']
        violations=[]; checked=invalid=0
        for entry in entries:
            row=entry['row']; state=row['states'].get(name,'MISSING'); t=float(row['t'])
            if state not in ('FREE','CONTACT'):
                invalid+=1
                if state not in ('DUPLICATE','WARMUP'): active,pending=False,None
                continue
            checked+=1
            try:
                if row.get('input_status','VALID')!='VALID' or not row.get('model_ready',{}).get(name,True):
                    violations.append({'t_s':t,'reason':'valid decision reported for invalid/not-ready input'})
                measured=np.asarray(row['measured'],float); predicted=np.asarray(row['predictions'][name],float)
                norm=float(np.linalg.norm(measured[:3]-predicted[:3]))
                recorded=float(row['residual_norms'][name])
                if measured.shape!=(6,) or predicted.shape!=(6,) or not math.isfinite(norm) or not math.isfinite(recorded):
                    raise ValueError('invalid measured/predicted/residual force')
                if not math.isclose(norm,recorded,rel_tol=1e-6,abs_tol=1e-6):
                    violations.append({'t_s':t,'reason':'logged residual differs from measured minus predicted force'})
                met=norm<=1.2 if active else norm>=3.
                if not met: pending=None
                else:
                    if pending is None: pending=t
                    if t-pending>=.020: active,pending=not active,None
                expected='CONTACT' if active else 'FREE'
                if state!=expected: violations.append({'t_s':t,'reason':'threshold/hold/hysteresis mismatch','expected':expected,'actual':state})
            except (KeyError,TypeError,ValueError) as exc:
                violations.append({'t_s':t,'reason':str(exc)})
        verdict='VIOLATION' if violations else 'INCOMPLETE' if not checked or not clock_ok or not complete_indices or gaps else 'PASS'
        result['models'][name]={'verdict':verdict,'valid_ticks_checked':checked,'invalid_or_warmup_ticks':invalid,
                                'violation_count':len(violations),'violations':violations}
    return result


def live_ramp_metrics(entries, context, external_times, external_forces, upper_n):
    """First new loading-phase decision, without an external CONTACT gate."""
    external_times=np.asarray(external_times); external_forces=np.asarray(external_forces)
    loading=[i for i,e in enumerate(entries) if e['phase']=='approach']
    result={}
    start=entries[loading[0]]['row']['t'] if loading else math.inf
    caps=np.flatnonzero((external_times>=start)&(external_forces>=upper_n))
    cap_time=float(external_times[caps[0]]) if len(caps) else math.inf
    for name in context['models']:
        previous=bool(context['initial_detectors'][name]['contact'])
        before=[]; edges=set()
        for i,entry in enumerate(entries):
            before.append(previous)
            state=entry['row']['states'].get(name,'MISSING')
            if state in ('FREE','CONTACT'):
                if state=='CONTACT' and not previous: edges.add(i)
                previous=state=='CONTACT'
            elif state not in ('DUPLICATE','WARMUP'): previous=False
        baseline=[e for e in entries if e['phase']=='pre_free']
        item={'status':'NO_NEW_DETECTION','t_s':None,'external_force_n':None,'reference_valid':False,
              'residual_force_n':None,'pre_free_contact_ticks':sum(e['row']['states'].get(name)=='CONTACT' for e in baseline),
              'preactivated_at_loading_start':False,'local_force_rise_rate_n_s':None}
        item['invalid_loading_ticks']=sum(entries[i]['row']['states'].get(name) not in ('FREE','CONTACT') for i in loading)
        if not loading: item['status']='NO_LOADING_PHASE'; result[name]=item; continue
        first=loading[0]
        if before[first] and entries[first]['row']['states'].get(name)=='CONTACT':
            item.update(status='PREACTIVATED',preactivated_at_loading_start=True);result[name]=item;continue
        for i in loading:
            if i not in edges: continue
            e=entries[i]; event_time=float(e['completed_s'])
            item.update(t_s=event_time,tick_start_s=float(e['row']['t']),residual_force_n=float(e['row']['residual_norms'][name]))
            if event_time>cap_time: item['status']='DETECTED_AFTER_UPPER';break
            j=int(np.searchsorted(external_times,event_time,side='right'))-1
            paired=j>=0 and 0<=event_time-external_times[j]<=.003
            item.update(status='DETECTED' if paired else 'EXTERNAL_UNPAIRED',reference_valid=bool(paired))
            if paired:
                item.update(external_force_n=float(external_forces[j]),external_stamp_s=float(external_times[j]),
                            pairing_age_ms=1000*(event_time-float(external_times[j])))
                window=(external_times>=event_time-.25)&(external_times<=event_time)
                if int(window.sum())>=3 and np.ptp(external_times[window])>=.1:
                    item['local_force_rise_rate_n_s']=float(np.polyfit(external_times[window]-event_time,external_forces[window],1)[0])
            break
        if item['status']=='NO_NEW_DETECTION' and item['invalid_loading_ticks']:
            item['status']='NO_DETECTION_WITH_INVALID_TICKS'
        result[name]=item
    return result


def read_external_samples(bag_path, baseline):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import WrenchStamped
    from .contract import stamp_to_seconds,wrench_from_message
    from .comparison import REFERENCE_TOPIC
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    samples={}
    while reader.has_next():
        topic,blob,_=reader.read_next()
        if topic!=REFERENCE_TOPIC: continue
        msg=deserialize_message(blob,WrenchStamped)
        values=wrench_from_message(msg); t=stamp_to_seconds(msg.header.stamp)
        if msg.header.frame_id=='aft_sensor1' and math.isfinite(t) and np.isfinite(values).all():
            samples[t]=float(np.linalg.norm(values[:3]-baseline))
    times=np.array(sorted(samples))
    return times,np.array([samples[t] for t in times])


def analyze_live_ramp(directory, manifest):
    directory=Path(directory); metadata=json.loads((directory/'trial.json').read_text())
    info=json.loads((directory/'live_trace_info.json').read_text())
    path=directory/'live_decisions.jsonl'
    if not info.get('complete') or info.get('sha256')!=_sha(path):
        raise ValueError('live trace incomplete or hash mismatch; raw data retained')
    with path.open() as stream:
        context=json.loads(next(stream)); entries=[json.loads(line) for line in stream]
    if context.get('schema')!=LIVE_RAMP_PROTOCOL or context.get('detector')!=DETECTOR:
        raise ValueError('live trace schema or detector configuration mismatch')
    expected={x['name']:{k:x[k] for k in ('model_sha256','metadata_sha256')} for x in manifest['models']}
    if context['model_artifacts']!=expected or manifest['observer_detector']!=DETECTOR:
        raise ValueError('live trace model hashes or observer parameters differ from session')
    if len(entries)!=info['written_rows'] or not entries: raise ValueError('live trace row count mismatch')
    noise_path=directory/manifest['post_zero_static_diagnostic']['report']
    noise=json.loads(noise_path.read_text())
    if noise['zero_set_id']!=manifest['zero_set_id']: raise ValueError('external baseline zero-set differs')
    baseline=np.array(noise['sensors']['1']['force_median_n'])
    if not np.array_equal(np.asarray(context['reference_baseline_n']),baseline):
        raise ValueError('live and stored external baselines differ')
    external_times,external_forces=read_external_samples(directory/'raw',baseline)
    upper=float(manifest['ramp_trial']['force_upper_n'])
    metrics=live_ramp_metrics(entries,context,external_times,external_forces,upper)
    audit=audit_detector_trace(entries,context)
    phases={e['phase'] for e in entries}
    loading_start=next((e['row']['t'] for e in entries if e['phase']=='approach'),math.inf)
    loading_end=max((e['completed_s'] for e in entries if e['phase']=='approach'),default=-math.inf)
    cap_reached=bool(np.any((external_times>=loading_start)&(external_times<=loading_end+.003)&(external_forces>=upper)))
    result={'analysis_schema':LIVE_RAMP_PROTOCOL,'kind':'ramp','owner':'shared','zero_set_id':manifest['zero_set_id'],
            'source':'actual per-tick live decisions; no model replay; external raw header stamps aligned to decision completion',
            'planned_shared_trials':int(manifest['ramp_trial']['physical_shared_trials']),
            'detector':DETECTOR,'detector_conformance':audit,'ramp_detection':metrics,
            'ramp_onsets':{n:v if v['t_s'] is not None else None for n,v in metrics.items()},
            'trace_integrity':info,'live_ticks':len(entries),
            'protocol_quality':{'plan_eligible':bool(metadata.get('guide_complete') and cap_reached and {'pre_free','approach','release','post_free'}.issubset(phases)),
                                'upper_force_reached':cap_reached,
                                'selection_basis':'completed load/release procedure and complete live trace; model miss, preactivation or contract violation never excludes a trial'},
            'limitations':['External sensor is a load meter, not a binary contact gate or exact physical-touch annotation.',
                           'Force at detection includes model computation and detector hold; loading rate and sensor noise affect the result.',
                           'PASS audits observed valid ticks; gaps/invalid ticks are reported and do not establish full runtime acceptance.',
                           'Current-zero sensor measurements are not a full-range absolute force calibration.',
                           'These are the three diagnostic observers in this GUI, not a new canonical observer or feedback approval.']}
    arrays={'t_s':np.array([e['row']['t'] for e in entries]),'completed_s':np.array([e['completed_s'] for e in entries]),
            'phase':np.array([e['phase'] for e in entries]),
            'measured':np.array([e['row']['measured'] if e['row']['measured'] is not None else [np.nan]*6 for e in entries])}
    for name in context['models']:
        arrays[name+'_prediction']=np.array([e['row']['predictions'].get(name,[np.nan]*6) for e in entries])
        arrays[name+'_state']=np.array([e['row']['states'].get(name,'MISSING') for e in entries])
    np.savez_compressed(directory/'comparison.npz',**arrays)
    (directory/'comparison_report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    metadata['analysis_complete']=True
    (directory/'trial.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n')
    summarize_live_ramps(directory.parents[2])
    return result


def summarize_live_ramps(root):
    root=Path(root); reports=[]; csv_rows=[]
    for p in sorted(root.glob('session_*/trials/*/comparison_report.json')):
        report=json.loads(p.read_text()); meta=json.loads((p.parent/'trial.json').read_text())
        if report.get('analysis_schema')!=LIVE_RAMP_PROTOCOL or not meta.get('complete') or not report['protocol_quality']['plan_eligible']: continue
        reports.append(report)
        for name,metric in report['ramp_detection'].items():
            csv_rows.append({'trial':str(p.parent.relative_to(root)),'model':name,'status':metric['status'],
                             'decision_t_s':metric['t_s'],'external_force_n':metric['external_force_n'],
                             'detector_contract':report['detector_conformance']['models'][name]['verdict']})
    summary={'protocol':LIVE_RAMP_PROTOCOL,'planned_shared_trials':20,'physical_trials':len(reports),'models':{}}
    names=reports[0]['ramp_detection'].keys() if reports else []
    for name in names:
        metrics=[r['ramp_detection'][name] for r in reports]
        values=[v['external_force_n'] for v in metrics if v['status']=='DETECTED']
        summary['models'][name]={'paired_detections':len(values),
            'pre_free_contact_trials':sum(v['pre_free_contact_ticks']>0 for v in metrics),
            'status_counts':{status:sum(v['status']==status for v in metrics) for status in sorted({v['status'] for v in metrics})},
            'force_min_n':min(values) if values else None,'force_median_n':float(np.median(values)) if values else None,
            'force_max_n':max(values) if values else None,'force_std_n':float(np.std(values)) if values else None,
            'contract_violating_trials':sum(r['detector_conformance']['models'][name]['verdict']=='VIOLATION' for r in reports),
            'contract_incomplete_trials':sum(r['detector_conformance']['models'][name]['verdict']=='INCOMPLETE' for r in reports)}
    (root/'live_ramp_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    with (root/'live_ramp_onsets.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['trial','model','status','decision_t_s','external_force_n','detector_contract'])
        writer.writeheader();writer.writerows(csv_rows)
    return summary
