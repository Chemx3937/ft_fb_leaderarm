"""Live diagnostic logging and an independent detector-contract audit; no ROS."""
import copy
import json

import numpy as np
import pytest

from ft_fb_leaderarm.comparison import ComparisonEngine


class Predictor:
    history = 1
    model_sha256 = metadata_sha256 = 'test'

    def predict(self, window):
        return np.zeros(6)


def trace_rows():
    engine = ComparisonEngine({'a':Predictor(), 'b':Predictor(), 'c':Predictor()})
    context = {'models':list(engine.names), 'initial_detectors':{
        n:{'contact':False, 'pending_since_s':None} for n in engine.names}}
    entries=[]
    for i in range(90):
        t=1+i*.004
        force=3.2 if 30<=i<60 else 2. if 60<=i<65 else 0.
        state={'stamp':t,'received':t,'frame':'right_base_link','sequence':i,'valid':True,'q':np.zeros(6),'dq':np.zeros(6)}
        wrist={'stamp':t,'received':t,'frame':'aft_sensor2','wrench':np.array([force,0,0,0,0,0])}
        row=engine.step(t,state,wrist)
        # The external load can be below the old 1.6N reference threshold.
        row['reference_force_n']=.8
        entries.append({'index':i+1,'phase':'pre_free' if i<20 else 'approach' if i<60 else 'release',
                        'completed_s':t+.001,'duration_s':.001,'row':row,'inputs':{'robot':state,'wrist':wrist,'external':None}})
    return engine,context,entries


def test_live_session_rejects_a_different_detector_before_loading_models(tmp_path):
    from ft_fb_leaderarm.comparison import load_comparison
    from ft_fb_leaderarm.comparison_trace import LIVE_RAMP_PROTOCOL,DETECTOR
    path=tmp_path/'session.json'
    path.write_text(json.dumps({'ramp_trial':{'mode':LIVE_RAMP_PROTOCOL},
                               'observer_detector':{**DETECTOR,'force_on_n':2.5}}))
    with pytest.raises(ValueError,match='fixed3.0/1.2'):load_comparison(path)


def test_live_detector_contract_checks_on_off_hold_and_hysteresis():
    from ft_fb_leaderarm.comparison_trace import audit_detector_trace
    _,context,entries=trace_rows()
    result=audit_detector_trace(entries,context)
    assert result['models']['a']['verdict']=='PASS'
    broken=copy.deepcopy(entries)
    broken[30]['row']['states']['a']='CONTACT'
    result=audit_detector_trace(broken,context)
    assert result['models']['a']['verdict']=='VIOLATION'
    assert result['models']['b']['verdict']=='PASS'


def test_live_detection_force_is_not_gated_by_external_contact_threshold():
    from ft_fb_leaderarm.comparison_trace import live_ramp_metrics
    _,context,entries=trace_rows()
    times=np.arange(.9,1.5,.002)
    forces=np.full(len(times),.8)
    result=live_ramp_metrics(entries,context,times,forces,30.)
    assert result['a']['status']=='DETECTED'
    assert result['a']['external_force_n']==pytest.approx(.8)
    assert result['a']['reference_valid']


def test_live_trace_writer_saves_every_tick_and_records_actual_inputs(tmp_path):
    from ft_fb_leaderarm.comparison_trace import LiveTraceWriter
    engine,_,entries=trace_rows()
    writer=LiveTraceWriter(tmp_path,engine)
    for e in entries:
        writer.phase=e['phase'];writer.submit(e['row'],e['inputs'],e['completed_s'],e['duration_s'])
    info=writer.close()
    saved=[json.loads(line) for line in (tmp_path/'live_decisions.jsonl').read_text().splitlines()]
    assert info['complete'] and info['written_rows']==90 and len(saved)==91
    assert saved[1]['inputs']['robot']['q']==[0.]*6
    assert saved[31]['row']['residual_norms']['a']==3.2
    assert saved[1]['phase']=='pre_free' and saved[-1]['phase']=='release'
    with pytest.raises(FileExistsError):LiveTraceWriter(tmp_path,engine)


def test_monitor_callback_captures_each_tick_without_a_ros_node_or_gui_refresh(tmp_path,monkeypatch):
    from types import SimpleNamespace
    import rclpy.node
    from ft_fb_leaderarm.comparison_gui import make_ros_node
    from ft_fb_leaderarm.comparison import ROBOT_TOPIC,WRIST_TOPIC,REFERENCE_TOPIC
    clock=[1.]
    class FakeBase:
        def __init__(self,*args):pass
        def create_subscription(self,*args):pass
        def create_timer(self,*args):pass
        def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9)))
    monkeypatch.setattr(rclpy.node,'Node',FakeBase)
    engine=ComparisonEngine({n:Predictor() for n in ('a','b','c')})
    node=make_ros_node(engine);node.start_trace(tmp_path)
    for i in range(12):
        clock[0]=1.+i*.004
        node.latest={ROBOT_TOPIC:{'stamp':clock[0],'received':clock[0],'frame':'right_base_link','sequence':i,
                                  'valid':True,'q':np.zeros(6),'dq':np.zeros(6)},
                     WRIST_TOPIC:{'stamp':clock[0],'received':clock[0],'frame':'aft_sensor2','wrench':np.zeros(6)},
                     REFERENCE_TOPIC:None}
        if i==4:node.trace_phase('approach')
        node.tick()
    info=node.finish_trace()
    assert info['complete'] and info['written_rows']==12 and node.trace is None
    saved=[json.loads(x) for x in (tmp_path/'live_decisions.jsonl').read_text().splitlines()][1:]
    assert saved[0]['phase']=='prepare' and saved[4]['phase']=='approach'
    assert all(set(e['row']['predictions'])=={'a','b','c'} for e in saved)
    assert all(e['completed_s']>=e['row']['t'] for e in saved)


def test_writer_reports_queue_overflow_instead_of_silently_losing_samples(tmp_path,monkeypatch):
    import threading
    from ft_fb_leaderarm import comparison_trace as module
    engine,_,entries=trace_rows()
    original=module.json.dumps;entered=threading.Event();proceed=threading.Event()
    def slow(value,*args,**kwargs):
        if isinstance(value,dict) and value.get('type')=='tick':
            entered.set();assert proceed.wait(3.)
        return original(value,*args,**kwargs)
    monkeypatch.setattr(module.json,'dumps',slow)
    writer=module.LiveTraceWriter(tmp_path,engine,queue_size=1)
    e=entries[0]
    def submit():writer.submit(e['row'],e['inputs'],e['completed_s'],e['duration_s'])
    submit();assert entered.wait(1.)
    submit();submit();proceed.set()
    info=writer.close()
    assert not info['complete'] and info['dropped_rows']==1 and info['written_rows']==2


def test_pre_activation_unpaired_force_and_post_upper_detection_are_separate():
    from ft_fb_leaderarm.comparison_trace import live_ramp_metrics
    _,context,entries=trace_rows()
    times=np.arange(.9,1.5,.002);forces=np.full(len(times),4.)
    early=copy.deepcopy(entries);early[19]['row']['states']['a']='CONTACT'
    assert live_ramp_metrics(early,context,times,forces,30.)['a']['status']=='DETECTED'
    early[20]['row']['states']['a']='CONTACT'
    assert live_ramp_metrics(early,context,times,forces,30.)['a']['status']=='PREACTIVATED'
    assert live_ramp_metrics(entries,context,[],[],30.)['a']['status']=='EXTERNAL_UNPAIRED'
    forces[times>=1.09]=30.
    assert live_ramp_metrics(entries,context,times,forces,30.)['a']['status']=='DETECTED_AFTER_UPPER'


def test_duplicate_input_does_not_hide_a_live_contact_transition():
    from ft_fb_leaderarm.comparison_trace import live_ramp_metrics
    _,context,entries=trace_rows()
    edge=next(i for i,e in enumerate(entries) if e['row']['states']['a']=='CONTACT')
    entries[edge-1]['row']['states']['a']='DUPLICATE'
    times=np.arange(.9,1.5,.002)
    assert live_ramp_metrics(entries,context,times,np.ones(len(times)),30.)['a']['status']=='DETECTED'


def test_residual_corruption_and_missing_tick_are_not_reported_as_contract_pass():
    from ft_fb_leaderarm.comparison_trace import audit_detector_trace
    _,context,entries=trace_rows()
    entries[35]['row']['residual_norms']['a']=.1
    result=audit_detector_trace(entries,context)
    assert result['models']['a']['verdict']=='VIOLATION'
    assert result['models']['b']['verdict']=='PASS'
    result=audit_detector_trace(entries[:10]+entries[11:],context)
    assert result['models']['b']['verdict']=='INCOMPLETE'


def test_short_threshold_pulse_stays_free_and_a_pending_hold_crosses_capture_start(tmp_path):
    from ft_fb_leaderarm.comparison_trace import audit_detector_trace,LiveTraceWriter
    engine,context,entries=trace_rows()
    for i,e in enumerate(entries):
        force=3.2 if 30<=i<33 else 0.
        e['row']['measured'][0]=force
        for name in engine.names:
            e['row']['residual_norms'][name]=force;e['row']['states'][name]='FREE'
    assert audit_detector_trace(entries,context)['models']['a']['verdict']=='PASS'
    for detector in engine.detectors.values():detector.update(3.2,2.)
    writer=LiveTraceWriter(tmp_path,engine)
    row=copy.deepcopy(entries[0]['row']);row['t']=2.024;row['measured'][0]=3.2
    for name in engine.names:row['states'][name]='CONTACT';row['residual_norms'][name]=3.2
    writer.submit(row,{},2.025,.001);assert writer.close()['complete']
    saved=[json.loads(x) for x in (tmp_path/'live_decisions.jsonl').read_text().splitlines()]
    assert saved[0]['initial_detectors']['a']['pending_since_s']==2.
    assert audit_detector_trace(saved[1:],saved[0])['models']['a']['verdict']=='PASS'


@pytest.mark.parametrize('scenario',['violation','miss'])
def test_live_analysis_uses_saved_decisions_and_retains_contract_failure(tmp_path,monkeypatch,scenario):
    from ft_fb_leaderarm import comparison_trace as module
    from ft_fb_leaderarm import comparison
    # Known initial checkpoint and synthetic captured decisions; analysis must not regenerate them.
    engine=ComparisonEngine({n:Predictor() for n in ('a','b','c')});engine.reference_baseline=np.zeros(3)
    directory=tmp_path/'session_test'/'trials'/'one';directory.mkdir(parents=True)
    writer=module.LiveTraceWriter(directory,engine)
    _,_,entries=trace_rows()
    # Explicitly include a bad early decision: a failed contract must still be counted.
    entries[30]['row']['states']['a']='CONTACT'
    if scenario=='miss':
        for e in entries:
            e['row']['measured']=[0.]*6
            e['row']['states']={n:'FREE' for n in engine.names}
            e['row']['residual_norms']={n:0. for n in engine.names}
    for e in entries:
        writer.phase=e['phase'];writer.submit(e['row'],e['inputs'],e['completed_s'],e['duration_s'])
    writer.phase='post_free';e=entries[-1]
    writer.submit({**e['row'],'t':1.4},e['inputs'],1.401,.001)
    assert writer.close()['complete']
    manifest={'zero_set_id':'test','models':[{'name':n,'model_sha256':'test','metadata_sha256':'test'} for n in engine.names],
              'observer_detector':module.DETECTOR,'post_zero_static_diagnostic':{'report':'noise_baseline.json'},
              'ramp_trial':{'mode':module.LIVE_RAMP_PROTOCOL,'force_upper_n':30.,'physical_shared_trials':20}}
    (directory/'session.json').write_text(json.dumps(manifest))
    (directory/'noise_baseline.json').write_text(json.dumps({'zero_set_id':'test','sensors':{'1':{'force_median_n':[0.,0.,0.]}}}))
    (directory/'trial.json').write_text(json.dumps({'kind':'ramp','zero_set_id':'test','owner':'shared','complete':True,'guide_complete':True,'score_start_s':1.08}))
    times=np.arange(.9,1.5,.002);forces=np.where(times<1.2,4.,30.)
    monkeypatch.setattr(module,'read_external_samples',lambda *args:(times,forces))
    monkeypatch.setattr(comparison,'replay_frames',lambda *args:pytest.fail('must not replace live states with replay'))
    report=comparison.analyze_trial(directory,directory/'session.json')
    assert report['analysis_schema']==module.LIVE_RAMP_PROTOCOL
    assert report['protocol_quality']['plan_eligible']
    if scenario=='violation':assert report['detector_conformance']['models']['a']['verdict']=='VIOLATION'
    else:assert report['ramp_detection']['a']['status']=='NO_NEW_DETECTION'
    summary=json.loads((tmp_path/'live_ramp_summary.json').read_text())
    assert summary['planned_shared_trials']==20 and summary['physical_trials']==1
    assert summary['models']['a']['contract_violating_trials']==int(scenario=='violation')
    if scenario=='miss':assert summary['models']['a']['paired_detections']==0
    assert json.loads((directory/'trial.json').read_text())['analysis_complete']
    # An edited or truncated live trace cannot be silently accepted.
    with (directory/'live_decisions.jsonl').open('a') as stream:stream.write('\n')
    with pytest.raises(ValueError,match='hash mismatch'):module.analyze_live_ramp(directory,manifest)
