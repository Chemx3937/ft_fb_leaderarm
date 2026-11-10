import numpy as np
import pytest
from pathlib import Path

from ft_fb_leaderarm.comparison import (
    ComparisonEngine, REFERENCE_TOPIC, ROBOT_TOPIC, WRIST_TOPIC,
    TRIAL_TOPICS, averaged_reference_force, external_force_bins, force_band, noise_summary, ramp_onsets,
)


class Predictor:
    def __init__(self, history=4, failure=False):
        self.history, self.failure, self.windows = history, failure, []

    def predict(self, window):
        self.windows.append(window.copy())
        if self.failure:
            raise RuntimeError("test model failure")
        return np.zeros(6)


def samples(t, seq, force=0., reference_force=2.):
    state = dict(stamp=t, received=t, frame="right_base_link", valid=True,
                 sequence=seq, q=np.zeros(6), dq=np.full(6, seq * .001))
    wrist = dict(stamp=t, received=t, frame="aft_sensor2", wrench=np.array([force,0,0,0,0,0]))
    reference = dict(stamp=t, received=t, frame="aft_sensor1", wrench=np.array([reference_force,0,0,0,0,0]))
    return state, wrist, reference


def engine(history=4):
    return ComparisonEngine({name: Predictor(history) for name in ("a", "b", "c")},
                            {"sensors":{num:{"force_median_n":[.1,0,0], "force_median_norm_n":.1,
                                             "force_axis_std_n":[.1,.1,.1],
                                             "centered_force_norm_p99_p99_9_max_n":[.2,.3,.4]}
                                        for num in ("1","2")}})


def test_shared_causal_history_and_stale_reset():
    e = engine()
    for i in range(4): row = e.step(1+i*.004, *samples(1+i*.004, i))
    assert set(row["states"].values()) == {"FREE"}
    windows = [p.windows[-1] for p in e.predictors.values()]
    assert all(np.array_equal(w, windows[0]) for w in windows)
    assert np.allclose(windows[0][-1,12:], .25)
    row = e.step(2., *samples(1.012, 3))
    assert set(row["states"].values()) == {"STALE / INVALID"}
    assert not row["predictions"] and not e.history
    assert set(e.step(2.004, *samples(2.004, 4))["states"].values()) == {"WARMUP"}


def test_bad_sync_duplicate_and_nonfinite_inputs_are_not_free():
    e = engine(1)
    row = e.step(1., *samples(1., 1))
    assert row["states"]["a"] == "FREE"
    assert e.step(1., *samples(1.,1))["states"]["a"] == "DUPLICATE"
    s,w,r = samples(2., 2); w["stamp"] -= .004
    assert e.step(2.,s,w,r)["states"]["a"] == "UNSYNCHRONIZED"
    s,w,r = samples(3.,3); w["wrench"][0] = np.nan
    row = e.step(3.,s,w,r)
    assert row["measured"] is None and not row["predictions"]


def test_native_history_lengths_and_one_model_failure_are_isolated():
    a,b,c = Predictor(1),Predictor(3),Predictor(1, failure=True)
    e = ComparisonEngine(dict(a=a,b=b,c=c))
    for i in range(3): row = e.step(1+i*.004,*samples(1+i*.004,i))
    assert a.windows[-1].shape==(1,18) and b.windows[-1].shape==(3,18)
    assert row["states"]["a"]==row["states"]["b"]=="FREE"
    assert row["states"]["c"].startswith("ERROR")


def test_contact_hold_ramp_force_and_missing_reference():
    e = engine(1); rows=[]
    for i in range(30):
        rows.append(e.step(1+i*.004,*samples(1+i*.004,i,force=3.2 if i>=15 else 0., reference_force=2.5 if i>=10 else .1)))
    assert rows[15]["states"]["a"]=="FREE" and rows[-1]["states"]["a"]=="CONTACT"
    result=ramp_onsets(rows,e.names)
    assert result["a"]["reference_valid"] and result["a"]["external_force_n"]==pytest.approx(2.4)
    assert result["a"]["t_s"]-rows[15]["t"]>=.020
    for row in rows: row["reference_stamp"]=row["wrist_stamp"]-.004
    assert ramp_onsets(rows,e.names)["a"]["external_force_n"] is None
    assert ramp_onsets(rows[:1],e.names)==dict.fromkeys(e.names)


def test_ramp_first_edge_before_external_contact_is_not_a_detection():
    e = engine(1); e.reference_baseline = np.zeros(3)
    rows = [e.step(1+i*.004, *samples(1+i*.004, i, force=4. if 15 <= i < 28 else 0.,
                                    reference_force=5. if i >= 40 else 0.)) for i in range(80)]
    # A real model activation before external loading must remain a false activation.
    assert any(r['states']['a'] == 'CONTACT' for r in rows[:40])
    assert ramp_onsets(rows,e.names)['a'] is None


def test_external_reference_and_observer_metrics_use_independent_thresholds():
    import json
    from ft_fb_leaderarm.comparison import external_reference_states, contact_metrics
    e = engine(1); e.reference_baseline = np.zeros(3)
    rows = [e.step(1+i*.004, *samples(1+i*.004, i, force=2.,
                                    reference_force=3. if 20 <= i < 50 else 0.)) for i in range(80)]
    labels = external_reference_states(rows, {})
    assert labels[0] == -1 and labels[10] == 0 and labels[30] == 1
    metrics = contact_metrics(rows,e.names,labels,np.ones(len(rows),dtype=bool))
    json.dumps(metrics,allow_nan=False)
    assert metrics['models']['a']['recall'] == 0.
    assert metrics['models']['a']['precision'] is None
    assert metrics['models']['a']['events'][0]['onset_latency_ms'] is None
    assert metrics['models']['a']['events'][0]['release_latency_ms'] is None


def test_reference_gap_is_unknown_and_release_latency_can_be_early():
    from ft_fb_leaderarm.comparison import external_reference_states, contact_metrics
    e = engine(1); e.reference_baseline = np.zeros(3)
    rows = [e.step(1+i*.004, *samples(1+i*.004, i, force=4. if 23 <= i < 50 else 0.,
                                    reference_force=5. if 20 <= i < 60 else 0.)) for i in range(90)]
    labels = external_reference_states(rows, {})
    event = contact_metrics(rows,e.names,labels,np.ones(len(rows),bool))['models']['a']['events'][0]
    assert event['onset_latency_ms'] > 0 and event['release_latency_ms'] < 0
    rows[65]['reference_force_n'] = None
    labels = external_reference_states(rows, {})
    assert labels[65] == -1 and labels[66] == -1 and labels[-1] == 0


def test_late_release_and_preactivation_are_not_invented_onsets():
    from ft_fb_leaderarm.comparison import external_reference_states, contact_metrics
    e = engine(1); e.reference_baseline = np.zeros(3)
    rows = [e.step(1+i*.004, *samples(1+i*.004,i,force=4. if 10 <= i < 65 else 0.,
                                    reference_force=5. if 20 <= i < 50 else 0.)) for i in range(90)]
    labels = external_reference_states(rows,{})
    event = contact_metrics(rows,e.names,labels,np.ones(len(rows),bool))['models']['a']['events'][0]
    assert event['preactivated_at_reference_onset']
    assert event['onset_latency_ms'] is None and event['release_latency_ms'] > 0
    assert ramp_onsets(rows,e.names) == dict.fromkeys(e.names)


def test_ramp_activation_after_force_cap_is_not_a_loading_detection():
    e = engine(1); e.reference_baseline = np.zeros(3)
    rows = [e.step(1+i*.004,*samples(1+i*.004,i,force=4. if i>=55 else 0.,
                                   reference_force=30. if i==50 else 5. if i>=20 else 0.)) for i in range(90)]
    assert ramp_onsets(rows,e.names)['a'] is not None
    assert ramp_onsets(rows,e.names,upper_n=30.)['a'] is None


def test_unknown_reference_and_unpaired_force_do_not_become_false_negatives():
    from ft_fb_leaderarm.comparison import external_reference_states, contact_metrics
    e = engine(1); e.reference_baseline = np.zeros(3)
    rows = [e.step(1+i*.004,*samples(1+i*.004,i,reference_force=5. if i>=20 else 0.)) for i in range(60)]
    rows[35]['reference_force_n'] = None
    rows[45]['reference_stamp'] -= .004
    labels = external_reference_states(rows,{})
    evaluation = contact_metrics(rows,e.names,labels,np.ones(len(rows),bool))
    assert evaluation['unknown_reference_samples'] > 0 and evaluation['unpaired_reference_samples'] == 2
    assert evaluation['scored_samples'] == int(np.sum(labels>=0))-1
    assert len(evaluation['reference_events']) == 2
    assert evaluation['reference_events'][0]['right_censored']
    assert evaluation['reference_events'][1]['left_censored']


def test_multiple_contacts_are_reported_as_separate_events():
    from ft_fb_leaderarm.comparison import external_reference_states, contact_metrics
    e = engine(1); e.reference_baseline = np.zeros(3)
    rows = [e.step(1+i*.004,*samples(1+i*.004,i,force=4. if 20<=i<45 else 0.,
                                   reference_force=5. if 20<=i<45 or 65<=i<90 else 0.)) for i in range(110)]
    labels = external_reference_states(rows,{})
    metrics = contact_metrics(rows,e.names,labels,np.ones(len(rows),bool))['models']['a']
    assert len(metrics['events']) == 2
    assert metrics['events'][0]['contact_observed'] and not metrics['events'][1]['contact_observed']
    assert metrics['events'][1]['onset_latency_ms'] is None


def test_noise_and_force_boundary_are_measurement_not_accuracy():
    n=noise_summary([[1.,2.,3.],[1.,2.,3.],[1.,2.,3.]])
    assert n["axis_std_n"]==[0.,0.,0.]
    assert n["baseline_norm_n"]==pytest.approx(np.sqrt(14))
    assert force_band(3.)=="≤ 3 N" and force_band(3.00001)=="> 3 N"
    assert force_band(float("nan"))==force_band(None)=="UNAVAILABLE"
    with pytest.raises(ValueError): noise_summary([[float("nan"),0,0]])


def test_guidance_averages_vectors_causally_and_rejects_missing_samples():
    rows=[]
    for i in range(27):
        wrench=np.array([4.+(.2 if i%2==0 else -.2),1. if i%2==0 else -1.,0.,0.,0.,0.])
        rows.append({'t':1.+i*.004,'reference_wrench':wrench,'reference_force_n':float(np.linalg.norm(wrench[:3]-[1,0,0]))})
    assert averaged_reference_force(rows,np.array([1,0,0]),.1)==pytest.approx(3.)
    assert averaged_reference_force(rows,None,.1) is None
    assert averaged_reference_force(rows[-2:],np.zeros(3),.1) is None
    rows[-1]['reference_force_n']=None
    assert averaged_reference_force(rows,np.zeros(3),.1) is None


def test_raw_force_bins_keep_boundaries_and_shared_valid_denominator():
    forces=np.array([0.,2.49,2.5,3.,3.01,3.5,3.51,np.nan,3.2])
    states=np.array(['FREE','CONTACT','FREE','CONTACT','FREE','CONTACT','CONTACT','CONTACT','CONTACT'])
    valid=np.ones(9,dtype=bool);valid[-1]=False
    result=external_force_bins(forces,states,valid)
    assert [v['samples'] for v in result.values()]==[2,2,2,1]
    assert [v['contact_fraction'] for v in result.values()]==[.5,.5,.5,1.]
    assert sum(v['samples'] for v in result.values())==7
    empty=external_force_bins([np.nan],['FREE'],[True])
    assert all(v['contact_fraction'] is None for v in empty.values())


def test_contact_analysis_exports_raw_bins_without_gui_averaging(tmp_path,monkeypatch):
    import json
    from ft_fb_leaderarm import comparison
    meta={'kind':'weak','owner':'shared','zero_set_id':'test','score_start_s':1.,'score_end_s':2.,'complete':True}
    (tmp_path/'trial.json').write_text(json.dumps(meta))
    e=engine(1);e.reference_baseline=np.zeros(3)
    rows=[e.step(1.1+i*.004,*samples(1.1+i*.004,i,reference_force=f)) for i,f in enumerate([2.9,3.2,3.8])]
    for p in e.predictors.values():p.model_sha256=p.metadata_sha256='test'
    monkeypatch.setattr(comparison,'load_comparison',lambda path:({'zero_set_id':'test'},e,None))
    monkeypatch.setattr(comparison,'replay_frames',lambda *args:rows)
    result=comparison.analyze_trial(tmp_path,tmp_path/'session.json')
    for value in result['models'].values():
        assert [v['samples'] for v in value['external_force_bins'].values()]==[0,1,1,1]
    assert np.load(tmp_path/'comparison.npz')['external_force_n'].tolist()==[2.9,3.2,3.8]


def test_weak_quality_counts_a_miss_but_rejects_sustained_strong_loading(tmp_path,monkeypatch):
    import json
    from ft_fb_leaderarm import comparison
    meta={'kind':'weak','owner':'shared','zero_set_id':'test','score_start_s':1.,'score_end_s':2.,'complete':True}
    e=engine(1);e.reference_baseline=np.zeros(3)
    for predictor in e.predictors.values():predictor.model_sha256=predictor.metadata_sha256='test'
    monkeypatch.setattr(comparison,'load_comparison',lambda path:({'zero_set_id':'test'},e,None))
    for load,eligible in [(3.,True),(8.,False)]:
        (tmp_path/'trial.json').write_text(json.dumps(meta))
        e.clear()
        rows=[e.step(1.+i*.004,*samples(1.+i*.004,i,reference_force=load if 20<=i<60 else 0.)) for i in range(90)]
        monkeypatch.setattr(comparison,'replay_frames',lambda *args:rows)
        report=comparison.analyze_trial(tmp_path,tmp_path/'session.json')
        assert report['protocol_quality']['plan_eligible'] is eligible
        assert report['contact_evaluation']['models']['a']['recall']==0.
        assert report['models']['a']['contact_events']==0


def test_free_records_no_external_sensor_and_ramp_requires_it():
    assert TRIAL_TOPICS["free"]==(ROBOT_TOPIC,WRIST_TOPIC)
    assert REFERENCE_TOPIC in TRIAL_TOPICS["ramp"]
    assert REFERENCE_TOPIC in TRIAL_TOPICS["noise"]


def test_missing_baseline_or_stale_reference_cannot_claim_below_three():
    e=ComparisonEngine({"a":Predictor(1)})
    assert e.step(1.,*samples(1.,1))["reference_band"]=="UNAVAILABLE"
    e=engine(1);s,w,r=samples(1.,1);r["received"]-=.1
    row=e.step(1.,s,w,r)
    assert row["reference_force_n"] is None and row["states"]["a"]=="FREE"


def test_gui_can_render_without_ros_or_starting_hardware(tmp_path, monkeypatch):
    import os
    os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
    from PyQt5.QtWidgets import QApplication
    from ft_fb_leaderarm.comparison_gui import ComparisonWindow
    import json
    app=QApplication.instance() or QApplication([])
    p=tmp_path/"session.json";p.write_text(json.dumps({"zero_set_id":"test"}))
    e=engine(1)
    for i in range(20): e.step(1+i*.004,*samples(1+i*.004,i,reference_force=3.2))
    window=ComparisonWindow(e,p);window.refresh();window.show();app.processEvents()
    assert "3.100" in window.reference.text() and "> 3 N" in window.reference.text()
    assert window.table.item(1,5).text()=="FREE" and not window.start.isEnabled()
    assert window.table.viewport().rect().contains(window.table.visualItemRect(window.table.item(3,5)))
    assert window.grab().save(str(tmp_path/"gui.png"))
    window.close()


@pytest.mark.parametrize("upper,force,reached",[(10.,9.9,False),(10.,10.,True),(10.,10.1,True),
                                             (30.,10.,False),(30.,29.9,False),(30.,30.,True),(30.,30.1,True)])
def test_configured_ramp_force_notice_is_display_only(tmp_path, upper, force, reached):
    import json, os
    os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
    from PyQt5.QtWidgets import QApplication
    from ft_fb_leaderarm.comparison_gui import ComparisonWindow
    app=QApplication.instance() or QApplication([])
    path=tmp_path/"session.json"
    path.write_text(json.dumps({"zero_set_id":"test","ramp_trial":{"force_upper_n":upper}}))
    e=engine(1);e.reference_baseline=np.zeros(3)
    e.step(1.,*samples(1.,1,reference_force=force))
    window=ComparisonWindow(e,path);window.kind.setCurrentIndex(3);window.refresh()
    assert f"점증 상한 {upper:g} N" in window.reference.text()
    assert ("도달:" in window.reference.text())==reached
    assert ("#8f2830" in window.reference.styleSheet())==reached
    assert ("가압 중단" in window.force_target.text())==reached
    assert f"눈금 {upper:g} N" in window.force_bar.format()
    assert window.process is None and window.active is None
    window.close()


@pytest.fixture
def contact_guide(tmp_path, monkeypatch, request):
    import json, os, threading
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication
    from ft_fb_leaderarm import comparison_gui as gui
    app = QApplication.instance() or QApplication([])
    clock = [1000.]
    monkeypatch.setattr(gui.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(gui.time, "time", lambda: 1700000000.+clock[0])
    path = tmp_path / "session.json"
    path.write_text(json.dumps({"zero_set_id":"test", "ramp_trial":{"force_upper_n":10.},
                                "contact_trial":getattr(request,'param',{})}))
    e = engine(1); e.reference_baseline = np.zeros(3)
    class Node:
        lock = threading.Lock()
        teleop = ("FAST", 1000.)
    window = gui.ComparisonWindow(e, path, Node()); window.timer.stop()
    def begin(kind):
        window.kind.setCurrentIndex(window.kind.findData(kind))
        window.active = dict(kind=kind, number=1, monotonic=1000., start_s=1700001000.,
                             directory=str(tmp_path), guide_events=[], guide_complete=False)
        window.set_guide_stage("prepare")
    previous = [None]; sequence = [0]
    def tick(t, force=0., refresh=True, wrist_force=0.):
        times=[] if previous[0] is None else list(np.arange(previous[0]+1/262.5,t-1e-8,1/262.5))
        for point in times+[t]:
            clock[0] = 1000.+float(point); stamp = gui.time.time(); sequence[0]+=1
            e.step(stamp, *samples(stamp, sequence[0], force=wrist_force, reference_force=0. if force is None else force))
            if force is None: e.frames[-1]["reference_force_n"] = None
        previous[0]=t
        if window.node: window.node.teleop = ("FAST", gui.time.monotonic())
        if refresh: window.refresh()
    yield window, begin, tick
    window.active = None; window.close()


@pytest.mark.parametrize('contact_guide',[{'weak_guidance':{'force_min_n':2.5,'force_max_n':3.5,'average_window_s':.1}}],indirect=True)
def test_boundary_guide_uses_average_band_without_resetting_on_raw_spike(contact_guide):
    window,begin,tick=contact_guide
    begin('weak');tick(0);tick(2);tick(5)
    tick(5.12,3.2)
    assert window.guide_stage=='hold' and '외부 약 3 N' in window.kind.currentText()
    assert '100 ms 평균' in window.guide_force_value.text() and '3.20' in window.guide_force_value.text()
    assert window.force_value.text()=='안내 힘 3.20 N'
    for t in np.arange(5.124,7.12,.004):tick(float(t),4.2 if 7.06<t<7.07 else 3.2,refresh=False)
    tick(7.12,3.2)
    assert window.guide_stage=='release'
    assert window.engine.frames[-1]['reference_force_n']==3.2
    average=window.guidance_force(window.rows(),3.2)
    assert average>3.2 and window.force_value.text()==f'안내 힘 {average:.2f} N'
    assert '원본 외부 힘 3.20 N' in window.guide_force_value.text()


@pytest.mark.parametrize('contact_guide',[{'weak_guidance':{'force_min_n':2.5,'force_max_n':3.5,'average_window_s':.1}}],indirect=True)
@pytest.mark.parametrize('upper',[10.,30.])
def test_boundary_guide_blocks_outside_average_band_and_warns_at_upper_force(contact_guide,upper):
    window,begin,tick=contact_guide
    window.ramp_upper_n=upper
    begin('weak');tick(0);tick(2);tick(5)
    tick(5.12,2.)
    assert window.guide_stage=='approach' and '조금 더 가압' in window.force_target.text()
    tick(6,3.8)
    assert window.guide_stage=='approach' and '힘을 줄이세요' in window.force_target.text()
    tick(6.01,upper)
    assert '가압 중단' in window.force_target.text() and '#ff6464' in window.force_value.styleSheet()
    assert window.force_value.text()==f'원본 외부 힘 {upper:.2f} N'


def test_normal_contact_target_is_shown_as_a_target_not_an_observer_threshold(contact_guide):
    window,begin,tick=contact_guide
    window.normal_target=5.
    begin('normal');tick(0,4.5)
    assert '약 5 N (>3 N)' in window.force_target.text()
    assert 'ON/OFF 3.0/1.2 N' in window.health.text()


@pytest.mark.parametrize("kind,force", [("weak",3.), ("normal",3.1)])
def test_contact_guide_automatically_times_external_contact_and_release(contact_guide, kind, force):
    window, begin, tick = contact_guide
    begin(kind); tick(0)
    assert "남은 2.0초" in window.countdown.format()
    tick(2); assert window.guide_stage == "pre_free"
    assert "아직 접근하지" in window.phase.text()
    assert "무접촉 유지" in window.force_target.text()
    tick(5); assert window.guide_stage == "approach"
    tick(5.05,force); assert window.guide_stage == "hold"
    tick(6.05,force); assert "남은 1.0초" in window.countdown.format()
    tick(7.05,force); assert window.guide_stage == "release"
    assert "해제" in window.force_target.text()
    tick(8.05,force); assert window.guide_stage == "release"
    tick(8.06,0); assert window.guide_stage == "release"
    tick(8.10,0); assert window.guide_stage == "post_free"
    assert "무접촉 유지" in window.force_target.text() and "더 가압" not in window.force_target.text()
    tick(11.09,0); assert not window.active["guide_complete"]
    tick(11.10,0); assert window.guide_stage == "done" and window.active["guide_complete"]
    assert "기록 종료" in window.phase.text()
    events = window.active["guide_events"]
    assert [e["stage"] for e in events] == ["prepare","pre_free","approach","hold","release","post_free","done"]
    assert events[3]["t_s"] == pytest.approx(1700001005.05)
    assert not hasattr(window,'hold_start') and not hasattr(window,'release_done')


@pytest.mark.parametrize("kind,bad,good", [("weak",3.01,2.9),("normal",3.,3.5)])
def test_contact_guide_restarts_hold_after_force_leaves_selected_band(contact_guide, kind, bad, good):
    window, begin, tick = contact_guide
    begin(kind); tick(0);tick(2);tick(5);tick(5.05,bad)
    assert window.guide_stage == "approach"
    tick(5.1,good); assert window.guide_stage == "hold"
    tick(6.1,bad); assert "목표 힘" in window.phase.text()
    tick(7.1,good); assert "남은 2.0초" in window.countdown.format()
    tick(8.1,good); assert window.guide_stage == "hold"
    tick(9.1,good); assert window.guide_stage == "release"


@pytest.mark.parametrize('upper',[10.,30.])
def test_ramp_guide_latches_brief_upper_crossing_and_waits_for_release(contact_guide,upper):
    window, begin, tick = contact_guide
    window.ramp_upper_n=upper
    begin("ramp"); tick(0,upper+1); tick(2,0);tick(5,0);tick(5.2,upper-2)
    assert window.guide_stage == "approach"
    assert "남은 힘 2.00 N" in window.force_target.text()
    tick(5.5,upper/2)
    assert window.guide_stage == "approach"
    tick(5.996,upper/2,refresh=False);tick(6,upper,refresh=False);tick(6.1,2.)
    assert window.guide_stage == "release" and "상한 도달" in window.phase.text()
    assert "해제" in window.force_target.text() and "남은 힘" not in window.force_target.text()
    assert not window.active["guide_complete"] and window.process is None
    tick(7,2.); assert window.guide_stage == "release"
    tick(7.01,0); assert window.guide_stage == "release"
    tick(7.05,0);assert window.guide_stage=='post_free'
    tick(10.05,0)
    assert window.active["guide_complete"]


def test_hold_timer_catches_force_excursion_between_screen_updates(contact_guide):
    window, begin, tick = contact_guide
    begin("weak");tick(0);tick(2);tick(5);tick(5.05,2.9)
    tick(5.8,2.9,refresh=False);tick(5.81,3.2,refresh=False);tick(5.9,2.9)
    assert "목표 힘" in window.phase.text() and window.guide_stage == "hold"
    tick(6,2.9); assert "남은 2.0초" in window.countdown.format()
    tick(8,2.9); assert window.guide_stage == "release"


def test_contact_countdown_waits_for_valid_reference_and_early_stop_is_incomplete(contact_guide):
    from PyQt5.QtCore import QProcess
    window, begin, tick = contact_guide
    begin("weak");tick(0);tick(2);tick(3,None)
    assert "외부 센서 입력" in window.phase.text() and "입력 없음" in window.force_bar.format()
    tick(7,0);tick(7.2,0)
    assert window.guide_stage == "pre_free" and "남은 3.0초" in window.countdown.format()
    tick(10.2);tick(10.25,2.9);tick(12.25,2.9);tick(12.3,0)
    assert window.guide_stage=='post_free'
    tick(15.2,0)
    window.active["stop_s"] = 1700001015.2
    tick(15.4,0)
    assert not window.active["guide_complete"]
    directory = Path(window.active["directory"]); (directory/"raw").mkdir()
    (directory/"raw/metadata.yaml").touch()
    window.recording_finished(0,QProcess.NormalExit)
    import json
    meta = json.loads((directory/"trial.json").read_text())
    assert not meta["complete"] and not meta["guide_complete"] and window.analysis is None


def test_automatic_hold_ignores_unloaded_force_and_brief_noise(contact_guide):
    window,begin,tick=contact_guide
    begin('weak');tick(0);tick(2);tick(5)
    assert window.guide_stage=='approach' and window.external_reference.state is False
    tick(5.01,2.9);assert window.guide_stage=='approach'
    tick(5.02,0);assert window.guide_stage=='approach'
    tick(5.1,2.9);assert window.guide_stage=='hold'


def test_automatic_normal_hold_waits_for_configured_target(contact_guide):
    window,begin,tick=contact_guide;window.normal_target=5.
    begin('normal');tick(0);tick(2);tick(5)
    tick(5.1,4.);assert window.guide_stage=='approach'
    tick(5.2,5.1);assert window.guide_stage=='hold'
    tick(7.2,4.5);assert window.guide_stage=='release'


def test_release_needs_fresh_low_force_hold_after_input_loss(contact_guide):
    window,begin,tick=contact_guide
    begin('normal');tick(0);tick(2);tick(5);tick(5.05,4.5);tick(7.05,4.5)
    assert window.guide_stage=='release'
    tick(8,None);assert window.guide_stage=='release' and window.external_reference.state is None
    tick(8.01,0);assert window.guide_stage=='release'
    tick(8.05,0);assert window.guide_stage=='post_free'


def test_post_free_timer_restarts_for_recontact_between_gui_updates(contact_guide):
    window,begin,tick=contact_guide
    begin('normal');tick(0);tick(2);tick(5);tick(5.05,4.5);tick(7.05,4.5);tick(7.1,0)
    assert window.guide_stage=='post_free'
    tick(8,0,refresh=False);tick(8.05,3.2,refresh=False);tick(8.1,0)
    assert window.guide_stage=='post_free' and window.external_reference.state is False
    assert '재접촉' in window.phase.text()
    tick(8.3,0);tick(11.2,0);assert not window.active['guide_complete']
    tick(11.3,0);assert window.active['guide_complete']


def test_automatic_release_and_free_timer_ignore_model_contact_outputs(contact_guide):
    window,begin,tick=contact_guide
    begin('normal');tick(0);tick(2);tick(5);tick(5.05,4.5);tick(7.05,4.5)
    tick(7.1,0,wrist_force=4.);assert window.guide_stage=='post_free'
    tick(10.1,0,wrist_force=4.)
    assert set(window.engine.frames[-1]['states'].values())=={'CONTACT'}
    assert window.active['guide_complete']


def test_reference_clock_regression_requires_new_free_evidence(contact_guide):
    window,_,_=contact_guide
    window.external_reference.seen_s=2.;window.external_reference.state=False
    assert window.update_reference_state([{'t':1.,'reference_force_n':0.}],0.)==[None]
    assert window.external_reference.state is None
    rows=[{'t':1.+i*.004,'reference_force_n':0.} for i in range(1,8)]
    window.update_reference_state(rows,0.)
    assert window.external_reference.state is False


def test_contact_collection_does_not_require_observers_to_say_free(contact_guide,monkeypatch):
    from PyQt5.QtCore import QProcess
    from PyQt5.QtWidgets import QMessageBox
    window,_,tick = contact_guide
    tick(0,0,wrist_force=4.);tick(.1,0,wrist_force=4.)
    window.kind.setCurrentIndex(window.kind.findData('weak'))
    assert all(s=='CONTACT' for s in window.engine.frames[-1]['states'].values())
    assert window.external_reference.state is False
    warnings=[]
    monkeypatch.setattr(QMessageBox,'warning',lambda *args:warnings.append(args[2]))
    monkeypatch.setattr(QProcess,'start',lambda *args:None)
    window.start_trial()
    assert not warnings and window.active is not None
    window.recording_finished(-1,QProcess.CrashExit)


def test_ramp_upper_crossing_survives_reference_outage(contact_guide):
    window,begin,tick=contact_guide;window.ramp_upper_n=30.
    begin('ramp');tick(0);tick(2);tick(5);tick(5.996,15.)
    tick(6,30.,refresh=False);tick(6.1,None)
    assert window.guide_stage=='release' and window.external_reference.state is None
    tick(6.11,0);assert window.guide_stage=='release'
    tick(6.15,0);assert window.guide_stage=='post_free'


def test_initial_free_countdown_waits_until_external_sensor_unloaded(contact_guide):
    window,begin,tick=contact_guide
    begin('weak');tick(0);tick(2);tick(3,3.2)
    assert window.guide_stage=='pre_free'
    tick(3.1,0);tick(3.3,0);tick(6.2,0)
    assert window.guide_stage=='pre_free'
    tick(6.3,0);assert window.guide_stage=='approach'


def test_live_ramp_guidance_does_not_gate_by_external_or_model_contact(contact_guide):
    window,begin,tick=contact_guide
    window.live_ramp=True;window.ramp_total=20;window.ramp_upper_n=30.;window.unloaded_bound=.4
    begin('ramp');tick(0,0,wrist_force=4.);tick(2,0,wrist_force=4.);tick(5,0,wrist_force=4.)
    assert window.guide_stage=='approach'
    assert all(v=='CONTACT' for v in window.engine.frames[-1]['states'].values())
    assert '외부 하중 기준:' not in window.reference_status.text()
    tick(5.5,.8,wrist_force=4.)
    assert window.guide_stage=='approach'
    tick(6.,30.,wrist_force=4.);assert window.guide_stage=='release'
    tick(6.2,0,wrist_force=4.);tick(6.5,0,wrist_force=4.)
    assert window.guide_stage=='post_free'
    tick(9.6,0,wrist_force=4.)
    assert window.guide_stage=='done' and window.active['guide_complete']
    assert window.external_reference.state is None


def test_live_ramp_latches_upper_crossing_and_waits_on_sensor_loss(contact_guide):
    window,begin,tick=contact_guide
    window.live_ramp=True;window.ramp_upper_n=30.;window.unloaded_bound=.4
    begin('ramp');tick(0);tick(2);tick(5)
    tick(5.9,31.,refresh=False);tick(6.,None)
    assert window.guide_stage=='release'
    tick(7.,None);assert window.guide_stage=='release'
    tick(7.2,0);tick(7.5,0);assert window.guide_stage=='post_free'
    tick(8.,3.,refresh=False);tick(8.1,0)
    assert window.guide_stage=='post_free' and not window.active['guide_complete']
    tick(11.3,0);assert window.guide_stage=='done'


def test_free_countdown_keeps_existing_motion_boundaries_and_auto_stop(contact_guide, monkeypatch):
    window, begin, tick = contact_guide
    begin("free")
    calls = []; monkeypatch.setattr(window, "stop_trial", lambda: calls.append("stop"))
    for t, cue, remaining in [(0,"준비",2),(2,"정지",10),(12,"저속 왕복",20),(32,"정상 속도 왕복",20),(52,"정지",10)]:
        tick(t)
        assert cue in window.phase.text() and f"남은 {remaining:.1f}초" in window.countdown.format()
    tick(62)
    assert calls == ["stop"] and "남은 0.0초" in window.countdown.format()


def test_recording_snapshot_keeps_noise_and_failed_start_is_not_complete(tmp_path, monkeypatch):
    import json, os, threading, time
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QProcess
    from ft_fb_leaderarm.comparison_gui import ComparisonWindow
    app=QApplication.instance() or QApplication([])
    directory=tmp_path/"session_test";directory.mkdir()
    p=directory/"session.json";p.write_text(json.dumps({"zero_set_id":"test"}))
    prior=directory/"trials"/"failed_analysis";prior.mkdir(parents=True)
    (prior/"trial.json").write_text(json.dumps({"kind":"free","owner":"shared","zero_set_id":"test",
                                               "complete":True,"analysis_complete":False}))
    e=engine(1);e.noise["zero_set_id"]="test";e.noise["static_zero_gate_pass"]=True
    now=time.time()
    e.step(now,*samples(now-.001,1))
    class Node:
        lock=threading.Lock()
        teleop=("FAST",time.monotonic())
    # No actual process or ROS recording is started by this regression check.
    monkeypatch.setattr(QProcess,"start",lambda *args:None)
    window=ComparisonWindow(e,p,Node())
    now=time.time(); e.step(now,*samples(now,2))
    window.start_trial()
    assert window.active["number"]==1
    trial=Path(window.active["directory"])
    snapshot=json.loads((trial/"session.json").read_text())
    baseline=trial/snapshot["post_zero_static_diagnostic"]["report"]
    assert baseline.is_file() and json.loads(baseline.read_text())["zero_set_id"]=="test"
    window.recording_error(QProcess.FailedToStart)
    meta=json.loads((trial/"trial.json").read_text())
    assert not meta["complete"] and REFERENCE_TOPIC not in meta["raw_topics"]
    assert window.active is None
    window.close()


def test_offline_analysis_with_no_common_valid_predictions_is_not_complete(tmp_path, monkeypatch):
    import json
    from ft_fb_leaderarm import comparison
    meta={"kind":"free","owner":"shared","zero_set_id":"test","score_start_s":1.,"score_end_s":2.,"complete":True}
    (tmp_path/"trial.json").write_text(json.dumps(meta))
    e=engine(4)
    row=e.step(1.1,*samples(1.1,1))
    monkeypatch.setattr(comparison,"load_comparison",lambda p:({"zero_set_id":"test"},e,None))
    monkeypatch.setattr(comparison,"replay_frames",lambda *args:[row])
    with pytest.raises(ValueError,match="no shared valid"):
        comparison.analyze_trial(tmp_path,tmp_path/"session.json")
    assert not json.loads((tmp_path/"trial.json").read_text()).get("analysis_complete",False)
    assert not (tmp_path/"comparison_report.json").exists()


def test_missing_noise_blocks_experiment_but_allows_noise_recording(tmp_path, monkeypatch):
    import json, os, threading, time
    os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from PyQt5.QtCore import QProcess
    from ft_fb_leaderarm.comparison_gui import ComparisonWindow
    app=QApplication.instance() or QApplication([])
    directory=tmp_path/"session_test";directory.mkdir()
    path=directory/"session.json";path.write_text(json.dumps({"zero_set_id":"test"}))
    e=engine(1);e.noise=None;e.reference_baseline=None
    now=time.time();s,w,r=samples(now-.001,1)
    s['q']=np.deg2rad([5.5,52,112,28,-107,-35]);s['dq']=np.zeros(6)
    e.step(now,s,w,r)
    class Node:
        lock=threading.Lock()
        teleop=("FAST",time.monotonic())
        latest={ROBOT_TOPIC:s}
    warnings=[]
    monkeypatch.setattr(QMessageBox,"warning",lambda *args:warnings.append(args[2]))
    monkeypatch.setattr(QProcess,"start",lambda *args:None)
    window=ComparisonWindow(e,path,Node())
    for sample in (s,w,r): sample.update(stamp=time.time(),received=time.time())
    s['sequence']=2; e.step(time.time(),s,w,r)
    window.start_trial()
    assert window.active is None and len(warnings)==1 and "정지 센서 노이즈" in warnings[0]
    window.kind.setCurrentIndex(4);window.start_trial()
    assert window.active is not None and window.active['kind']=='noise'
    window.recording_finished(-1,QProcess.CrashExit)
    assert window.active is None
    window.close()


@pytest.mark.parametrize("prior_total,prior_current,allowed",[(15,0,True),(19,4,True),(20,0,False),(20,5,False),(15,5,False)])
def test_twenty_trial_plan_keeps_five_per_zero_limit(tmp_path, monkeypatch, prior_total, prior_current, allowed):
    import json, os, threading, time
    os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from PyQt5.QtCore import QProcess
    from ft_fb_leaderarm.comparison_gui import ComparisonWindow
    app=QApplication.instance() or QApplication([])
    directory=tmp_path/"session_current";directory.mkdir()
    path=directory/"session.json"
    path.write_text(json.dumps({"zero_set_id":"current","planned_independent_zero_sessions":4,
                               "free_trial":{"repeats_planned":20,"trials_per_zero_session":5}}))
    for i in range(prior_total):
        old=tmp_path/"session_previous"/"trials"/str(i);old.mkdir(parents=True)
        (old/"trial.json").write_text(json.dumps({"kind":"free","owner":"shared","complete":True,
            "analysis_complete":True,"zero_set_id":"current" if i<prior_current else f"other_{i//5}"}))
    e=engine(1);now=time.time();e.step(now,*samples(now-.001,1))
    class Node:
        lock=threading.Lock()
        teleop=("FAST",time.monotonic())
    messages=[]
    monkeypatch.setattr(QMessageBox,"information",lambda *args:messages.append(args[2]))
    monkeypatch.setattr(QMessageBox,"warning",lambda *args:pytest.fail(args[2]))
    monkeypatch.setattr(QProcess,"start",lambda *args:None)
    window=ComparisonWindow(e,path,Node())
    assert "20회" in window.kind.currentText()
    now=time.time(); e.step(now,*samples(now,2))
    window.start_trial()
    assert (window.active is not None)==allowed
    if allowed:
        assert window.active['number']==prior_total+1
        window.recording_finished(-1,QProcess.CrashExit)
    else:
        assert len(messages)==1 and "20회" in messages[0]
        assert ("이미 기록" in messages[0])==(prior_total>=20)
    window.close()


@pytest.mark.parametrize("kind,prior,allowed",[("weak",9,True),("weak",10,False),("normal",9,True),("normal",10,False)])
def test_shared_contacts_have_one_ten_trial_count_for_all_models(tmp_path, monkeypatch, kind, prior, allowed):
    import json, os, threading, time
    os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from PyQt5.QtCore import QProcess
    from ft_fb_leaderarm.comparison_gui import ComparisonWindow
    app=QApplication.instance() or QApplication([])
    directory=tmp_path/"session_current";directory.mkdir()
    path=directory/"session.json"
    path.write_text(json.dumps({"zero_set_id":"current","contact_trial":{"weak_shared_trials":10,"normal_shared_trials":10},
                               "trial_selection":{"excluded_from_planned_counts":[{"trial":"session_previous/trials/mixed"}]}}))
    other="normal" if kind=="weak" else "weak"
    old_rows=[(kind,True,True)]*prior+[(other,True,True)]*10+[(kind,False,False),(kind,True,False)]
    for i,(old_kind,complete,analyzed) in enumerate(old_rows):
        p=tmp_path/"session_previous"/"trials"/str(i);p.mkdir(parents=True)
        (p/"trial.json").write_text(json.dumps({"kind":old_kind,"owner":["shared","a","b","c"][i%4],
            "zero_set_id":"previous","complete":complete,"analysis_complete":analyzed}))
    for name in ('mixed','bad_protocol'):
        p=tmp_path/'session_previous'/'trials'/name;p.mkdir()
        (p/'trial.json').write_text(json.dumps({'kind':kind,'zero_set_id':'previous','complete':True,'analysis_complete':True}))
        if name=='bad_protocol':
            (p/'comparison_report.json').write_text(json.dumps({'protocol_quality':{'plan_eligible':False}}))
    e=engine(1);now=time.time();e.step(now,*samples(now-.001,1))
    class Node:
        lock=threading.Lock()
        teleop=("FAST",time.monotonic())
    messages=[]
    monkeypatch.setattr(QMessageBox,"information",lambda *args:messages.append(args[2]))
    monkeypatch.setattr(QProcess,"start",lambda *args:None)
    window=ComparisonWindow(e,path,Node());window.kind.setCurrentIndex(window.kind.findData(kind))
    now=time.time(); e.step(now,*samples(now,2))
    window.refresh()
    assert window.owner.count()==1 and window.owner.currentData()=="shared" and not window.owner.isEnabled()
    assert "공통 10회" in window.kind.currentText()
    window.start_trial()
    assert (window.active is not None)==allowed
    if allowed:
        assert window.active['owner']=='shared' and window.active['number']==10
        assert set(e.frames[-1]['predictions'])==set(e.names)
        trial=Path(window.active['directory']);window.recording_finished(-1,QProcess.CrashExit)
        assert set(json.loads((trial/'trial.json').read_text())['raw_topics'])=={ROBOT_TOPIC,WRIST_TOPIC,REFERENCE_TOPIC}
    else:
        assert len(messages)==1 and "10회" in messages[0]
    window.close()


@pytest.mark.parametrize('prior,allowed',[(19,True),(20,False)])
def test_live_ramp_twenty_trial_limit_and_log_lifecycle(tmp_path,monkeypatch,prior,allowed):
    import json,os,threading,time
    from PyQt5.QtWidgets import QApplication,QMessageBox
    from PyQt5.QtCore import QProcess
    from ft_fb_leaderarm.comparison_gui import ComparisonWindow
    from ft_fb_leaderarm.comparison_trace import LIVE_RAMP_PROTOCOL
    os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
    app=QApplication.instance() or QApplication([])
    folder=tmp_path/'session_current';folder.mkdir();path=folder/'session.json'
    path.write_text(json.dumps({'zero_set_id':'test','enabled_experiments':['ramp','noise'],
        'ramp_trial':{'mode':LIVE_RAMP_PROTOCOL,'physical_shared_trials':20,'force_upper_n':30.}}))
    for i in range(prior+1):
        old=tmp_path/'session_previous'/'trials'/str(i);old.mkdir(parents=True)
        (old/'trial.json').write_text(json.dumps({'kind':'ramp','zero_set_id':'test','complete':True,'analysis_complete':True}))
        (old/'comparison_report.json').write_text(json.dumps({'analysis_schema':LIVE_RAMP_PROTOCOL if i<prior else 'legacy',
            'protocol_quality':{'plan_eligible':True}}))
    e=engine(1);calls=[]
    class Node:
        lock=threading.Lock();teleop=('FAST',time.monotonic());trace=None
        def start_trace(self,directory):calls.append(('start',str(directory)))
        def trace_phase(self,phase):calls.append(('phase',phase))
        def finish_trace(self):calls.append(('finish',None));return {'complete':False}
    notices=[]
    monkeypatch.setattr(QMessageBox,'warning',lambda *a:pytest.fail(a[2]))
    monkeypatch.setattr(QMessageBox,'information',lambda *a:notices.append(a[2]))
    monkeypatch.setattr(QProcess,'start',lambda *a:None)
    window=ComparisonWindow(e,path,Node());window.timer.stop()
    now=time.time();e.step(now,*samples(now,1))
    window.refresh()
    assert window.kind.count()==2 and window.kind.currentData()=='ramp'
    assert '20회' in window.kind.currentText() and '외부 하중 기준:' not in window.reference_status.text()
    window.start_trial()
    assert (window.active is not None)==allowed
    if allowed:
        assert window.active['number']==20 and window.active['guide_mode']==LIVE_RAMP_PROTOCOL
        saved=Path(window.active['directory'])
        assert json.loads((saved/'session.json').read_text())['ramp_trial']['physical_shared_trials']==20
        window.recording_finished(-1,QProcess.CrashExit)
        assert not json.loads((saved/'trial.json').read_text())['live_trace_complete']
        assert any(c[0]=='finish' for c in calls)
    else:assert '20회' in notices[0] and not calls
    window.close()


def test_ten_shared_ramps_aggregate_missing_without_zero_force(tmp_path):
    import json
    from ft_fb_leaderarm.comparison import summarize_ramps
    for i in range(10):
        p=tmp_path/"session_test"/"trials"/str(i);p.mkdir(parents=True)
        (p/"trial.json").write_text(json.dumps({"complete":True}))
        onset={"t_s":1.+i,"external_force_n":2.+.1*i,"reference_valid":True}
        (p/"comparison_report.json").write_text(json.dumps({"kind":"ramp","ramp_onsets":{"a":onset,"b":None}}))
    invalid=tmp_path/'session_test'/'trials'/'mixed';invalid.mkdir()
    (invalid/'trial.json').write_text(json.dumps({'complete':True}))
    (invalid/'comparison_report.json').write_text(json.dumps({'kind':'ramp','protocol_quality':{'plan_eligible':False},
        'ramp_onsets':{'a':{'t_s':1.,'external_force_n':.1,'reference_valid':True},'b':None}}))
    r=summarize_ramps(tmp_path)
    assert r["physical_trials"]==10
    assert r["models"]["a"]["force_median_n"]==pytest.approx(2.45)
    assert r["models"]["b"]["missing_or_unpaired"]==10
    assert r["models"]["b"]["force_min_n"] is None
    assert r['excluded_protocol_trials']==['session_test/trials/mixed']
    assert (tmp_path/"ramp_onsets.csv").is_file()


def test_comparison_has_no_control_publishers_or_robot_launch():
    import ast
    root=Path(__file__).resolve().parents[1]
    for name in ["comparison.py","comparison_gui.py"]:
        tree=ast.parse((root/"ft_fb_leaderarm"/name).read_text())
        forbidden=[n for n in ast.walk(tree) if isinstance(n,ast.Attribute) and n.attr in ("create_publisher","create_client","create_service")]
        assert not forbidden
    launch=(root/"launch/three_model_comparison_gui.launch.py").read_text()
    assert "IncludeLaunchDescription" not in launch and "teleop.launch" not in launch


def test_gui_launch_prefers_torch_dependencies_over_system_qt(monkeypatch):
    import importlib.util
    path=Path(__file__).resolve().parents[1]/"launch/three_model_comparison_gui.launch.py"
    spec=importlib.util.spec_from_file_location("comparison_launch_test",path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setenv("PYTHONPATH","/usr/lib/python3/dist-packages")
    paths=module._pythonpath().split(":")
    assert paths.index("/home/vision/venv_act/lib/python3.10/site-packages") < paths.index("/usr/lib/python3/dist-packages")


def test_gui_shutdown_waits_for_recording_and_analysis(tmp_path, monkeypatch):
    import json, os
    os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
    from PyQt5.QtWidgets import QApplication
    from ft_fb_leaderarm.comparison_gui import ComparisonWindow
    app=QApplication.instance() or QApplication([])
    path=tmp_path/"session.json";path.write_text(json.dumps({"zero_set_id":"test"}))
    window=ComparisonWindow(engine(1),path)
    calls=[]
    monkeypatch.setattr(window,"stop_trial",lambda:calls.append("stop"))
    monkeypatch.setattr(window,"close",lambda:calls.append("close"))
    window.active={"kind":"free"}
    window.request_exit()
    assert calls==["stop"] and window.exit_requested
    window.active=None;window.analysis=object();window.refresh()
    assert "close" not in calls
    window.analysis=None;window.refresh()
    assert calls[-1]=="close"
    window.timer.stop()


def test_ros_context_shutdown_is_an_expected_monitor_exit(monkeypatch):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from ft_fb_leaderarm.comparison_gui import spin_monitor
    def shutdown(node): raise ExternalShutdownException()
    monkeypatch.setattr(rclpy,"spin",shutdown)
    spin_monitor(object())
