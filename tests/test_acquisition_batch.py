import importlib.util
from pathlib import Path
import pytest


def selector():
    path=Path(__file__).resolve().parents[1]/'tools/acquire_room_3d.py'
    spec=importlib.util.spec_from_file_location('acquisition_batch_fixture',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.select_pending


def partitioner():
    path=Path(__file__).resolve().parents[1]/'tools/acquire_room_3d.py'
    spec=importlib.util.spec_from_file_location('acquisition_partition_fixture',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.partition_pending


def test_newest_frame_and_spaced_bridge_views_survive_backpressure_without_a_fifo_backlog():
    select=selector();pending=list(range(11))
    assert select(pending,1)==[10]
    assert select(pending,2)==[0,10]
    assert select(pending,4)==[0,3,6,10]
    for size in range(1,97):
        pending=list(range(size));chosen=select(pending,4)
        assert chosen[-1]==size-1 and len(chosen)<=4
        assert chosen==sorted(set(chosen))
        if size<=4:assert chosen==pending


@pytest.mark.parametrize('batch_size',[2,4])
@pytest.mark.parametrize('backlog',[4,8])
def test_bridge_reserve_bounds_work_keeps_chronology_and_drains_the_last_photo(batch_size,backlog):
    partition=partitioner();selected_all=[];discarded_all=[];pending=[];seen=set()
    # A burst exceeds the reserve; later batches include new arrivals while
    # previously retained bridge photos still wait. No older photo may be
    # admitted after a newer one, and the final retained view must be analyzed.
    for arrivals in [list(range(17)),list(range(17,23)),list(range(23,51)),[],[],[],[],[],[]]:
        pending+=arrivals
        selected,deferred,discarded=partition(pending,batch_size,backlog)
        assert len(selected)<=batch_size and len(deferred)<=backlog
        assert sorted(selected+deferred+discarded)==pending
        assert not set(selected)&set(deferred) and not set(discarded)&set(deferred)
        if pending:assert pending[-1] in selected+deferred
        assert not seen&set(selected+discarded)
        seen.update(selected+discarded);selected_all+=selected;discarded_all+=discarded
        pending=deferred
    assert not pending and selected_all==sorted(set(selected_all))
    assert set(selected_all+discarded_all)==set(range(51))
    assert selected_all[-1]==50


def test_disabled_bridge_reserve_preserves_the_original_selection():
    partition=partitioner();select=selector();pending=list(range(18))
    for batch_size in (1,2,4):
        selected,deferred,discarded=partition(pending,batch_size)
        assert selected==select(pending,batch_size) and not deferred
        assert sorted(selected+discarded)==pending
    for backlog in (True,-1,1,16):
        with pytest.raises(ValueError):partition(pending,4,backlog)
    with pytest.raises(ValueError):partition(pending,1,4)


@pytest.mark.parametrize(('option','error'),[
    (['--depth-stereo-calibration'],'--depth-stereo-calibration requires --estimated-depth-model'),
    (['--native-direct-verification'],'--native-direct-verification requires a native matching backend'),
    (['--historical-query-budget','16'],'--historical-query-budget requires a native matching backend')])
def test_experimental_depth_and_matching_flags_require_their_backend_before_any_io(tmp_path,option,error):
    import subprocess,sys
    script=Path(__file__).resolve().parents[1]/'tools/acquire_room_3d.py'
    output=tmp_path/'never-created'
    result=subprocess.run([sys.executable,str(script),'--replay',str(tmp_path/'missing-source'),
        '--camera',str(tmp_path/'missing-camera'),'--output',str(output),*option],capture_output=True,text=True)
    assert result.returncode==2 and error in result.stderr and not output.exists()
