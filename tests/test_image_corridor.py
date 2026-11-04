"""Rendered images -> estimated poses/depth -> persisted free/blocked corridor.

Source camera poses, depths and obstacle surfaces are inferred from images.
The planned hover position and body dimensions are declared fixture inputs.
This scene does not validate physical camera accuracy or execute a flight.
"""
import json
import subprocess
import sys

import pytest

np=pytest.importorskip('numpy')
cv=pytest.importorskip('cv2')

from neo2_control.depth_scan import depth_geometry,save_depth_scan
from neo2_control.metric_atlas import create_atlas,densify_atlas,extend_atlas
from neo2_control.metric_reference import anchor_scan
from neo2_control.reconstruction import reconstruct_pair
from neo2_control.spatial_memory import FREE,OCCUPIED,load_memory
from room_scene import RoomScene
from test_metric_reference import MODEL,detect


def run_cli(arguments):
    result=subprocess.run([sys.executable,'-m','neo2_control.cli',*map(str,arguments)],
        capture_output=True,text=True,timeout=90)
    assert result.returncode==0,result.stderr
    return json.loads(result.stdout)


@pytest.fixture(scope='module')
def image_survey(tmp_path_factory):
    cv.setNumThreads(2);cv.setRNGSeed(47)
    folder=tmp_path_factory.mktemp('image-corridor')
    room=RoomScene(folder,texture_cells=128,texture_limits=(0,256))
    first=room.pair(0)
    atlas=create_atlas(anchor_scan(reconstruct_pair(*first,MODEL),*[detect(p) for p in first]))
    for degrees in (30,60,90):atlas=extend_atlas(atlas,*room.pair(degrees))
    room.initial_R=np.eye(3)
    retained=[]

    def observe(name,offset,*,keep=False,panels=()):
        nonlocal atlas
        center=room.center+offset;tangent=room.orientation(90).T@np.array([1.,0.,0.])
        images=[]
        for j,shift in enumerate([-.06,.06]):
            path=folder/f'{name}-{j}.png'
            room.render(path,90,center+shift*tangent,panels=panels);images.append(path)
        atlas=extend_atlas(atlas,*images)
        if not keep:return None
        depth=densify_atlas(atlas,len(atlas['extensions'])-1,*images,maximum_m=6.)
        path=folder/f'{name}-depth.json';save_depth_scan(path,depth)
        return path,depth

    for i,offset in enumerate([[-1.,0.,0.],[-1.,-.25,0.],[-1.,.25,0.]]):
        result=observe(f'near-{i}',offset,keep=i<2)
        if result is not None:retained.append(result[0])
    cv.setRNGSeed(47)
    for i,offset in enumerate([[-2.,0.,0.],[-1.8,-.25,0.],[-1.8,.25,0.]]):
        result=observe(f'far-{i}',offset,keep=i==1)
        if result is not None:retained.append(result[0])

    cv.setRNGSeed(47)
    panel_center=room.center+[.55,0.,0.]
    vertices=[panel_center+[0,-.3,-.3],panel_center+[0,-.3,.3],
        panel_center+[0,.3,.3],panel_center+[0,.3,-.3]]
    texture=cv.GaussianBlur(cv.resize(np.random.default_rng(738).integers(0,256,
        (128,128),dtype=np.uint8),(256,256)),(3,3),.6)
    obstacle,depth=observe('obstacle',[-1.8,-.25,0.],keep=True,panels=[(vertices,texture)])
    return room,retained,obstacle,depth,folder


def assess_saved(room,depths,folder,name):
    memory=folder/f'{name}-memory.json';route=folder/f'{name}-route.json'
    run_cli(['memory-build',*depths,'--lower',*(room.center+[-.4,-.4,-.4]),
        '--upper',*(room.center+[1.4,.4,.4]),'--resolution',.05,'--surface-margin',.05,
        '--output',memory])
    run_cli(['memory-plan',memory,'--position',*room.center,'--forward',0,0,1,'--up',0,-1,0,
        '--radius',.15,'--tracking-margin',.05,'--distance',1.,'--angle-step',10,'--output',route])
    return load_memory(memory),json.loads(route.read_text())


def test_rendered_depth_observes_complete_swept_corridor_and_cold_cli_plans_round_trip(image_survey):
    room,depths,_,_,folder=image_survey
    memory,route=assess_saved(room,depths,folder,'clear')
    selected=route['candidate']
    assert selected and selected['model_candidate'] and not selected['outside_memory']
    assert selected['swept_cell_count']>1000
    assert selected['unknown_cell_count']==selected['surface_cell_count']==0
    steps=route['steps']
    assert [s['kind'] for s in steps]==['takeoff','rotate','translate_measured','rotate',
        'translate_measured','rotate','land']
    assert steps[2]['distance_m']==steps[4]['distance_m']==1.
    assert np.linalg.norm(np.array(steps[2]['target_position_m'])-room.center)==pytest.approx(1.)
    assert steps[4]['target_position_m']==room.center.tolist()
    assert steps[3]['degrees']==steps[5]['degrees']==180
    assert memory['coordinate_frame']==route['coordinate_frame']
    assert memory['free_space_verified'] is False
    assert route['physical_clearance_verified'] is False and route['flight_authorized'] is False


def test_obstacle_in_new_images_overrides_previously_clear_cells_and_refuses_the_route(image_survey):
    room,depths,obstacle,depth,folder=image_survey
    memory,route=assess_saved(room,[*depths[:2],obstacle],folder,'blocked')
    assert memory['counts']['surface_or_margin']>0
    rightward=next(c for c in route['headings'] if c['yaw_degrees']==90)
    assert rightward['surface_cell_count']>0 and not rightward['model_candidate']
    assert route['candidate'] is None and route['steps']==[]
    clear_path=folder/'clear-memory.json'
    if not clear_path.exists():assess_saved(room,depths,folder,'clear')
    before,after=np.array(load_memory(clear_path)['states']),np.array(memory['states'])
    assert ((before==FREE)&(after==OCCUPIED)).any()
    points=np.array(depth_geometry(depth)['points'])
    nearby=points[(abs(points[:,1]-room.center[1])<.25)&(abs(points[:,2]-room.center[2])<.25)
        &(points[:,0]<room.center[0]+.8)]
    assert len(nearby)>40
    assert np.median(nearby[:,0])==pytest.approx(room.center[0]+.55,abs=.06)
    assert route['flight_authorized'] is False
