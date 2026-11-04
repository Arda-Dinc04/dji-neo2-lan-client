from types import SimpleNamespace as NS
import numpy as np
import pytest
from neo2_control.progressive_room import _surface_snapshot


def scene(*,baseline=.15,bad_observation=False,duplicate_track=False):
    camera=NS(model_name='PINHOLE',width=200,height=200,
        calibration_matrix=lambda:np.array([[100.,0,100],[0,100,100],[0,0,1]]))
    positions=np.array([[x,y,3.] for x in np.linspace(-.6,.6,4) for y in np.linspace(-.6,.6,4)])
    points={i:NS(xyz=X,color=np.array([100,150,200]),
        track=NS(elements=[NS(image_id=v) for v in ([1,1,3] if duplicate_track and i==0 else [1,2,3])]))
        for i,X in enumerate(positions)}
    images={};records={}
    for image_id,center in enumerate(([-baseline,0.,0.],[0.,0.,0.],[baseline,0.,0.]),1):
        t=-np.asarray(center);q=positions+t;xy=q[:,:2]/q[:,2:]*100+100
        if bad_observation and image_id==3:xy[0]+=[8,0]
        pose=NS(rotation=NS(matrix=lambda:np.eye(3)),translation=t)
        name=f'{image_id}.png';records[name]={'sha256':'a'*64}
        images[image_id]=NS(name=name,camera_id=1,cam_from_world=lambda pose=pose:pose,
            points2D=[NS(point3D_id=i,xy=pixel) for i,pixel in enumerate(xy)])
    return NS(images=images,cameras={1:camera},points3D=points,reg_image_ids=lambda:list(images)),records


def test_three_view_geometry_produces_photographic_facets_in_the_native_frame():
    rec,records=scene();result=_surface_snapshot(rec,records)
    assert len(result['points'])==16 and result['triangle_count']>0
    assert all(np.isfinite(patch['positions']).all() for patch in result['patches'])
    assert all(0<=u<=1 for patch in result['patches'] for u in patch['uv'])
    assert {p['image'] for p in result['patches']}<=set(records)
    assert result['poses'][0]['center']==pytest.approx([-.15,0,0])


@pytest.mark.parametrize('kwargs',[{'bad_observation':True},{'duplicate_track':True}])
def test_one_inconsistent_or_duplicate_track_cannot_enter_the_surface(kwargs):
    rec,records=scene(**kwargs);result=_surface_snapshot(rec,records)
    assert len(result['points'])==15
    assert [-.6,-.6,3.] not in result['points']


def test_rotation_without_a_triangulation_baseline_does_not_invent_depth():
    rec,records=scene(baseline=0);result=_surface_snapshot(rec,records)
    assert not result['points'] and not result['patches'] and result['triangle_count']==0
