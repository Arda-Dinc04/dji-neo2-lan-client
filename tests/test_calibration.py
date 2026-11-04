"""Real corner detection on rendered known-camera boards, without a drone."""
import json

import pytest

np=pytest.importorskip('numpy')
cv=pytest.importorskip('cv2')

from neo2_control.calibration import calibrate_checkerboard
from neo2_control.reconstruction import read_camera_model


def rendered_boards(folder):
    cv.setNumThreads(2)
    board=np.full((420,600),255,dtype=np.uint8)
    for y in range(7):
        for x in range(10):
            if (x+y)%2==0:board[y*60:(y+1)*60,x*60:(x+1)*60]=0
    K=np.array([[800.,0,640],[0,800.,360],[0,0,1.]])
    corners=np.array([[-1.,-1.,0],[9.,-1.,0],[9.,6.,0],[-1.,6.,0]])
    source=np.float32([[0,0],[600,0],[600,420],[0,420]])
    paths=[]
    for i in range(12):
        angle=2*np.pi*i/12
        r=np.array([.22*np.sin(angle),.30*np.cos(angle),.07*np.sin(2*angle)])
        t=np.array([-4.+.9*np.sin(angle),-2.5+.5*np.cos(angle),13.+np.sin(angle)])
        projected=cv.projectPoints(corners,r,t,K,None)[0].reshape(-1,2).astype(np.float32)
        image=cv.warpPerspective(board,cv.getPerspectiveTransform(source,projected),(1280,720),
            borderValue=200,flags=cv.INTER_LINEAR)
        path=folder/f'board-{i}.png';assert cv.imwrite(str(path),image);paths.append(path)
    return paths,K


def test_checkerboard_fit_recovers_known_intrinsics_without_claiming_scale(tmp_path):
    paths,K=rendered_boards(tmp_path)
    model,report=calibrate_checkerboard(paths)
    assert model.fx==pytest.approx(K[0,0],rel=.02)
    assert model.fy==pytest.approx(K[1,1],rel=.02)
    assert model.cx==pytest.approx(K[0,2],abs=5)
    assert model.cy==pytest.approx(K[1,2],abs=5)
    assert report['accepted_views']==12 and report['rms_pixels']<.8
    assert report['maximum_board_normal_angle_degrees']>10
    assert report['metric_scale_verified'] is False
    output=tmp_path/'model.json';output.write_text(json.dumps(model.as_dict()))
    assert read_camera_model(output)==model


def test_duplicate_views_or_short_input_do_not_establish_calibration(tmp_path):
    with pytest.raises(ValueError,match='images_or_pattern'):calibrate_checkerboard([])
    paths,K=rendered_boards(tmp_path)
    with pytest.raises(ValueError,match='duplicate_image'):
        calibrate_checkerboard([paths[0]]*10)
