"""Native camera/frame ownership for features computed from retained pixels."""
import pytest
np=pytest.importorskip('numpy')
cv=pytest.importorskip('cv2')
p=pytest.importorskip('pycolmap')
from neo2_control.progressive_room import _extract_features
from neo2_control.reconstruction import CameraModel


def test_feature_batches_keep_one_camera_and_rig_with_distinct_native_frames(tmp_path):
    model=CameraModel(1280,720,680.,680.,640.,360.,(),'candidate','synthetic fixture')
    rng=np.random.default_rng(21);gray_cache={}
    for index in range(3):
        tile=rng.integers(0,256,(180,320),dtype=np.uint8)
        gray_cache[f'{index}.png']=cv.resize(tile,(1280,720))
    dbpath=tmp_path/'database.db'
    for names in ({'0.png','1.png'},{'2.png'}):
        features=_extract_features(dbpath,tmp_path,names,model,'sift',gray_cache)
        assert set(features)==names and all(len(v)>20 for v in features.values())
    assert not gray_cache
    with p.Database.open(dbpath) as db:
        assert len(db.read_all_cameras())==len(db.read_all_rigs())==1
        images=db.read_all_images();frames=db.read_all_frames()
        assert len(images)==len(frames)==3
        camera=db.read_all_cameras()[0]
        assert camera.params==pytest.approx([680,680,640,360])
        assert len({image.camera_id for image in images})==1
        assert len({image.frame_id for image in images})==3
        assert {d.id for frame in frames for d in frame.data_ids}=={i.image_id for i in images}
        for image in images:
            xy=db.read_keypoints(image.image_id)
            assert len(xy)==len(db.read_descriptors(image.image_id).data)
            assert (xy>=0).all() and (xy[:,0]<1280).all() and (xy[:,1]<720).all()
