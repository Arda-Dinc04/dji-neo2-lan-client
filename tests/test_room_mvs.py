"""Synthetic GLB packaging checks; these fixtures are not room reconstruction."""
import base64
import json
import struct
import cv2
import numpy as np
import pytest
from neo2_control.room_mvs import pack_glb,inspect_glb


def native_glb(tmp_path,*,bad_index=False,external_name='photo.png'):
    points=np.arange(63*3,dtype='<f4').reshape(63,3)/100
    indices=np.arange(63,dtype='<u4')
    if bad_index:indices[-1]=999
    first=points.tobytes()+indices.tobytes()
    uv=np.linspace(0,1,126,dtype='<f4').reshape(63,2).tobytes()
    texture=np.indices((16,16)).sum(axis=0).astype(np.uint8)*8
    png=cv2.imencode('.png',cv2.cvtColor(texture,cv2.COLOR_GRAY2BGR))[1].tobytes()
    (tmp_path/'photo.png').write_bytes(png)
    model={'asset':{'version':'2.0'},'buffers':[{'byteLength':len(first)},
        {'byteLength':len(uv),'uri':'data:application/octet-stream;base64,'+base64.b64encode(uv).decode()}],
        'bufferViews':[{'buffer':0,'byteOffset':0,'byteLength':points.nbytes},
            {'buffer':0,'byteOffset':points.nbytes,'byteLength':indices.nbytes},
            {'buffer':1,'byteOffset':0,'byteLength':len(uv)}],
        'accessors':[{'bufferView':0,'componentType':5126,'count':63,'type':'VEC3'},
            {'bufferView':1,'componentType':5125,'count':63,'type':'SCALAR'},
            {'bufferView':2,'componentType':5126,'count':63,'type':'VEC2'}],
        'meshes':[{'primitives':[{'attributes':{'POSITION':0,'TEXCOORD_0':2},'indices':1,'material':0}]}],
        'materials':[{'pbrMetallicRoughness':{'baseColorTexture':{'index':0}}}],
        'textures':[{'source':0}],'images':[{'uri':external_name}]}
    encoded=json.dumps(model).encode();encoded+=b' '*((-len(encoded))%4)
    first+=b'\0'*((-len(first))%4)
    raw=struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(first))
    raw+=struct.pack('<I4s',len(encoded),b'JSON')+encoded
    raw+=struct.pack('<I4s',len(first),b'BIN\0')+first
    path=tmp_path/'native.glb';path.write_bytes(raw)
    return path,points,png


def test_external_photographs_and_multiple_buffers_are_embedded_without_rewriting(tmp_path):
    source,points,png=native_glb(tmp_path);target=tmp_path/'packed.glb'
    evidence=pack_glb(source,target);result=inspect_glb(target)
    assert result['triangles']==21 and result['vertices']==63
    assert result['embedded_images']==1 and result['decoded_texture_pixels']==256
    assert result['navigation_eligible'] is result['metric_scale_verified'] is False
    assert evidence['geometry_buffers_copied_byte_identically']
    assert points.tobytes() in target.read_bytes() and png in target.read_bytes()
    with pytest.raises(FileExistsError):pack_glb(source,target)


def test_invalid_face_index_is_not_counted_as_a_verified_textured_mesh(tmp_path):
    source,_,_=native_glb(tmp_path,bad_index=True);target=tmp_path/'packed.glb'
    pack_glb(source,target)
    with pytest.raises(ValueError,match='triangle_indices'):inspect_glb(target)


def test_packaging_cannot_read_outside_the_asset_folder(tmp_path):
    source,_,_=native_glb(tmp_path,external_name='../unrelated.png')
    with pytest.raises(ValueError,match='external_path'):pack_glb(source,tmp_path/'refused.glb')
    assert not (tmp_path/'refused.glb').exists()
