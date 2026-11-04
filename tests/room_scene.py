"""Deterministic, known-camera room images for multi-direction map tests.

Wall textures and the ChArUco reference are projected from independent 3D
geometry. No landmark coordinates or query poses are injected into the map.
This scene does not model aircraft control, sensor errors or physical safety.
"""
import cv2 as cv
import numpy as np

from test_metric_reference import BOARD,MODEL


class RoomScene:
    def __init__(self,folder,*,texture_cells=64,texture_limits=(20,236)):
        self.folder=folder
        self.initial_R=cv.Rodrigues(np.array([.3,-.2,.06]))[0]
        self.center=-self.initial_R.T@np.array([-.13,-.07,.65])
        self.textures=[]
        for i in range(8):
            texture=np.random.default_rng(281+i).integers(*texture_limits,
                size=(texture_cells,texture_cells),dtype=np.uint8)
            self.textures.append(cv.GaussianBlur(cv.resize(texture,(512,512)),(3,3),.6))
        self.board=BOARD.opencv_board(.035).generateImage((980,700))

    def orientation(self,degrees):
        return self.initial_R@cv.Rodrigues(np.array([0.,-np.radians(degrees),0.]))[0]

    def render(self,path,degrees,center,*,panels=()):
        R=self.orientation(degrees);rv=cv.Rodrigues(R)[0];t=-R@center
        canvas=np.full((MODEL.height,MODEL.width),160,np.uint8)

        def place(texture,vertices):
            vertices=np.asarray(vertices,dtype=float)
            if ((vertices@R.T+t)[:,2]<=.05).any():return
            target=cv.projectPoints(vertices,rv,t,MODEL.matrix(),None)[0].reshape(-1,2).astype(np.float32)
            h,w=texture.shape
            H=cv.getPerspectiveTransform(np.float32([[0,0],[w,0],[w,h],[0,h]]),target)
            warped=cv.warpPerspective(texture,H,(MODEL.width,MODEL.height),borderValue=160)
            mask=cv.warpPerspective(np.full_like(texture,255),H,(MODEL.width,MODEL.height))
            canvas[mask>254]=warped[mask>254]

        radius=2.5;half=radius*np.tan(np.pi/8)
        for i in range(8):
            angle=i*np.pi/4
            direction=np.array([np.sin(angle),0.,np.cos(angle)])
            tangent=np.array([np.cos(angle),0.,-np.sin(angle)])
            c=self.center+radius*direction
            place(self.textures[i],[c-half*tangent+[0,-1.2,0],c+half*tangent+[0,-1.2,0],
                c+half*tangent+[0,1.2,0],c-half*tangent+[0,1.2,0]])
        if center[2]<0:
            place(self.board,[[0.,0.,0.],[.245,0.,0.],[.245,.175,0.],[0.,.175,0.]])
        for vertices,texture in panels:place(texture,vertices)
        if not cv.imwrite(str(path),canvas):raise RuntimeError('room_fixture_write_failed')
        return R

    def pair(self,degrees):
        tangent=self.orientation(degrees).T@np.array([1.,0.,0.])
        paths=[]
        for i,shift in enumerate([-.06,.06]):
            path=self.folder/f'view-{degrees:03d}-{i}.png'
            self.render(path,degrees,self.center+shift*tangent);paths.append(path)
        return paths
