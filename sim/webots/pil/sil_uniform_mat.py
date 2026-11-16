"""Conservative uniform-MAT certificate over ORIGINAL immutable scene faces.

No mixed-material integration, scene-cell compilation, or unknown-as-white.
One opaque axis-aligned rectangular MAT_WHITE face must contain the entire rotated4sigma box.
Every relevant competing face is checked; touching a material boundary rejects.
"""
import math
from .scene import EPS_MM
from .s3_optics import OpticalSample


class UniformMatCertificate:
    def __init__(self,scene):
        self.scene=scene
        self.faces=tuple((face,tuple(min(p[i]for p in face.vertices_mm)for i in range(3)),
            tuple(max(p[i]for p in face.vertices_mm)for i in range(3)))for face in scene.surfaces)
    def sample(self,scene,profile,origin,axis_u,*,direction=(0,0,-1)):
        if scene is not self.scene:return None
        if direction!=(0,0,-1)or abs(axis_u[2])>1e-12:return None
        if abs(sum(v*v for v in axis_u)-1)>1e-10:return None
        if profile.noise_sigma_adc!=0:return None
        level=profile.levels_adc.get('MAT_WHITE')
        if level is None or not math.isfinite(level)or not 0<=level<=4095:return None
        x,y,z=origin;floor=z-profile.reference_height_mm
        # The approved flat-world reference is10mm; no height/tilt extrapolation.
        if abs(floor-10)>1e-9:return None
        ux,uy,_=axis_u;vx,vy=-uy,ux
        bx=4*(profile.sigma_x_mm*abs(ux)+profile.sigma_y_mm*abs(vx))
        by=4*(profile.sigma_x_mm*abs(uy)+profile.sigma_y_mm*abs(vy))
        box=(x-bx,y-by,x+bx,y+by);epsilon=EPS_MM
        corners=((box[0],box[1]),(box[2],box[1]),(box[2],box[3]),(box[0],box[3]))
        covered=False
        for face,lo,hi in self.faces:
            if hi[2]<floor-epsilon:continue # fully hidden behind the proven opaque top
            vertices=face.vertices_mm
            # Authoritative containment uses EPS_MM on edge cross products.
            # Restrict the shortcut to axis-aligned rectangles; their finite
            # edge tolerance is bounded by EPS/minimum_edge_length.
            def distinct_axis(i):
                clusters=[]
                for value in sorted(p[i]for p in vertices):
                    if not clusters or value-clusters[-1]>epsilon:clusters.append(value)
                return len(clusters)
            shape=sorted(distinct_axis(i)for i in range(3))
            if len(vertices)!=4 or shape!=[1,2,2]:return None
            edge=min(math.dist(a,b)for a,b in zip(vertices,vertices[1:]+vertices[:1]))
            pad=epsilon+epsilon/edge
            if hi[0]<box[0]-pad or lo[0]>box[2]+pad or hi[1]<box[1]-pad or lo[1]>box[3]+pad:continue
            a,b,c=vertices[:3]
            normal_z=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
            coplanar=all(p[2]==floor for p in vertices) # strict actual datum, NOT EPS-equivalent planes
            if face.material_id!='MAT_WHITE'or not coplanar or normal_z<=0:return None
            # Convexity/CCW are enforced by PolygonSurface construction.
            if all(all((b[0]-a[0])*(py-a[1])-(b[1]-a[1])*(px-a[0])>epsilon
                for a,b in zip(vertices,vertices[1:]+vertices[:1]))for px,py in corners):covered=True
        if not covered:return None
        return OpticalSample(round(level),True,'',level,profile.reference_height_mm,{'MAT_WHITE':1.},
            tuple(profile.assumptions)+('ORIGINAL_FACE_FULL_4SIGMA_BOX_UNIFORM_MAT_CERTIFICATE',),False,0)
