"""Explicit authoritative sampler selection; no automatic fallback."""
from .s3_optics import sample_sensor
from .s3_fast import FastScene


class FullSceneSampler:
    def __init__(self,built):
        self.scene=built.scene;self.gp28_calls={'uniform':0,'fine':0};self.uniform_certificate=None
    def sample(self,profile,origin_mm,**kwargs):
        return sample_sensor(self.scene,profile,origin_mm,**kwargs)


def sample_gp28(sampler,profile,position,yaw,mount,*,tolerance=1e-7,uniform_mat=False):
    """Explicit approved GP28 boundary-aware full-scene route, never fallback."""
    import math
    from .s3_referee import fine_sample
    from .s3_gp28_nonlinear import board_axis_u
    if not isinstance(sampler,FullSceneSampler):raise ValueError('GP28 needs full scene')
    c,s=math.cos(yaw),math.sin(yaw);f,l=mount['forward_mm'],mount['left_mm']
    origin=(position[0]*1000+c*f-s*l,position[1]*1000+s*f+c*l,10+mount['flat_clearance_mm'])
    axis=board_axis_u(yaw)
    if uniform_mat:
        from .sil_uniform_mat import UniformMatCertificate
        if sampler.uniform_certificate is None or sampler.uniform_certificate.scene is not sampler.scene:
            sampler.uniform_certificate=UniformMatCertificate(sampler.scene)
        value=sampler.uniform_certificate.sample(sampler.scene,profile,origin,axis)
        if value is not None:
            sampler.gp28_calls['uniform']+=1;return value
    sampler.gp28_calls['fine']+=1
    return fine_sample(sampler.scene,profile,origin,axis_u=axis,tolerance=tolerance)


def make_sampler(built,kind):
    if kind=='full':return FullSceneSampler(built)
    if kind=='fast':return FastScene.from_course(built)
    raise ValueError('unknown optical sampler: '+str(kind))
