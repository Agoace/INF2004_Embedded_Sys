"""Explicit authoritative sampler selection; no automatic fallback."""
from .s3_optics import sample_sensor
from .s3_fast import FastScene


class FullSceneSampler:
    def __init__(self,built):self.scene=built.scene
    def sample(self,profile,origin_mm,**kwargs):
        return sample_sensor(self.scene,profile,origin_mm,**kwargs)


def make_sampler(built,kind):
    if kind=='full':return FullSceneSampler(built)
    if kind=='fast':return FastScene.from_course(built)
    raise ValueError('unknown optical sampler: '+str(kind))
