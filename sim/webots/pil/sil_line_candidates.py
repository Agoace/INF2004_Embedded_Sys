"""Read-only SW9 Git source overlay; never merge or edit firmware sources."""
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
from .sil_line import ROOT
from .sil_search import SEARCH_SOURCES,SearchController

REVISION='89b7935a91de915582d264e094339d9ccb9ec1d0'
BASE_REVISION='1311338c2ef24f4d3722077bc59f4bcea211319a'


def build_candidate(directory,gain,*,base_mm_s=146,same_tick=False,revision=REVISION):
    if gain not in (85,60,40):raise ValueError('authorized candidate gains85/60/40 only')
    if base_mm_s not in (146,120,100):raise ValueError('authorized bases146/120/100 only')
    if type(same_tick)is not bool or revision not in (REVISION,BASE_REVISION):raise ValueError('explicit timing/source required')
    if base_mm_s!=146 and revision!=BASE_REVISION:raise ValueError('LINE_BASE requires SW9 base adapter revision')
    directory=Path(directory);snapshot=directory/'sw9-source';snapshot.mkdir()
    archive=subprocess.check_output(['git','archive',revision,'app_program'],cwd=ROOT)
    with tarfile.open(fileobj=io.BytesIO(archive))as tar:tar.extractall(snapshot,filter='data')
    # Native ABI adapters belong to this simulator. Every firmware C/header is
    # obtained from SW9's exact code commit, including line_p_near.h.
    for name in ('search_adapter.c','motor_adapter.c'):
        relative=Path('sim/webots/controllers/sil_line')/name;target=snapshot/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/relative,target)
    commands=[];objects=[];base=['cc','-std=c11','-O2','-fPIC','-Wall','-Wextra','-Wconversion','-Wshadow','-Werror','-I',str(snapshot)]
    adapter='app_program/vehicle/line_p_near.h'
    for i,source in enumerate(SEARCH_SOURCES):
        obj=directory/f'candidate-{i}.o';flags=[]
        if i==0 and (gain!=85 or base_mm_s!=146):
            flags=[f'-DLINE_P_NEAR={gain}','-include',str(snapshot/adapter),'-I',str(snapshot/'app_program/vehicle')]
            if revision==BASE_REVISION:flags.append(f'-DLINE_BASE={base_mm_s}')
        if i==0 and same_tick:flags.append('-DSIL_SAME_TICK_APPLY=1')
        command=[*base,*flags,'-c',str(snapshot/source),'-o',str(obj)];commands.append(command)
        subprocess.run(command,check=True,capture_output=True,text=True);objects.append(obj)
    library=directory/'libsil_line.so';command=['cc','-shared',*(str(o)for o in objects),'-o',str(library)]
    subprocess.run(command,check=True,capture_output=True,text=True);commands.append(command)
    metadata=SearchController(library).metadata()
    if metadata['steer']!=[base_mm_s,gain,85,62,378,50,340]:raise ValueError('candidate native config mismatch')
    hashes={str(p.relative_to(snapshot)):hashlib.sha256(p.read_bytes()).hexdigest()for p in snapshot.rglob('*')if p.is_file()}
    record=dict(source_commit=revision,source_policy='read-only git archive overlay, NOT merged',gain=gain,base_mm_s=base_mm_s,same_tick_apply=same_tick,
        adapter=adapter if gain!=85 or base_mm_s!=146 else None,forced_include_scope='search_adapter.c owns LINE_CFG; includes motor_adapter.c in same translation unit',
        sources_sha256=hashes,commands=commands,configuration=metadata,library_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
    (directory/'sil-build.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n');return record
