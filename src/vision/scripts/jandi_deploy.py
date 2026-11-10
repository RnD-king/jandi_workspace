#!/usr/bin/env python3
"""Versioned JANDI source synchronization, immutable installs and launch selection.
No command here sends robot services. Only explicit `run` starts the existing launcher.
"""
from __future__ import annotations
import argparse
from contextlib import contextmanager
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import socket
import subprocess
import sys
import tarfile
import tempfile
import time
import yaml

BRANCHES={'main':'main','fast':'race-round2-fast-20261109'}
PACKAGES=('vision','forward_walk','camera_overlay','vision_core')
ORIGIN='https://github.com/RnD-king/jandi_workspace.git'
SKIP={'.git','install','log','__pycache__'}
HOME_DIR=Path.home()
CONFIG_DIR=HOME_DIR/'.config/jandi'
STATE_DIR=HOME_DIR/'.local/state/jandi'
TOOLS_DIR=HOME_DIR/'.local/share/jandi-tools'


def call(args, **kwargs):
    return subprocess.run([str(a) for a in args],check=True,**kwargs)


def git(repo,*args):
    return call(['git','-C',repo,*args],capture_output=True,text=True).stdout.rstrip("\n")


def atomic_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile('w',dir=path.parent,delete=False) as stream:
        json.dump(data,stream,ensure_ascii=False,indent=2);stream.write('\n')
        stream.flush();os.fsync(stream.fileno());temporary=Path(stream.name)
    temporary.replace(path)


class AlgorithmLoader(yaml.SafeLoader):
    """Do not silently select one of two differently tuned duplicate keys."""
    def construct_mapping(self,node,deep=False):
        mapping={}
        for key_node,value_node in node.value:
            key=self.construct_object(key_node,deep=deep)
            if not isinstance(key,str) or key in mapping:
                raise ValueError('알고리즘 YAML 중복 또는 비문자열 키: '+str(key))
            mapping[key]=self.construct_object(value_node,deep=deep)
        return mapping


def algorithm_document(text):
    try:
        data=yaml.load(text,Loader=AlgorithmLoader)
        if not isinstance(data,dict) or not isinstance(data.get('vision_algorithm'),dict):
            raise ValueError('vision_algorithm 매핑이 필요합니다.')
        return data,yaml.compose(text,Loader=AlgorithmLoader)
    except (yaml.YAMLError,ValueError) as error:
        raise RuntimeError('알고리즘 YAML 형식 오류: '+str(error)) from error


def fill_algorithm_defaults(default_text,user_text):
    """Append missing mapping keys without rewriting any existing tuning/comment."""
    defaults,_=algorithm_document(default_text)
    user,node=algorithm_document(user_text)
    lines=user_text.splitlines(keepends=True)
    offsets=[0]
    for line in lines:offsets.append(offsets[-1]+len(line))
    edits=[];added=[]
    def content_end(current):
        if isinstance(current,yaml.MappingNode) and not current.flow_style and current.value:
            return content_end(current.value[-1][1])
        if isinstance(current,yaml.SequenceNode) and not current.flow_style and current.value:
            return content_end(current.value[-1])
        return current.end_mark
    def visit(expected,actual,current,path,depth):
        if not isinstance(expected,dict):return
        if not isinstance(actual,dict) or not isinstance(current,yaml.MappingNode):
            raise RuntimeError('알고리즘 YAML 매핑 형식이 다릅니다: '+'.'.join(path))
        children={key.value:value for key,value in current.value}
        missing={}
        for key,value in expected.items():
            if key not in actual:
                missing[key]=value;added.append('.'.join((*path,key)))
            elif isinstance(value,dict):
                visit(value,actual[key],children[key],(*path,key),depth+1)
        if not missing:return
        if current.flow_style:
            # JSON and inline YAML maps keep their existing spelling/quotes.
            chunk=yaml.safe_dump(missing,allow_unicode=True,sort_keys=False,
                                 default_flow_style=True,width=10**9).strip()
            chunk=chunk[1:-1].strip()
            edits.append((current.end_mark.index-1,depth,(', ' if current.value else '')+chunk))
        else:
            end=content_end(current)
            line_start=offsets[min(end.line,len(lines))]
            # A block map ends at the next dedented key, or after its last scalar.
            at=line_start if not user_text[line_start:end.index].strip() else offsets[min(end.line+1,len(lines))]
            indent=' '*current.start_mark.column
            chunk=yaml.safe_dump(missing,allow_unicode=True,sort_keys=False)
            chunk=''.join(indent+line if line.strip() else line for line in chunk.splitlines(keepends=True))
            prefix='\n' if at and user_text[at-1]!='\n' else ''
            edits.append((at,depth,prefix+chunk))
    visit(defaults,user,node,(),0)
    result=user_text
    # At a common EOF, insert a parent first so its child's keys finish above it.
    for at,depth,chunk in sorted(edits,key=lambda e:(e[0],-e[1]),reverse=True):
        result=result[:at]+chunk+result[at:]
    merged,_=algorithm_document(result)
    def check(expected,actual):
        if isinstance(expected,dict):
            return isinstance(actual,dict) and all(k in actual and check(v,actual[k]) for k,v in expected.items())
        return True
    def retained(old,new):
        if isinstance(old,dict):
            return isinstance(new,dict) and all(k in new and retained(v,new[k]) for k,v in old.items())
        if isinstance(old,list):
            return isinstance(new,list) and len(old)==len(new) and all(retained(a,b) for a,b in zip(old,new))
        return type(old) is type(new) and (old==new or isinstance(old,float) and old!=old and new!=new)
    if not check(defaults,merged) or not retained(user,merged):
        raise RuntimeError('알고리즘 YAML 새 항목 병합에 실패했습니다.')
    return result,added


def validate_algorithm(core,path,env):
    checker=Path(core)/'libexec/shared_vision_core/config_check'
    if not checker.is_file():
        raise RuntimeError('알고리즘 검사기가 없는 이전 설치본입니다. jandi_update.sh로 해당 버전을 업데이트하세요.')
    try:
        call([checker,path],env=env,capture_output=True,text=True)
    except subprocess.CalledProcessError as error:
        detail=(error.stderr or error.stdout or str(error)).strip()
        raise RuntimeError('알고리즘 YAML 검사 실패 ('+str(path)+'): '+detail) from error


def ensure_algorithm_config(version,core,env):
    """Validate against this release, then atomically create/extend operator tuning."""
    if version not in BRANCHES:raise ValueError('잘못된 알고리즘 버전: '+version)
    defaults=Path(core)/'share/shared_vision_core/config/vision_algorithm.yaml'
    path=CONFIG_DIR/version/'vision_algorithm.yaml'
    if path.is_symlink():raise RuntimeError('알고리즘 편집 파일의 심볼릭 링크는 자동 교체하지 않습니다: '+str(path))
    baseline=defaults.read_text()
    previous=path.read_text() if path.exists() else None
    if previous is None:
        algorithm_document(baseline)
        candidate=baseline;added=[]
    else:candidate,added=fill_algorithm_defaults(baseline,previous)
    if candidate==previous:
        validate_algorithm(core,path,env)
        return path
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=None
    try:
        with tempfile.NamedTemporaryFile('w',dir=path.parent,prefix='.algorithm-',suffix='.yaml',delete=False) as stream:
            stream.write(candidate);stream.flush();os.fsync(stream.fileno());temporary=Path(stream.name)
        validate_algorithm(core,temporary,env)
        if (path.read_text() if path.exists() else None)!=previous:
            raise RuntimeError('알고리즘 파일 편집 중입니다. 저장을 마친 뒤 다시 실행하세요: '+str(path))
        if previous is not None:
            saved=STATE_DIR/'algorithm-originals'/version/(str(time.time_ns())+'.yaml')
            saved.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,saved)
            temporary.chmod(path.stat().st_mode & 0o777)
        temporary.replace(path)
    except RuntimeError as error:
        raise RuntimeError('알고리즘 편집 파일 '+str(path)+': '+str(error)) from error
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)
    print(('알고리즘 편집 파일 생성: ' if previous is None else
           '알고리즘 새 항목 추가 ('+str(len(added))+'): ')+str(path))
    return path


def load_config():
    path=CONFIG_DIR/'deployment.json'
    if not path.is_file():raise RuntimeError('먼저 jandi_deploy.py install --machine pc 또는 jetson을 실행하세요.')
    return json.loads(path.read_text())


@contextmanager
def operation_lock():
    STATE_DIR.mkdir(parents=True,exist_ok=True)
    with (STATE_DIR/'operations.lock').open('a') as stream:
        try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('다른 JANDI 업데이트/시작 작업이 진행 중입니다.')
        yield


def active_plan():
    directory=Path(f'/tmp/jandi-{os.getuid()}')
    try:
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
            client.settimeout(1);client.connect(str(directory/'manager.sock'))
            client.sendall(b'{"op":"status"}\n');data=bytearray()
            while not data.endswith(b'\n'):
                part=client.recv(65536)
                if not part:break
                data.extend(part)
                if len(data)>1024*1024:raise RuntimeError('잘못된 관리자 응답')
            reply=json.loads(data)
            if not reply.get('status',{}).get('finished'):return reply.get('plan',{})
    except (OSError,ValueError):pass
    # A crashed manager cannot make its surviving motor process safe to replace.
    path=directory/'state.json'
    if path.is_file():
        state=json.loads(path.read_text())
        for item in state.get('nodes',{}).values():
            try:
                fields=Path(f"/proc/{int(item['pid'])}/stat").read_text().rsplit(')',1)[1].split()
                if fields[0]!='Z' and str(item.get('identity'))==fields[19]:return state.get('plan',{})
            except (OSError,KeyError,ValueError,TypeError,IndexError):pass
    # The global lifetime lock also covers the period before manager.sock exists.
    lock=directory/'version.lock'
    if lock.exists():
        with lock.open('a') as stream:
            try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:return {}
    return None


def require_stopped(version=None):
    active=active_plan()
    if active is not None and (version is None or (active.get('version') not in BRANCHES or active.get('version')==version)):
        raise RuntimeError(f"JANDI {active.get('version') or '기존 버전'} 실행 중입니다. 기존 콘솔에서 전체 종료 후 진행하세요.")


def check_origin(repo):
    remote=git(repo,'remote','get-url','origin').removesuffix('.git').rstrip('/')
    if remote not in (ORIGIN.removesuffix('.git'),'git@github.com:RnD-king/jandi_workspace','ssh://git@github.com/RnD-king/jandi_workspace'):
        raise RuntimeError('예상한 origin이 아닙니다: '+remote)


def source_roots(cfg,version,repo):
    if cfg['machine']=='jetson':return {p:repo/'src'/p for p in PACKAGES}
    ws=HOME_DIR/'my_cv' if version=='main' else HOME_DIR/'my_cv/variants/fast'
    core=HOME_DIR/'vision_core' if version=='main' else HOME_DIR/'vision_core_fast'
    return {p:(core if p=='vision_core' else ws/'src'/p) for p in PACKAGES}


def files(root):
    result={}
    if root.is_symlink():raise RuntimeError('소스 루트가 심볼릭 링크입니다: '+str(root))
    if not root.exists():return result
    for base,dirs,names in os.walk(root):
        dirs[:]=[d for d in dirs if d not in SKIP and not d.startswith(('build','install_','log_'))]
        for name in names:
            p=Path(base)/name
            if name=='.git' or name.endswith('.pyc'):continue
            if p.is_symlink():raise RuntimeError('소스 내 심볼릭 링크를 자동 덮어쓰지 않습니다: '+str(p))
            result[str(p.relative_to(root))]=p.read_bytes()
    return result


def archive(repo,ref):
    with tempfile.TemporaryFile() as stream:
        call(['git','-C',repo,'archive',ref,'--',*[f'src/{p}' for p in PACKAGES]],stdout=stream)
        stream.seek(0);result={p:{} for p in PACKAGES}
        with tarfile.open(fileobj=stream) as tar:
            for member in tar:
                if not member.isfile():continue
                parts=Path(member.name).parts
                if len(parts)>2 and parts[0]=='src' and parts[1] in result:
                    result[parts[1]]['/'.join(parts[2:])]=tar.extractfile(member).read()
        return result


def reconcile(base,local,remote):
    merged={};conflicts=[]
    for name in sorted(base.keys()|local.keys()|remote.keys()):
        a,b,c=base.get(name),local.get(name),remote.get(name)
        if b!=a and c!=a and b!=c:conflicts.append(name)
        else:
            value=b if b!=a else c
            if value is not None:merged[name]=value
    if conflicts:raise RuntimeError('PC/GitHub 동시 변경 충돌; 원본을 보존하고 중단합니다: '+', '.join(conflicts[:15]))
    return merged


def write_source(root,old,new,backup):
    if old==new:return
    root.mkdir(parents=True,exist_ok=True)
    for name in old.keys()|new.keys():
        if old.get(name)==new.get(name):continue
        p=root/name
        if p.exists():
            saved=backup/name;saved.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,saved)
        if name not in new:p.unlink()
        else:
            p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(new[name])
            if p.suffix=='.sh' or new[name].startswith(b'#!/'):p.chmod(p.stat().st_mode|0o111)


def ensure_repo(cfg,version,allow_dirty=False):
    path=Path(cfg['repositories'][version]).expanduser()
    branch=BRANCHES[version]
    if path.exists():
        if git(path,'rev-parse','--show-toplevel')!=str(path.resolve()):raise RuntimeError('Git 최상위 경로가 아닙니다: '+str(path))
    else:
        base=Path(cfg['repository']).expanduser()
        if not (base/'.git').exists():
            if base.exists():raise RuntimeError('기존 비 Git 폴더는 덮어쓰지 않습니다: '+str(base))
            call(['git','clone','--no-checkout',ORIGIN,base])
        check_origin(base)
        call(['git','-C',base,'fetch','origin',f'refs/heads/{branch}:refs/remotes/origin/{branch}'])
        call(['git','-C',base,'worktree','add','--detach',path,'origin/'+branch])
    check_origin(path)
    current=git(path,'branch','--show-current')
    if current not in ('',branch,'jandi-'+version):raise RuntimeError('다른 브랜치 작업 폴더입니다: '+current)
    exclude=Path(git(path,'rev-parse','--git-path','info/exclude'))
    if not exclude.is_absolute():exclude=path/exclude
    exclude.parent.mkdir(parents=True,exist_ok=True)
    old_exclude=exclude.read_text() if exclude.exists() else ''
    patterns=['/releases/','/current.json']
    missing=[x for x in patterns if x not in old_exclude.splitlines()]
    if missing:exclude.write_text(old_exclude+'\n'+'\n'.join(missing)+'\n')
    dirty=git(path,'status','--porcelain')
    if dirty and not allow_dirty:raise RuntimeError('저장소에 미커밋 변경이 있습니다: '+str(path))
    if dirty:
        changed=git(path,'diff','--name-only','--no-renames','HEAD','-z').split('\0')
        changed+=git(path,'ls-files','--others','--exclude-standard','-z').split('\0')
        for name in filter(None,changed):
            if not any(name.startswith('src/'+p+'/') for p in PACKAGES):
                raise RuntimeError('선택 패키지 외 미커밋 변경을 자동 업로드하지 않습니다: '+name)
    return path


def synchronize(cfg,version,pushing=False):
    repo=ensure_repo(cfg,version,allow_dirty=pushing and cfg["machine"]=="jetson");branch=BRANCHES[version]
    base_ref=git(repo,'rev-parse','HEAD');base=archive(repo,base_ref)
    roots=source_roots(cfg,version,repo)
    if pushing and any(not roots[p].is_dir() for p in PACKAGES):
        raise RuntimeError('업로드할 원본 패키지 경로가 누락됐습니다.')
    disk={p:files(roots[p]) for p in PACKAGES}
    local={p:disk[p] if roots[p].exists() else base[p] for p in PACKAGES}
    # Independent PC canonical sources may contain unpublished edits: merge only nonconflicting files.
    call(['git','-C',repo,'fetch','origin',f'refs/heads/{branch}:refs/remotes/origin/{branch}'])
    remote_ref=git(repo,'rev-parse','origin/'+branch)
    if pushing:
        changed=git(repo,'diff','--name-only','--no-renames','origin/'+branch+'...HEAD','-z').split('\0')
        allowed_scripts={'scripts/'+name for name in ('jandi_update.sh','run_jandi.sh','push_jandi_workspace.sh',
            'sync_jandi_to_my_cv.sh','jetson_push_to_github.sh','jetson_pull_from_github.sh','jandi_branch_switch.sh')}
        for name in filter(None,changed):
            if name not in allowed_scripts and not any(name.startswith('src/'+p+'/') for p in PACKAGES):
                raise RuntimeError('미전송 커밋에 다른 파일이 있습니다. 자동 업로드하지 않습니다: '+name)

    ancestor=subprocess.run(['git','-C',str(repo),'merge-base','--is-ancestor',base_ref,remote_ref]).returncode==0
    if not ancestor:
        if pushing and subprocess.run(['git','-C',str(repo),'merge-base','--is-ancestor',remote_ref,base_ref]).returncode==0:
            remote_ref=base_ref
        else:raise RuntimeError('로컬/원격 이력이 갈라졌습니다. 강제 reset/push는 하지 않습니다.')
    remote=archive(repo,remote_ref)
    merged={p:reconcile(base[p],local[p],remote[p]) for p in PACKAGES}
    backup=STATE_DIR/'source-backups'/f"{version}-{time.time_ns()}"
    if cfg['machine']=='jetson' and base_ref!=remote_ref and any(local[p]!=base[p] for p in PACKAGES):
        raise RuntimeError('Jetson 미커밋 변경과 원격 새 커밋이 함께 있습니다. 원본을 보존하고 중단합니다. 먼저 로컬 변경을 커밋해 병합하세요.')
    if base_ref!=remote_ref:call(['git','-C',repo,'merge','--ff-only',remote_ref])
    if cfg['machine']=='pc':
        for p in PACKAGES:write_source(roots[p],disk[p],merged[p],backup/p)
        if not pushing and any(merged[p]!=remote[p] for p in PACKAGES):
            raise RuntimeError('PC 미전송 변경을 보존했습니다. 먼저 push 후 업데이트하세요. 보존 위치: '+str(backup))
        if pushing:
            for p in PACKAGES:write_source(repo/'src'/p,remote[p],merged[p],backup/('staging-'+p))
    return repo,roots


def clean_environment():
    env=dict(os.environ)
    for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','LD_PRELOAD',
                'PYTHONPATH','ROS_PACKAGE_PATH','VISION_CORE_ROOT','SHARED_VISION_CORE_ROOT',
                'VISION_CORE_ALGORITHM_CONFIG','YOLO26_ENGINE_PATH','YOLO26_RUNTIME_CONFIG','JANDI_RELEASE','JANDI_VERSION','JANDI_CAMERA_CONFIG','JANDI_CAMERA_LAUNCH','COLCON_CURRENT_PREFIX','AMENT_CURRENT_PREFIX'):
        env.pop(key,None)
    env['PATH']=':'.join(p for p in env.get('PATH',os.defpath).split(':')
                         if p and not any(x in p for x in ('/jandi_main/','/jandi_fast/','/my_cv/install','/variants/fast/')))
    env['ROS_DOMAIN_ID']='22'
    return env


def ros_environment(cfg,install=None,core=None):
    setups=[Path('/opt/ros/humble/setup.bash')]
    for prefix in cfg.get('dependency_prefixes',[]):
        prefix=Path(prefix).expanduser()
        indexes=list(prefix.glob('**/ament_index/resource_index/packages/vision'))+list(prefix.glob('**/ament_index/resource_index/packages/forward_walk'))
        if indexes:raise RuntimeError('공통 의존성 경로에 다른 JANDI 패키지가 있습니다: '+str(prefix))
        setup=prefix/'local_setup.bash'
        if not setup.is_file():
            matches=list(prefix.glob('share/*/local_setup.bash'))
            if len(matches)!=1:raise RuntimeError('의존성 환경을 하나로 식별할 수 없습니다: '+str(prefix))
            setup=matches[0]
        setups.append(setup)
    if install:setups.append(Path(install)/'local_setup.bash')
    for setup in setups:
        if not setup.is_file():raise RuntimeError('환경 파일 없음: '+str(setup))
    command='set -e; '+'; '.join('unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX; source '+shlex.quote(str(p)) for p in setups)+'; env -0'
    result=call(['bash','--noprofile','--norc','-c',command],env=clean_environment(),capture_output=True)
    env=dict(item.decode().split('=',1) for item in result.stdout.split(b'\0') if item)
    for library in cfg.get('library_paths',[]):
        library=str(Path(library).expanduser())
        env['LD_LIBRARY_PATH']=library+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    if core:
        core=str(core);env['VISION_CORE_ROOT']=core;env['SHARED_VISION_CORE_ROOT']=core
        env['VISION_CORE_ALGORITHM_CONFIG']=str(Path(core)/'share/shared_vision_core/config/vision_algorithm.yaml')
        env['CMAKE_PREFIX_PATH']=core+(':'+env['CMAKE_PREFIX_PATH'] if env.get('CMAKE_PREFIX_PATH') else '')
        env['LD_LIBRARY_PATH']=core+'/lib'+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    return env


def release_paths(version):
    root=HOME_DIR/('jandi_'+version)
    return root,root/'current.json'


def valid_release(data):
    if not isinstance(data,dict) or data.get('version') not in BRANCHES:return False
    if not all(isinstance(data.get(k),str) and data[k] for k in ('core','ros')):return False
    if Path(data['core']).parent!=Path(data['ros']).parent:return False
    if not isinstance(data.get('artifacts',{}),dict):return False
    paths=[Path(data['core'])/'lib/libshared_vision_core.so',
           Path(data['core'])/'libexec/shared_vision_core/config_check',
           Path(data['core'])/'share/shared_vision_core/config/vision_algorithm.yaml',
           Path(data['ros'])/'local_setup.bash',Path(data['ros'])/'vision/lib/vision/line_perception_node',
           Path(data['ros'])/'vision/lib/vision/run_jandi',Path(data['ros'])/'forward_walk/lib/forward_walk/main_node']
    if not all(p.is_file() for p in paths):return False
    for name,digest in data.get('artifacts',{}).items():
        p=Path(name)
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:return False
    return True


def fingerprint(commit,cfg,roots):
    h=hashlib.sha256(commit.encode())
    h.update(json.dumps({k:cfg.get(k) for k in ('dependency_prefixes','cmake_args','machine')},sort_keys=True).encode())
    for name in PACKAGES:
        for path,data in sorted(files(roots[name]).items()):h.update(name.encode()+b'\0'+path.encode()+b'\0'+data)
    return h.hexdigest()


def build_release(cfg,version,repo,roots,force=False):
    commit=git(repo,'rev-parse','HEAD');stamp=fingerprint(commit,cfg,roots)
    root,current=release_paths(version)
    previous=json.loads(current.read_text()) if current.exists() else None
    if not force and valid_release(previous) and previous.get('version')==version and previous.get('fingerprint')==stamp:
        ensure_algorithm_config(version,previous['core'],ros_environment(cfg,previous['ros'],previous['core']))
        print(f'[{version.upper()}] 같은 소스의 정상 설치본이 있습니다. 빌드 생략.');return previous
    # Build into its final immutable prefix. Never move an installed tree with embedded RPATHs.
    release=root/'releases'/f'{commit[:12]}-{time.time_ns()}'
    core=release/'core';ros=release/'ros';build=release/'build'
    release.mkdir(parents=True)
    log=release/'build.log';env=ros_environment(cfg,core=core)
    jobs=int(cfg.get('jobs',2))
    if jobs<1:raise RuntimeError('jobs는 1 이상이어야 합니다.')
    jobs=str(jobs)
    env['CMAKE_BUILD_PARALLEL_LEVEL']=jobs
    env['MAKEFLAGS']='-j'+jobs
    commands=[['cmake','-S',roots['vision_core'],'-B',build/'core','-DCMAKE_BUILD_TYPE=Release',
               '-DBUILD_TESTING=OFF',f'-DCMAKE_INSTALL_PREFIX={core}'],
              ['cmake','--build',build/'core','-j'+jobs],['cmake','--install',build/'core'],
              ['colcon','--log-base',release/'log','build','--base-paths',*[roots[p] for p in PACKAGES if p!='vision_core'],
               '--build-base',build/'ros','--install-base',ros,'--executor','sequential',
               '--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF',f'-DSHARED_VISION_CORE_ROOT={core}',
               *cfg.get('cmake_args',[])]]
    try:
        with log.open('w') as stream:
            for index,command in enumerate(commands,1):
                print(f'[{version.upper()}] 빌드 {index}/{len(commands)} — {log}',flush=True)
                call(command,env=env,cwd=release,stdout=stream,stderr=subprocess.STDOUT)
        if fingerprint(commit,cfg,roots)!=stamp:raise RuntimeError('빌드 중 소스가 변경됐습니다. 설치본을 활성화하지 않습니다.')
        data={'version':version,'branch':BRANCHES[version],'commit':commit,'fingerprint':stamp,
              'core':str(core),'ros':str(ros),'release':str(release),'completed_at':datetime.now().isoformat()}
        if not valid_release(data):raise RuntimeError('필수 설치 파일이 누락됐습니다.')
        ensure_algorithm_config(version,core,env)
        data['artifacts']={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for prefix in (core,ros) for p in prefix.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
        # Both paths become visible in one manifest replacement, only after all installs succeed.
        atomic_json(release/'complete.json',data)
        atomic_json(current,data)
        # Compatibility aliases are conveniences only. Launch always reads the atomic manifest.
        for link,target in ((root/'install',ros),(HOME_DIR/('vision_core_'+version)/'install',core)):
            if link.exists() and not link.is_symlink():
                print('기존 일반 디렉터리를 보존합니다: '+str(link));continue
            try:
                link.parent.mkdir(parents=True,exist_ok=True)
                temporary=link.with_name(link.name+'.new-'+str(time.time_ns()))
                temporary.symlink_to(target,target_is_directory=True);temporary.replace(link)
            except OSError as error:
                print('편의 링크를 갱신하지 못했습니다. current.json 설치본은 정상입니다: '+str(error),file=sys.stderr)
        print(f'[{version.upper()}] 설치 완료: {commit[:12]}');return data
    except Exception:
        print('업데이트 실패. 기존 current.json과 설치본을 유지합니다. 로그: '+str(log),file=sys.stderr)
        raise


def choose_start(default):
    if not sys.stdin.isatty():raise RuntimeError('실행 선택에는 터미널이 필요합니다. --version과 --no-menu를 지정하세요.')
    version=default;show=False;terminal=False;stream=True
    while True:
        print('\n========== JANDI START ==========')
        print(f'1. 실행 버전: {version.upper()}\n3. OpenCV 디버그 화면: {"ON" if show else "OFF"}\n4. 터미널: {"ON" if terminal else "OFF"}\n5. HTTP MJPEG: {"ON" if stream else "OFF"}\nEnter: 실행 / 0: 취소')
        key=input('선택: ').strip()
        if not key:return version,show,terminal,stream
        if key=='0':return None
        if key=='1':version='fast' if version=='main' else 'main'
        elif key=='3':show=not show
        elif key=='4':terminal=not terminal
        elif key=='5':stream=not stream


def start(cfg,args):
    active=active_plan()
    if active is not None:
        # Reconnecting to the same live console must not accidentally spawn another version.
        print('실행 중: '+(active.get('version') or '기존 버전')+' — 기존 JANDI 전체 종료 전 새 실행은 거부합니다.')
        if args.version and args.version!=active.get('version'):
            return 1
        script=Path(active.get('share','/missing')).parent.parent/'lib/vision/run_jandi'
        if not script.is_file():raise RuntimeError('기존 콘솔 경로를 찾을 수 없습니다. 기존 설치의 콘솔로 연결하세요.')
        print('기존 콘솔에 다시 연결합니다. 새 노드를 시작하지 않습니다.')
        return call(['/usr/bin/python3',script,'attach']).returncode
    if args.attach:raise RuntimeError('연결할 실행 중 JANDI가 없습니다.')
    selected=choose_start(args.version or cfg.get('selected','main')) if not args.no_menu else (
        args.version or cfg.get('selected','main'),args.opencv,args.terminal,args.http)
    if selected is None:return 0
    version,show,terminal,stream=selected
    _,current=release_paths(version)
    if not current.exists():raise RuntimeError(f'{version.upper()} 설치본이 없습니다. jandi_update.sh {version}')
    data=json.loads(current.read_text())
    if not valid_release(data) or data.get('version')!=version:raise RuntimeError('설치본 버전이 다르거나 불완전합니다. 업데이트 또는 --force 재빌드가 필요합니다.')
    env=ros_environment(cfg,data['ros'],data['core'])
    env.update(JANDI_VERSION=version,JANDI_RELEASE=data['release'])
    algorithm=ensure_algorithm_config(version,data['core'],env)
    env['VISION_CORE_ALGORITHM_CONFIG']=str(algorithm.resolve())
    repo=Path(cfg['repositories'][version]).expanduser()
    runtime=source_roots(cfg,version,repo)['vision']/'config/yolo26_runtime.yaml'
    if not runtime.is_file():raise RuntimeError('YOLO runtime 파일 없음: '+str(runtime))
    env['YOLO26_RUNTIME_CONFIG']=str(runtime.resolve())
    script=Path(data['ros'])/'vision/lib/vision/run_jandi'
    command=['/usr/bin/python3',str(script),'--no-menu','--workspace',str(HOME_DIR/('jandi_'+version)),
             '--core-prefix',data['core'],'--show-debug-view',str(show).lower(),
             '--terminal-node',str(terminal).lower(),'--stream-debug-view',str(stream).lower()]
    if args.background:command.append('--background')
    # Resolve through the installed runtime loader before camera/motor startup.
    # Only the existing YAML path is passed; deployment owns no engine setting.
    probe=("import sys; from pathlib import Path; sys.path.insert(0,sys.argv[1]); "
           "from jandi_runtime import Plan; p=Plan.load(sys.argv[2],sys.argv[3],sys.argv[4]); "
           "engine=Path(p.engine); "
           "engine.is_file() or sys.exit('YOLO 엔진 파일 없음: '+str(engine)); "
           "print('YOLO engine: '+str(engine),flush=True)")
    call(['/usr/bin/python3','-s','-c',probe,str(script.parent),
          str(Path(data['ros'])/'vision/share/vision'),str(HOME_DIR/('jandi_'+version)),data['core']],env=env)
    # Starts only the existing backend; lifetime version lock is owned by that backend.
    return call(command,env=env).returncode


def install(machine):
    CONFIG_DIR.mkdir(parents=True,exist_ok=True);TOOLS_DIR.mkdir(parents=True,exist_ok=True)
    target=TOOLS_DIR/'jandi_deploy.py'
    if Path(__file__).resolve()!=target.resolve():shutil.copy2(__file__,target)
    config=CONFIG_DIR/'deployment.json'
    if not config.exists():
        bases=[HOME_DIR/'my_cv/install',HOME_DIR/'jandi/install']
        if machine=='jetson':bases=[HOME_DIR/'realsense_ws/install',HOME_DIR/'jandi/install',HOME_DIR/'my_cv/install']
        prefixes=[]
        for name in ('realsense2_camera_msgs','realsense2_description','realsense2_camera','dynamixel_sdk'):
            found=next((base/name for base in bases if (base/name/'share'/name/'local_setup.bash').is_file()),None)
            if found:prefixes.append(str(found))
        repo=HOME_DIR/('jandi_workspace' if machine=='pc' else 'jandi')
        repos={'main':str(repo),'fast':str(HOME_DIR/'jandi_workspace_fast')} if machine=='pc' else {
            v:str(HOME_DIR/('jandi_'+v)) for v in BRANCHES}
        cfg={'machine':machine,'repository':str(repo),'repositories':repos,'selected':'main','jobs':2,
             'dependency_prefixes':prefixes,
             'cmake_args':[],'library_paths':[str(p) for p in [Path('/usr/local/cuda/lib64')] if p.is_dir()]}
        atomic_json(config,cfg)
    cfg=json.loads(config.read_text())
    cfg.pop('camera_config',None);cfg.pop('camera_launch',None);cfg.pop('engine',None)
    atomic_json(config,cfg)
    wrappers={'push_jandi_workspace.sh':('push','pc'),'sync_jandi_to_my_cv.sh':('update','pc'),
              'jetson_push_to_github.sh':('push','jetson'),'jetson_pull_from_github.sh':('update','jetson'),
              'jandi_update.sh':('update',machine),'jandi_branch_switch.sh':('select',machine),'run_jandi.sh':('run',machine)}
    saved=STATE_DIR/'script-originals'/str(time.time_ns())
    # Retire only our exact old wrappers. Keep user-modified files and all saved originals.
    for name,command in (('jandi_camera.sh','camera'),('jandi_depth.sh','depth')):
        path=HOME_DIR/name
        expected='#!/usr/bin/env bash\n# JANDI versioned entrypoint\nset -euo pipefail\nexec /usr/bin/python3 '+shlex.quote(str(target))+' '+command+' --machine '+machine+' "$@"\n'
        if path.is_file() and not path.is_symlink() and path.read_text()==expected:
            saved.mkdir(parents=True,exist_ok=True);path.replace(saved/name)
    for name,(command,host) in wrappers.items():
        if name.startswith('jetson_') and machine=='pc':continue
        if name in ('push_jandi_workspace.sh','sync_jandi_to_my_cv.sh') and machine=='jetson':continue
        p=HOME_DIR/name
        if p.exists() or p.is_symlink():
            saved.mkdir(parents=True,exist_ok=True);shutil.copy2(p,saved/name)
        p.write_text('#!/usr/bin/env bash\n# JANDI versioned entrypoint\nset -euo pipefail\nexec /usr/bin/python3 '+shlex.quote(str(target))+' '+command+' --machine '+host+' "$@"\n')
        p.chmod(0o755)
    bin_dir=HOME_DIR/'.local/bin';bin_dir.mkdir(parents=True,exist_ok=True)
    entry=bin_dir/'run_jandi'
    if entry.exists() or entry.is_symlink():
        saved.mkdir(parents=True,exist_ok=True);shutil.copy2(entry,saved/'run_jandi')
    entry.write_text('#!/usr/bin/env bash\n# JANDI versioned entrypoint\nexec bash '+shlex.quote(str(HOME_DIR/'run_jandi.sh'))+' "$@"\n')
    entry.chmod(0o755)
    print('명령 설치 완료. 기존 파일 보존: '+str(saved))
    print('설정: '+str(config)+'\n카메라: ros2 launch realsense2_camera rs_launch.py')
    print('브랜치 선택은 기본 버전만 바꿉니다. 설치: ~/jandi_update.sh main 또는 fast')
    print('튜닝 YAML (update 후 생성): '+str(CONFIG_DIR/'main/vision_algorithm.yaml')+' / '+str(CONFIG_DIR/'fast/vision_algorithm.yaml'))


def main(argv=None):
    argv=list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0]=='run' and 'attach' in argv[1:]:
        argv.remove('attach');argv+=['--attach']
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('install','update','push','select','run','status'))
    parser.add_argument('version',nargs='?',choices=tuple(BRANCHES))
    parser.add_argument('--version',dest='explicit_version',choices=tuple(BRANCHES))
    parser.add_argument('--machine',choices=('pc','jetson'))
    parser.add_argument('--force',action='store_true');parser.add_argument('--message',default='Update JANDI canonical sources')
    parser.add_argument('--no-menu',action='store_true');parser.add_argument('--background',action='store_true')
    parser.add_argument('--attach',action='store_true')
    parser.add_argument('--opencv',action='store_true');parser.add_argument('--terminal',action='store_true')
    parser.add_argument('--no-http',dest='http',action='store_false',default=True)
    args=parser.parse_intermixed_args(argv);args.version=args.explicit_version or args.version
    if args.command=='install':install(args.machine or 'jetson');return 0
    cfg=load_config()
    if args.machine and cfg['machine']!=args.machine:raise RuntimeError('PC/Jetson 설정이 일치하지 않습니다.')
    legacy_branch=os.environ.get('JANDI_BRANCH') if not args.version else None
    legacy_version=next((v for v,b in BRANCHES.items() if b==legacy_branch),None)
    if legacy_branch and not legacy_version:raise RuntimeError('MAIN/FAST 외 JANDI_BRANCH는 허용하지 않습니다.')
    version=args.version or legacy_version or cfg.get('selected','main')
    if args.command=='status':
        print(json.dumps({'active':active_plan(),'selected':cfg.get('selected'),'installs':{
            v:json.loads(release_paths(v)[1].read_text()) if release_paths(v)[1].exists() else None for v in BRANCHES}},indent=2));return 0
    with operation_lock():
        if args.command=='select':
            require_stopped();cfg['selected']=version;atomic_json(CONFIG_DIR/'deployment.json',cfg)
            print('기본 버전 선택: '+version.upper()+' (Git 브랜치/설치본 변경 없음)');return 0
        if args.command=='run':return start(cfg,args)
        require_stopped(version)
        repo,roots=synchronize(cfg,version,pushing=args.command=='push')
        if args.command=='update':build_release(cfg,version,repo,roots,args.force)
        else:
            call(['git','-C',repo,'add','-A','--',*[f'src/{p}' for p in PACKAGES]])
            call(['git','-C',repo,'diff','--cached','--check'])
            if git(repo,'diff','--cached','--name-only'):call(['git','-C',repo,'commit','-m',args.message])
            # No force pushes and no backup refs; a concurrent remote advance rejects the push.
            call(['git','-C',repo,'push','origin',f'HEAD:refs/heads/{BRANCHES[version]}'])
    return 0

if __name__=='__main__':
    try:sys.exit(main())
    except (OSError,ValueError,RuntimeError,subprocess.CalledProcessError) as error:
        print('JANDI [ERROR]: '+str(error),file=sys.stderr);sys.exit(1)
