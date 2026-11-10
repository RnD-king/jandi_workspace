#!/usr/bin/python3
"""Failure isolation and environment tests; compiler/ROS/hardware commands are mocked."""
import contextlib
import hashlib
import io
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import jandi_deploy as d


def fixture_release(root):
    data={'core':str(root/'core'),'ros':str(root/'ros'),'commit':'a'*40,'version':'main','release':str(root)}
    for path in [root/'core/lib/libshared_vision_core.so',root/'core/share/shared_vision_core/config/vision_algorithm.yaml',
                 root/'ros/local_setup.bash',root/'ros/vision/lib/vision/line_perception_node',
                 root/'ros/vision/lib/vision/run_jandi',root/'ros/forward_walk/lib/forward_walk/main_node']:
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'installed fixture')
    defaults=root/'core/share/shared_vision_core/config/vision_algorithm.yaml'
    defaults.write_text('vision_algorithm:\n  test_gain: 1 # fixture default\n')
    checker=root/'core/libexec/shared_vision_core/config_check'
    checker.parent.mkdir(parents=True,exist_ok=True)
    checker.write_text("#!/usr/bin/python3\nimport sys,yaml\na=yaml.safe_load(open(sys.argv[1]))['vision_algorithm']\nif a.get('test_gain',0)<=0:\n sys.stderr.write('test_gain must be positive\\n');sys.exit(1)\n")
    checker.chmod(0o755)
    return data


def fixture_roots(root):
    roots={p:root/p for p in d.PACKAGES}
    for path in roots.values():path.mkdir(parents=True,exist_ok=True)
    path=roots['vision_core']/'config/vision_algorithm.yaml'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text('vision_algorithm:\n  test_gain: 2 # shared source\n')
    return roots


class DeployTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        for name,value in [('HOME_DIR',self.root),('STATE_DIR',self.root/'state'),('CONFIG_DIR',self.root/'config')]:
            p=patch.object(d,name,value);p.start();self.addCleanup(p.stop)

    def test_three_way_preserves_independent_edits_and_deletions(self):
        self.assertEqual(d.reconcile({'a':b'a','b':b'b','gone':b'old'},
                                     {'a':b'local','b':b'b'},
                                     {'a':b'a','b':b'remote','gone':b'old'}),{'a':b'local','b':b'remote'})
        with self.assertRaisesRegex(RuntimeError,'충돌'):
            d.reconcile({'a':b'base'},{'a':b'local'},{'a':b'remote'})

    def test_source_write_preserves_original_binary_before_replace(self):
        source=self.root/'source';source.mkdir();(source/'a').write_bytes(b'\x00old')
        d.write_source(source,{'a':b'\x00old'},{'a':b'\x00new'},self.root/'saved')
        self.assertEqual((self.root/'saved/a').read_bytes(),b'\x00old')
        self.assertEqual((source/'a').read_bytes(),b'\x00new')

    def test_foreign_workspace_environment_is_removed(self):
        with patch.dict(os.environ,{'AMENT_PREFIX_PATH':'/wrong','CMAKE_PREFIX_PATH':'/wrong',
             'LD_LIBRARY_PATH':'/wrong','PYTHONPATH':'/wrong','VISION_CORE_ROOT':'/wrong',
             'VISION_CORE_ALGORITHM_CONFIG':'/wrong','SHARED_VISION_CORE_ROOT':'/wrong','JANDI_VERSION':'fast',
             'YOLO26_ENGINE_PATH':'/wrong.engine','YOLO26_RUNTIME_CONFIG':'/wrong.yaml',
             'PATH':'/home/u/jandi_fast/install/bin:/usr/bin'},clear=True):
            env=d.clean_environment()
        for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH',
                    'VISION_CORE_ROOT','VISION_CORE_ALGORITHM_CONFIG','SHARED_VISION_CORE_ROOT','JANDI_VERSION','YOLO26_ENGINE_PATH','YOLO26_RUNTIME_CONFIG'):
            self.assertNotIn(key,env)
        self.assertEqual(env['PATH'],'/usr/bin');self.assertEqual(env['ROS_DOMAIN_ID'],'22')

    def test_active_target_update_refused(self):
        with patch.object(d,'active_plan',return_value={'version':'main'}):
            with self.assertRaisesRegex(RuntimeError,'실행 중'):d.require_stopped('main')
            d.require_stopped('fast')
            with self.assertRaises(RuntimeError):d.require_stopped()
        with patch.object(d,'active_plan',return_value={}):
            with self.assertRaises(RuntimeError):d.require_stopped('main')

    def test_missing_or_modified_install_cannot_be_reused(self):
        data=fixture_release(self.root/'release')
        p=Path(data['core'])/'lib/libshared_vision_core.so'
        data['artifacts']={str(p):hashlib.sha256(p.read_bytes()).hexdigest()}
        self.assertTrue(d.valid_release(data));p.write_bytes(b'changed')
        self.assertFalse(d.valid_release(data));p.unlink();self.assertFalse(d.valid_release(data))

    def test_failed_new_build_does_not_publish_over_previous_success(self):
        _,manifest=d.release_paths('main');old=fixture_release(self.root/'previous');d.atomic_json(manifest,old)
        before=manifest.read_bytes();roots=fixture_roots(self.root)
        with patch.object(d,'git',return_value='a'*40),patch.object(d,'ros_environment',return_value={}), \
             patch.object(d,'call',side_effect=subprocess.CalledProcessError(1,['colcon'])):
            with self.assertRaises(subprocess.CalledProcessError):
                d.build_release({'machine':'pc'},'main',self.root,roots,True)
        self.assertEqual(manifest.read_bytes(),before);self.assertTrue(d.valid_release(old))

    def test_new_build_only_publishes_after_all_steps(self):
        _,manifest=d.release_paths('main');roots=fixture_roots(self.root)
        commands=[]
        def fake_call(command,**kwargs):
            self.assertFalse(manifest.exists());commands.append(command)
            self.assertEqual(kwargs['env']['CMAKE_BUILD_PARALLEL_LEVEL'],'2')
            self.assertEqual(kwargs['env']['MAKEFLAGS'],'-j2')
            if command[0]=='colcon':
                ros=Path(command[command.index('--install-base')+1]);fixture_release(ros.parent)
            return subprocess.CompletedProcess(command,0)
        with patch.object(d,'git',return_value='b'*40),patch.object(d,'ros_environment',return_value={}),patch.object(d,'call',side_effect=fake_call):
            installed=d.build_release({'machine':'pc'},'main',self.root,roots)
        self.assertEqual(len(commands),5);self.assertEqual(json.loads(manifest.read_text()),installed)
        self.assertTrue(d.valid_release(installed))
        with patch.object(d,'git',return_value='b'*40),patch.object(d,'ros_environment',return_value={}),patch.object(d,'call') as call:
            self.assertEqual(d.build_release({'machine':'pc'},'main',self.root,roots),installed)
            self.assertEqual(call.call_count,1)
            self.assertEqual(Path(call.call_args[0][0][0]).name,'config_check')

    def test_incremental_build_adopts_cache_and_keeps_paths_across_commits_and_force(self):
        root,manifest=d.release_paths('main')
        old=fixture_release(root/'releases/old-build-id')
        cache=Path(old['release'])/'build/core/CMakeCache.txt'
        cache.parent.mkdir(parents=True);cache.write_text('existing cached compilation')
        ros_cache=Path(old['release'])/'build/ros/vision/unchanged.o'
        ros_cache.parent.mkdir(parents=True);ros_cache.write_bytes(b'existing object')
        log=Path(old['release'])/'build.log';log.write_text('previous build log\n')
        d.atomic_json(manifest,old);roots=fixture_roots(self.root/'sources')
        build_paths=[];real=d.call
        def fake_build(command,**kwargs):
            if Path(command[0]).name=='config_check':return real(command,**kwargs)
            self.assertTrue((Path(old['release'])/'building.json').is_file())
            self.assertFalse(d.valid_release(old))
            self.assertEqual(cache.read_text(),'existing cached compilation')
            self.assertEqual(ros_cache.read_bytes(),b'existing object')
            if command[0]=='cmake' and '-B' in command:
                build_paths.append(Path(command[command.index('-B')+1]))
            if command[0]=='colcon':
                self.assertEqual(Path(command[command.index('--build-base')+1]),Path(old['release'])/'build/ros')
                self.assertEqual(Path(command[command.index('--install-base')+1]),Path(old['ros']))
            return subprocess.CompletedProcess(command,0)
        with patch.object(d,'ros_environment',return_value={}),patch.object(d,'call',side_effect=fake_build):
            for commit,force in [('b'*40,False),('c'*40,False),('c'*40,True)]:
                with patch.object(d,'git',return_value=commit):
                    updated=d.build_release({'machine':'pc'},'main',self.root,roots,force)
                self.assertEqual(updated['commit'],commit)
                self.assertEqual(updated['release'],old['release'])
                self.assertEqual(updated['core'],old['core']);self.assertEqual(updated['ros'],old['ros'])
                self.assertTrue(d.valid_release(updated))
                self.assertFalse((Path(old['release'])/'building.json').exists())
        self.assertEqual(build_paths,[cache.parent]*3)
        self.assertEqual(list((root/'releases').iterdir()),[Path(old['release'])])
        self.assertTrue(log.read_text().startswith('previous build log\n'))

    def test_partial_incremental_install_blocks_start_and_resumes_same_cache(self):
        root,manifest=d.release_paths('main');old=fixture_release(root/'releases/old-build-id')
        cache=Path(old['release'])/'build/core/cached.o'
        cache.parent.mkdir(parents=True);cache.write_bytes(b'reusable object')
        library=Path(old['core'])/'lib/libshared_vision_core.so'
        old['artifacts']={str(library):hashlib.sha256(library.read_bytes()).hexdigest()}
        d.atomic_json(manifest,old);before=manifest.read_bytes()
        roots=fixture_roots(self.root/'sources');real=d.call
        other=fixture_release(d.release_paths('fast')[0]/'releases/fast-old');other['version']='fast'
        other_file=Path(other['core'])/'lib/libshared_vision_core.so';other_bytes=other_file.read_bytes()
        d.atomic_json(d.release_paths('fast')[1],other)
        def fail_after_core_install(command,**kwargs):
            if command[0]=='cmake' and '--install' in command:library.write_bytes(b'new core installed')
            if command[0]=='colcon':raise subprocess.CalledProcessError(1,command)
            return subprocess.CompletedProcess(command,0)
        with patch.object(d,'git',return_value='b'*40),patch.object(d,'ros_environment',return_value={}), \
             patch.object(d,'call',side_effect=fail_after_core_install):
            with self.assertRaises(subprocess.CalledProcessError):
                d.build_release({'machine':'pc'},'main',self.root,roots)
        self.assertEqual(manifest.read_bytes(),before)
        self.assertEqual(library.read_bytes(),b'new core installed')
        self.assertFalse(d.valid_release(old));self.assertTrue(d.valid_release(other))
        self.assertEqual(other_file.read_bytes(),other_bytes)
        pending=Path(old['release'])/'building.json';self.assertTrue(pending.is_file())
        with patch.object(d,'load_config',return_value={'machine':'pc'}), \
             patch.object(d,'active_plan',return_value=None),patch.object(d,'call') as call:
            with self.assertRaisesRegex(RuntimeError,'완료되지'):d.main(['run','main','--no-menu'])
            call.assert_not_called()
        def recover(command,**kwargs):
            if Path(command[0]).name=='config_check':return real(command,**kwargs)
            self.assertTrue(pending.exists());self.assertEqual(cache.read_bytes(),b'reusable object')
            if command[0]=='colcon':self.assertEqual(Path(command[command.index('--install-base')+1]),Path(old['ros']))
            return subprocess.CompletedProcess(command,0)
        with patch.object(d,'git',return_value='b'*40),patch.object(d,'ros_environment',return_value={}), \
             patch.object(d,'call',side_effect=recover):
            updated=d.build_release({'machine':'pc'},'main',self.root,roots)
        self.assertEqual(updated['release'],old['release']);self.assertTrue(d.valid_release(updated))
        self.assertFalse(pending.exists());self.assertEqual(json.loads(manifest.read_text())['commit'],'b'*40)

    def test_interrupted_first_build_reuses_work_directory_and_pending_blocks_old_stamp(self):
        roots=fixture_roots(self.root/'sources');root,manifest=d.release_paths('main')
        with patch.object(d,'git',return_value='a'*40),patch.object(d,'ros_environment',return_value={}), \
             patch.object(d,'call',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):d.build_release({'machine':'pc'},'main',self.root,roots)
        working=root/'releases/incremental';self.assertTrue((working/'building.json').is_file())
        self.assertFalse(manifest.exists())
        data=fixture_release(working)
        self.assertFalse(d.valid_release(data),'pending build bypassed by intact installed files')
        cache=working/'build/core/retained.o';cache.parent.mkdir(parents=True);cache.write_bytes(b'partial cached object')
        real=d.call
        def resume(command,**kwargs):
            self.assertEqual(cache.read_bytes(),b'partial cached object')
            if Path(command[0]).name=='config_check':return real(command,**kwargs)
            return subprocess.CompletedProcess(command,0)
        with patch.object(d,'git',return_value='a'*40),patch.object(d,'ros_environment',return_value={}), \
             patch.object(d,'call',side_effect=resume):
            updated=d.build_release({'machine':'pc'},'main',self.root,roots)
        self.assertEqual(Path(updated['release']),working);self.assertTrue(d.valid_release(updated))
        self.assertEqual(list((root/'releases').iterdir()),[working])

    def test_mixed_core_and_ros_releases_are_rejected(self):
        a=fixture_release(self.root/'a');b=fixture_release(self.root/'b')
        a['ros']=b['ros']
        self.assertFalse(d.valid_release(a))

    def test_first_fast_sync_populates_missing_canonical_directories(self):
        roots={p:self.root/p for p in d.PACKAGES}
        base={p:{'source.cpp':p.encode()} for p in d.PACKAGES}
        with patch.object(d,'ensure_repo',return_value=self.root/'repo'), \
             patch.object(d,'source_roots',return_value=roots), \
             patch.object(d,'archive',side_effect=[base,base]), \
             patch.object(d,'git',side_effect=['a'*40,'a'*40]), \
             patch.object(d,'call'), \
             patch.object(d.subprocess,'run',return_value=subprocess.CompletedProcess([],0)):
            d.synchronize({'machine':'pc'},'fast')
        for name,root in roots.items():self.assertEqual((root/'source.cpp').read_bytes(),name.encode())

    def test_background_uses_explicit_flags_without_mode(self):
        with patch.object(d,'load_config',return_value={'machine':'pc','selected':'main'}), \
             patch.object(d,'start',return_value=0) as start:
            self.assertEqual(d.main(['run','--machine','pc','--background','--no-menu','--terminal']),0)
        args=start.call_args[0][1]
        self.assertTrue(args.no_menu);self.assertTrue(args.terminal);self.assertTrue(args.background)
        self.assertFalse(hasattr(args,'mode'))

    def test_menu_has_version_and_three_flags_only(self):
        with patch.object(d.sys.stdin,'isatty',return_value=True), \
             patch('builtins.input',side_effect=['1','3','4','5','']), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            result=d.choose_start('main')
        self.assertEqual(result,('fast',True,True,False))
        self.assertNotIn('PRACTICE',output.getvalue());self.assertNotIn('실행 모드',output.getvalue())

    def test_install_retires_overrides_and_only_unmodified_helper_wrappers(self):
        d.CONFIG_DIR.mkdir(parents=True)
        old=d.CONFIG_DIR/'old.yaml';old.write_text('old user settings')
        launch=d.CONFIG_DIR/'realsense.launch.py';launch.write_text('# user old launch')
        d.atomic_json(d.CONFIG_DIR/'deployment.json',{'machine':'pc','selected':'main','repositories':{v:str(self.root/('repo-'+v)) for v in d.BRANCHES},'camera_config':str(old),'camera_launch':str(launch),'engine':'/obsolete.engine'})
        target=self.root/'tools/jandi_deploy.py'
        original='#!/usr/bin/env bash\n# JANDI versioned entrypoint\nset -euo pipefail\nexec /usr/bin/python3 '+str(target)+' camera --machine pc "$@"\n'
        (self.root/'jandi_camera.sh').write_text(original)
        (self.root/'jandi_depth.sh').write_text('# user modified helper')
        with patch.object(d,'TOOLS_DIR',self.root/'tools'),contextlib.redirect_stdout(io.StringIO()):
            d.install('pc')
            config=json.loads((d.CONFIG_DIR/'deployment.json').read_text())
            self.assertNotIn('camera_config',config);self.assertNotIn('camera_launch',config);self.assertNotIn('engine',config)
            d.install('pc')
        self.assertEqual(old.read_text(),'old user settings')
        self.assertEqual(launch.read_text(),'# user old launch')
        self.assertFalse((self.root/'jandi_camera.sh').exists())
        self.assertTrue(any(x.read_text()==original for x in d.STATE_DIR.glob('script-originals/*/jandi_camera.sh')))
        self.assertEqual((self.root/'jandi_depth.sh').read_text(),'# user modified helper')

    def test_jetson_install_detects_actual_realsense_workspace(self):
        prefix=self.root/'realsense_ws/install/realsense2_camera'
        setup=prefix/'share/realsense2_camera/local_setup.bash'
        setup.parent.mkdir(parents=True);setup.write_text('# existing driver')
        with patch.object(d,'TOOLS_DIR',self.root/'tools'),contextlib.redirect_stdout(io.StringIO()):
            d.install('jetson')
        cfg=json.loads((d.CONFIG_DIR/'deployment.json').read_text())
        self.assertIn(str(prefix),cfg['dependency_prefixes'])
        self.assertNotIn('engine',cfg)
        self.assertFalse((d.CONFIG_DIR/'realsense.launch.py').exists())

    def test_start_reads_selected_source_runtime_and_preflight_blocks_missing_engine(self):
        cfg={'machine':'jetson','selected':'main','repositories':{v:str(self.root/('repo-'+v)) for v in ['main','fast']},'engine':'/obsolete-deployment.engine'}
        bindings={}
        for version in ['main','fast']:
            runtime=Path(cfg['repositories'][version])/'src/vision/config/yolo26_runtime.yaml'
            runtime.parent.mkdir(parents=True)
            engine=self.root/(version+'.engine');engine.write_bytes(b'fake model; never loaded')
            runtime.write_text(json.dumps({'yolo26_runtime':{'ros__parameters':{'pc_engine_path':str(engine),'jetson_engine_path':str(engine)}}}))
            data=fixture_release(self.root/('release-'+version));data['version']=version
            lib=Path(data['ros'])/'vision/lib/vision'
            for name in ['jandi_runtime.py','jandi_logs.py']:
                shutil.copy2(Path(d.__file__).parent/name,lib/name)
            d.atomic_json(d.release_paths(version)[1],data)
            roots=fixture_roots(Path(cfg['repositories'][version])/'src')
            bindings[version]=(runtime,engine,roots['vision_core']/'config/vision_algorithm.yaml')
            old=d.CONFIG_DIR/version/'vision_algorithm.yaml'
            old.parent.mkdir(parents=True);old.write_text('vision_algorithm: [invalid old home config')
        real_call=d.call;calls=[]
        def fake_backend(command,**kwargs):
            calls.append((command,dict(kwargs.get('env',{}))))
            if '-c' in command or Path(command[0]).name=='config_check':return real_call(command,**kwargs) # real read-only resolver, no ROS init
            self.assertEqual(Path(command[1]).name,'run_jandi')
            return subprocess.CompletedProcess(command,0)
        with patch.object(d,'load_config',return_value=cfg),patch.object(d,'active_plan',return_value=None), \
             patch.object(d,'ros_environment',side_effect=lambda *args:d.clean_environment()),patch.object(d,'call',side_effect=fake_backend):
            for version in ['main','fast']:
                calls.clear();self.assertEqual(d.main(['run',version,'--no-menu']),0)
                self.assertEqual(len(calls),3)
                self.assertEqual(calls[-1][1]['YOLO26_RUNTIME_CONFIG'],str(bindings[version][0]))
                self.assertNotIn('YOLO26_ENGINE_PATH',calls[-1][1])
                self.assertEqual(calls[-1][1]['VISION_CORE_ALGORITHM_CONFIG'],str(bindings[version][2]))
            runtime,engine,algorithm=bindings['main']
            new_engine=self.root/'changed.engine';new_engine.write_bytes(b'new fake model')
            runtime.write_text(runtime.read_text().replace(str(engine),str(new_engine)))
            calls.clear();self.assertEqual(d.main(['run','main','--no-menu']),0)
            new_engine.unlink();calls.clear()
            with self.assertRaises(subprocess.CalledProcessError):d.main(['run','main','--no-menu'])
            self.assertEqual(len(calls),2,'missing engine started the backend')
            new_engine.write_bytes(b'restored fake')
            algorithm.write_text('vision_algorithm:\n  test_gain: 9 # next start reads latest source\n')
            calls.clear();self.assertEqual(d.main(['run','main','--no-menu']),0)
            self.assertEqual(calls[0][0][1],algorithm)
            algorithm.write_text('vision_algorithm:\n  test_gain: -1\n')
            calls.clear()
            with self.assertRaisesRegex(RuntimeError,'test_gain'):d.main(['run','main','--no-menu'])
            self.assertEqual(len(calls),1,'invalid algorithm started engine probe or backend')

    def test_pc_and_jetson_use_the_selected_shared_source(self):
        release=fixture_release(self.root/'release');core=Path(release['core'])
        installed=(core/'share/shared_vision_core/config/vision_algorithm.yaml').read_bytes()
        for machine in ('pc','jetson'):
            for version in ('main','fast'):
                repo=self.root/('jetson-'+version)
                roots=d.source_roots({'machine':machine},version,repo)
                path=roots['vision_core']/'config/vision_algorithm.yaml'
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text('vision_algorithm:\n  test_gain: 7 # shared\n')
                previous=path.read_bytes();mtime=path.stat().st_mtime_ns
                old=d.CONFIG_DIR/version/'vision_algorithm.yaml'
                old.parent.mkdir(parents=True,exist_ok=True)
                old.write_text('invalid ignored old config: [')
                self.assertEqual(d.source_algorithm_config(roots,core,{}),path)
                self.assertEqual(path.read_bytes(),previous)
                self.assertEqual(path.stat().st_mtime_ns,mtime)
                self.assertEqual(old.read_text(),'invalid ignored old config: [')
                expected=(self.root/('vision_core' if version=='main' else 'vision_core_fast') if machine=='pc'
                          else repo/'src/vision_core')/'config/vision_algorithm.yaml'
                self.assertEqual(path,expected)
        self.assertEqual((core/'share/shared_vision_core/config/vision_algorithm.yaml').read_bytes(),installed)
        self.assertFalse((d.STATE_DIR/'algorithm-originals').exists())

    def test_missing_or_invalid_source_never_falls_back_or_rewrites_it(self):
        release=fixture_release(self.root/'release');roots=fixture_roots(self.root/'sources')
        core=Path(release['core']);path=roots['vision_core']/'config/vision_algorithm.yaml'
        path.unlink()
        with self.assertRaisesRegex(RuntimeError,'소스 YAML 없음'):d.source_algorithm_config(roots,core,{})
        for bad in ['', 'vision_algorithm: [', 'vision_algorithm: null\n',
                    'vision_algorithm:\n  test_gain: 1\n  test_gain: 2\n',
                    'vision_algorithm:\n  test_gain: -1\n']:
            path.write_text(bad)
            with self.assertRaises(RuntimeError):d.source_algorithm_config(roots,core,{})
            self.assertEqual(path.read_text(),bad)
        self.assertFalse(d.CONFIG_DIR.exists())

    def test_failed_source_validation_preserves_previous_success_and_yaml(self):
        old=fixture_release(self.root/'previous');_,manifest=d.release_paths('main');d.atomic_json(manifest,old)
        before=manifest.read_bytes();roots=fixture_roots(self.root/'sources')
        path=roots['vision_core']/'config/vision_algorithm.yaml'
        path.write_text('vision_algorithm:\n  test_gain: -1 # invalid shared config\n')
        tuned=path.read_bytes();real=d.call
        def fake_build(command,**kwargs):
            if Path(command[0]).name=='config_check':return real(command,**kwargs)
            if command[0]=='colcon':fixture_release(Path(command[command.index('--install-base')+1]).parent)
            return subprocess.CompletedProcess(command,0)
        with patch.object(d,'git',return_value='b'*40),patch.object(d,'ros_environment',return_value={}), \
             patch.object(d,'call',side_effect=fake_build):
            with self.assertRaisesRegex(RuntimeError,'test_gain'):
                d.build_release({'machine':'pc'},'main',self.root,roots)
        self.assertEqual(manifest.read_bytes(),before);self.assertEqual(path.read_bytes(),tuned)
        self.assertTrue(d.valid_release(old));self.assertFalse(d.CONFIG_DIR.exists())

    def test_source_tuning_is_revalidated_without_modifying_the_install(self):
        roots=fixture_roots(self.root/'sources')
        release=fixture_release(self.root/'release');core=Path(release['core'])
        installed=core/'share/shared_vision_core/config/vision_algorithm.yaml'
        release['artifacts']={str(installed):hashlib.sha256(installed.read_bytes()).hexdigest()}
        path=d.source_algorithm_config(roots,core,{})
        path.write_text('vision_algorithm:\n  test_gain: 9\n')
        self.assertEqual(d.source_algorithm_config(roots,core,{}),path)
        self.assertTrue(d.valid_release(release))
        self.assertIn('test_gain: 9',path.read_text())

    def test_generated_install_link_does_not_block_update_but_source_edits_do(self):
        repo=self.root/'repo';repo.mkdir()
        subprocess.run(['git','init','-q',str(repo)],check=True)
        subprocess.run(['git','-C',str(repo),'remote','add','origin',d.ORIGIN],check=True)
        subprocess.run(['git','-C',str(repo),'checkout','--orphan','main'],check=True,capture_output=True)
        subprocess.run(['git','-C',str(repo),'-c','user.name=Test','-c','user.email=test@example.invalid',
                        'commit','-q','--allow-empty','-m','fixture'],check=True)
        source=repo/'src/vision_core/config/vision_algorithm.yaml'
        source.parent.mkdir(parents=True);source.write_text('shared fixture')
        subprocess.run(['git','-C',str(repo),'add','src'],check=True)
        subprocess.run(['git','-C',str(repo),'-c','user.name=Test','-c','user.email=test@example.invalid',
                        'commit','-q','-m','source fixture'],check=True)
        target=repo/'releases/release/ros';target.mkdir(parents=True)
        (repo/'install').symlink_to(target,target_is_directory=True)
        cfg={'repositories':{'main':str(repo)}}
        self.assertEqual(d.ensure_repo(cfg,'main'),repo)
        self.assertEqual(d.git(repo,'status','--porcelain'),'')
        source.write_text('local tuning must not be discarded')
        with self.assertRaisesRegex(RuntimeError,'미커밋'):d.ensure_repo(cfg,'main')
        self.assertEqual(source.read_text(),'local tuning must not be discarded')

    def test_backup_branch_is_not_an_accepted_version(self):
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            d.main(['select','backup/pre-final-main-20261108'])

    def test_nested_operations_lock_refused(self):
        with d.operation_lock():
            with self.assertRaisesRegex(RuntimeError,'진행 중'):
                with d.operation_lock():pass

    def test_compat_wrapper_accepts_version_after_machine_option(self):
        with patch.object(d,'load_config',return_value={'machine':'pc','selected':'main'}), \
             patch.object(d,'active_plan',return_value=None):
            self.assertEqual(d.main(['select','--machine','pc','fast']),0)
        self.assertEqual(json.loads((self.root/'config/deployment.json').read_text())['selected'],'fast')

    def test_build_and_install_artifacts_are_not_canonical_sources(self):
        (self.root/'source.cpp').write_text('source')
        for directory in ('build_final','install_final','install','log_final'):
            path=self.root/directory;path.mkdir();(path/'artifact').write_bytes(b'compiled')
        self.assertEqual(d.files(self.root),{'source.cpp':b'source'})

    def test_algorithm_edit_invalidates_source_fingerprint(self):
        roots=fixture_roots(self.root)
        a=d.fingerprint('a',{},roots);(roots['vision_core']/'algorithm.yaml').write_text('weight: 1')
        self.assertNotEqual(a,d.fingerprint('a',{},roots))

if __name__=='__main__':unittest.main()
