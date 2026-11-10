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
    return data


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
        before=manifest.read_bytes();roots={p:self.root/p for p in d.PACKAGES}
        for p in roots.values():p.mkdir()
        with patch.object(d,'git',return_value='a'*40),patch.object(d,'ros_environment',return_value={}), \
             patch.object(d,'call',side_effect=subprocess.CalledProcessError(1,['colcon'])):
            with self.assertRaises(subprocess.CalledProcessError):
                d.build_release({'machine':'pc'},'main',self.root,roots,True)
        self.assertEqual(manifest.read_bytes(),before);self.assertTrue(d.valid_release(old))

    def test_new_build_only_publishes_after_all_steps(self):
        _,manifest=d.release_paths('main');roots={p:self.root/p for p in d.PACKAGES}
        for p in roots.values():p.mkdir()
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
        self.assertEqual(len(commands),4);self.assertEqual(json.loads(manifest.read_text()),installed)
        self.assertTrue(d.valid_release(installed))
        with patch.object(d,'git',return_value='b'*40),patch.object(d,'call') as call:
            self.assertEqual(d.build_release({'machine':'pc'},'main',self.root,roots),installed)
            call.assert_not_called()

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
        d.atomic_json(d.CONFIG_DIR/'deployment.json',{'machine':'pc','selected':'main','camera_config':str(old),'camera_launch':str(launch),'engine':'/obsolete.engine'})
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
            bindings[version]=(runtime,engine)
        real_call=d.call;calls=[]
        def fake_backend(command,**kwargs):
            calls.append((command,dict(kwargs.get('env',{}))))
            if '-c' in command:return real_call(command,**kwargs) # real read-only resolver, no ROS init
            self.assertEqual(Path(command[1]).name,'run_jandi')
            return subprocess.CompletedProcess(command,0)
        with patch.object(d,'load_config',return_value=cfg),patch.object(d,'active_plan',return_value=None), \
             patch.object(d,'ros_environment',side_effect=lambda *args:d.clean_environment()),patch.object(d,'call',side_effect=fake_backend):
            for version in ['main','fast']:
                calls.clear();self.assertEqual(d.main(['run',version,'--no-menu']),0)
                self.assertEqual(len(calls),2)
                self.assertEqual(calls[-1][1]['YOLO26_RUNTIME_CONFIG'],str(bindings[version][0]))
                self.assertNotIn('YOLO26_ENGINE_PATH',calls[-1][1])
            runtime,engine=bindings['main']
            new_engine=self.root/'changed.engine';new_engine.write_bytes(b'new fake model')
            runtime.write_text(runtime.read_text().replace(str(engine),str(new_engine)))
            calls.clear();self.assertEqual(d.main(['run','main','--no-menu']),0)
            new_engine.unlink();calls.clear()
            with self.assertRaises(subprocess.CalledProcessError):d.main(['run','main','--no-menu'])
            self.assertEqual(len(calls),1,'missing engine started the backend')

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
        roots={p:self.root/p for p in d.PACKAGES}
        for p in roots.values():p.mkdir()
        a=d.fingerprint('a',{},roots);(roots['vision_core']/'algorithm.yaml').write_text('weight: 1')
        self.assertNotEqual(a,d.fingerprint('a',{},roots))

if __name__=='__main__':unittest.main()
