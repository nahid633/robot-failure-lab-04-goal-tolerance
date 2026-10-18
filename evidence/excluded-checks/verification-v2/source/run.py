#!/usr/bin/env python3
"""Fresh processes, frozen evidence, two excluded checks and six main trials."""
import argparse,hashlib,json,os,shutil,signal,subprocess,sys,time
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from core import CONDITIONS,TOLERANCES,PARAMETER,POSITION_LIMIT,START,GOAL,configuration,differences
from validate import validate_trial


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def stop(process):
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=12)
    except subprocess.TimeoutExpired:pass
    try:os.killpg(process.pid,signal.SIGKILL)
    except ProcessLookupError:pass
    process.wait(timeout=5)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('output',type=Path)
    parser.add_argument('--verification',action='store_true')
    parser.add_argument('--verification-root',type=Path)
    parser.add_argument('--resume-infrastructure',action='store_true')
    parser.add_argument('--domain',type=int,default=193)
    args=parser.parse_args();assert 1<=args.domain<=220
    root=args.output.resolve();parent=Path(__file__).resolve().parent;snapshot=root/'source'
    if args.resume_infrastructure:
        protocol=json.loads((root/'PROTOCOL.json').read_text())
        assert protocol['verification']==args.verification
        for name,digest in protocol['source_sha256'].items():assert sha(snapshot/name)==digest
        for name,digest in protocol['config_sha256'].items():assert sha(root/name)==digest
        for name,digest in protocol['source_sha256'].items():
            if (parent/name).exists():assert sha(parent/name)==digest
        records=json.loads((root/'results.json').read_text())
        (root/f'continuation-{time.time_ns()}.json').write_text(json.dumps(dict(command=sys.argv,
            domain=args.domain,preserved_trials=[r['trial'] for r in records],
            reason='Continue before-goal infrastructure failures only; preserve every completed outcome.'),indent=2))
    else:
        root.mkdir(parents=True,exist_ok=False);snapshot.mkdir()
        for path in [*parent.glob('*.py'),parent/'README.md']:shutil.copy2(path,snapshot/path.name)
        bringup=Path(get_package_share_directory('nav2_bringup'))
        simulation=Path(get_package_share_directory('nav2_minimal_tb3_sim'))
        upstream=bringup/'launch/tb3_simulation_launch.py';launch=upstream.read_text()
        needle="cmd=['gz', 'sim', '-r', '-s', world_sdf]";assert launch.count(needle)==1
        shutil.copy2(upstream,snapshot/'upstream_tb3_simulation_launch.txt')
        (snapshot/'seeded_simulation_launch.py').write_text(launch.replace(needle,
            "cmd=['gz', 'sim', '--seed', os.environ['RFL_GZ_SEED'], '-r', '-s', world_sdf]"))
        robot=(simulation/'urdf/gz_waffle.sdf.xacro').read_text();assert robot.count('</model>')==1
        (snapshot/'upstream_robot.sdf.xacro').write_text(robot)
        plugin='''<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
          <publish_link_pose>false</publish_link_pose><publish_visual_pose>false</publish_visual_pose>
          <publish_collision_pose>false</publish_collision_pose><publish_sensor_pose>false</publish_sensor_pose>
          <publish_model_pose>true</publish_model_pose><publish_nested_model_pose>false</publish_nested_model_pose>
          <use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>30</update_frequency>
        </plugin>'''
        (snapshot/'measured_robot.sdf.xacro').write_text(robot.replace('</model>',plugin+'\n</model>'))
        shutil.copy2(simulation/'worlds/tb3_sandbox.sdf.xacro',snapshot/'world.sdf.xacro')
        base=yaml.safe_load((bringup/'params/nav2_params.yaml').read_text())
        assert base['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']==.25
        (root/'upstream-params.yaml').write_text(yaml.safe_dump(base,sort_keys=False))
        configs={c:configuration(base,c) for c in CONDITIONS}
        delta=differences(configs['baseline'],configs['relaxed'])
        assert delta==[('controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance',.25,.75)],delta
        for condition,config in configs.items():(root/f'{condition}-params.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
        (root/'CONFIGURATION-DIFF.json').write_text(json.dumps(delta,indent=2))
        for name in ('tb3_sandbox.pgm','tb3_sandbox.yaml'):shutil.copy2(bringup/'maps'/name,root/name)
        versions=subprocess.run(['dpkg-query','-W','ros-jazzy-nav2-*','ros-jazzy-ros-gz-*','ros-jazzy-gz-*','gz-*'],capture_output=True,text=True)
        (root/'packages.txt').write_text(versions.stdout+versions.stderr)
        if not args.verification:
            if args.verification_root is None:raise RuntimeError('Main trials require excluded verification checks')
            vr=args.verification_root.resolve();vp=json.loads((vr/'PROTOCOL.json').read_text())
            checks=json.loads((vr/'results.json').read_text())
            assert len(checks)==2 and {r['condition'] for r in checks}==set(CONDITIONS)
            for record in checks:validate_trial(vr,record)
            for path in snapshot.iterdir():assert sha(path)==vp['source_sha256'][path.name],path.name
            for condition in CONDITIONS:assert sha(root/f'{condition}-params.yaml')==sha(vr/f'{condition}-params.yaml')
            assert sha(root/'packages.txt')==sha(vr/'packages.txt')
        schedule=[dict(block=0,condition=c,gazebo_seed=100) for c in CONDITIONS] if args.verification else [
            dict(block=b,condition=c,gazebo_seed=100+b) for b,order in [(1,CONDITIONS),(2,CONDITIONS[::-1]),(3,CONDITIONS)] for c in order]
        protocol=dict(experiment='Robot Failure Lab 04',verification=args.verification,
            verification_root=str(args.verification_root) if args.verification_root else None,schedule=schedule,
            hypothesis='Relaxed XY tolerance may shorten reported completion while worsening independently measured arrival position. No outcome guaranteed.',
            change={PARAMETER:TOLERANCES},start=START,goal=GOAL,
            primary_requirement=f'Nav2 succeeded AND every world-pose sample 1.0..1.5 sim seconds after result is within {POSITION_LIMIT}m of target. This illustrative XY requirement is fixed across both configs; not docking certification.',
            primary_metrics=['original_goal_outcome','settled_world_error_max_m','settled_position_pass'],
            secondary_metrics=['elapsed_sim_s','terminal_world_error_m','terminal_localized_error_m','world_path_length_m','recoveries','terminal_world_yaw_error_rad'],
            task_timeout_sim_s=90,task_timeout_wall_s=270,startup_timeout_wall_s=75,
            observation_tail_sim_s=1.8,equal_pre_goal_wait_sim_s=1.5,
            controls='Same installed upstream configuration except one declared leaf. No sensor injection. Collision monitor enabled. Common observer-only Gazebo PosePublisher and bridge; world pose is never published on /tf.',
            measurement='Model world pose; model/base_footprint/base_link have equal XY origins in inspected SDF. Initial world XY checked within 0.05m of spawn; map/world coordinate correspondence uses supplied sandbox, no fitted alignment.',
            interpretation='Report every outcome, within-seed time differences and all endpoint errors. Three pairs on one route are exploratory, not a general speedup/reliability claim. Do not change threshold/sample after results.',
            randomness='Gazebo seeds matched per pair; AMCL, MPPI, host scheduling uncontrolled; paths not deterministic.',
            scope='Standalone simulation. No replacement goal, physical robot, Vektor integration or public posting.',
            command=sys.argv,platform=os.uname().release,python=sys.version,
            source_sha256={p.name:sha(p) for p in snapshot.iterdir()},
            config_sha256={p.name:sha(p) for p in root.glob('*.yaml')},packages_sha256=sha(root/'packages.txt'))
        (root/'PROTOCOL.json').write_text(json.dumps(protocol,indent=2)+'\n')
        records=[];(root/'results.json').write_text('[]\n')
    for entry in protocol['schedule']:
        block,condition,seed=entry['block'],entry['condition'],entry['gazebo_seed']
        prior=[r for r in records if r['block']==block and r['condition']==condition]
        if prior:
            assert len(prior)==1 and prior[0]['evidence_validated'];validate_trial(root,prior[0]);continue
        base=f'block-{block}-{condition}';name=base;trial=root/name;attempt=1
        while trial.exists():
            if (not args.resume_infrastructure or not (trial/'INFRASTRUCTURE_ERROR.txt').exists()
                    or any((trial/f).exists() for f in ('READY.json','GOAL_SUBMITTED.json','TASK_TERMINAL.json','result.json'))):
                raise RuntimeError(f'Refusing to overwrite/rerun evidence: {trial}')
            attempt+=1
            if attempt>3:raise RuntimeError('Three startup attempts exhausted')
            name=f'{base}-attempt-{attempt}';trial=root/name
        trial.mkdir()
        env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                 GZ_PARTITION=f'rfl04-{os.getpid()}-{name}',LIBGL_ALWAYS_SOFTWARE='1',RFL_GZ_SEED=str(seed))
        env.pop('ROS_LOCALHOST_ONLY',None)
        command=['ros2','launch',str(snapshot/'seeded_simulation_launch.py'),'headless:=True','use_rviz:=False',
            'use_composition:=False',f'params_file:={root / (condition+"-params.yaml")}',
            f'map:={root / "tb3_sandbox.yaml"}',f'robot_sdf:={snapshot / "measured_robot.sdf.xacro"}',f'world:={snapshot / "world.sdf.xacro"}']
        bridge_command=['ros2','run','ros_gz_bridge','parameter_bridge',
            '/model/turtlebot3_waffle/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','--ros-args',
            '-r','__node:=failure_lab_04_truth_bridge','-r','/model/turtlebot3_waffle/pose:=/rfl04/truth']
        (trial/'launch-command.json').write_text(json.dumps(dict(command=command,bridge=bridge_command,
            gazebo_seed=seed,domain=args.domain,host_load=os.getloadavg(),params_sha256=sha(root/f'{condition}-params.yaml')),indent=2))
        print(f'Starting {name}',flush=True);processes=[]
        try:
            with (trial/'simulation.log').open('w') as sl,(trial/'bridge.log').open('w') as bl,(trial/'trial.log').open('w') as tl:
                processes.append(subprocess.Popen(command,env=env,stdout=sl,stderr=subprocess.STDOUT,start_new_session=True))
                processes.append(subprocess.Popen(bridge_command,env=env,stdout=bl,stderr=subprocess.STDOUT,start_new_session=True))
                time.sleep(8)
                worker=subprocess.Popen([sys.executable,str(snapshot/'trial.py'),'--condition',condition,'--output',str(trial)],
                    env=env,stdout=tl,stderr=subprocess.STDOUT,start_new_session=True);processes.append(worker)
                began=time.monotonic()
                while worker.poll() is None:
                    elapsed=time.monotonic()-began
                    if elapsed>75 and not (trial/'READY.json').exists():raise TimeoutError('Readiness exceeded 75 wall seconds')
                    if elapsed>390:raise TimeoutError('Whole-worker deadline exceeded')
                    time.sleep(.2)
                if worker.returncode or not (trial/'result.json').exists():raise RuntimeError('Worker error; inspect retained logs')
        except BaseException as error:
            (trial/'INFRASTRUCTURE_ERROR.txt').write_text(repr(error)+'\n');raise
        finally:
            for process in reversed(processes):stop(process)
            (trial/'process-exits.json').write_text(json.dumps([dict(pid=p.pid,returncode=p.returncode) for p in processes],indent=2))
        record=json.loads((trial/'result.json').read_text());record.update(trial=name,block=block,gazebo_seed=seed,evidence_validated=False)
        records.append(record);(root/'results.json').write_text(json.dumps(records,indent=2)+'\n')
        try:
            validation=validate_trial(root,record)
            (trial/'VALIDATION.json').write_text(json.dumps(validation,indent=2)+'\n')
            record['evidence_validated']=True;(root/'results.json').write_text(json.dumps(records,indent=2)+'\n')
        except BaseException as error:
            (trial/'PROTOCOL_ERROR.txt').write_text(repr(error)+'\n');raise
        print(f'{name}: {record["outcome"]}, {record["elapsed_sim_s"]:.3f}s; settled error {record["metrics"]["settled_world_error_max_m"]:.3f}m; position pass={record["metrics"]["settled_position_pass"]}',flush=True)
    print(f'COMPLETE: {len(records)} validated trials',flush=True)


if __name__=='__main__':main()
