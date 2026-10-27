"""Bounded DDS-only burst regression; no hardware/SDK/network-device use.

Source ROS2, build this directory with CMake, then set ROS_DOMAIN_ID=199 and
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST. Run --camera-depth8 --imu-depth512
--ack-chunk64 (with spaces between option/value) and a new --output directory.
Requires Pillow and rclpy; every synthetic JPEG is decoded fully. --ack-chunk0
reproduces unbounded publication bursts; test outputs are never overwritten.
"""
import argparse,io,json,os,pathlib,subprocess,time
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.event_handler import SubscriptionEventCallbacks
from sensor_msgs.msg import CompressedImage,Imu
from PIL import Image

p=argparse.ArgumentParser()
p.add_argument('--binary',required=True)
p.add_argument('--output',required=True)
p.add_argument('--camera-depth',type=int,required=True)
p.add_argument('--imu-depth',type=int,required=True)
p.add_argument('--jpeg')
p.add_argument('--publisher-profile')
p.add_argument('--ack-chunk',type=int,default=0)
a=p.parse_args()
out=pathlib.Path(a.output);out.mkdir()
assert os.environ['ROS_DOMAIN_ID']=='199'
# Representative full-sized noisy JPEG, deterministic size is recorded.
if a.jpeg:
    (out/'sample.jpg').write_bytes(pathlib.Path(a.jpeg).read_bytes())
else:
    Image.effect_noise((1280,1024),45).convert('RGB').save(out/'sample.jpg',quality=88)
rclpy.init();node=rclpy.create_node('prism_dds_burst_observer')
q=QoSProfile(depth=8192,reliability=ReliabilityPolicy.RELIABLE)
received={str(i):[] for i in range(4)};received['imu']=[]
lost={k:0 for k in received};subscriptions=[]
def stamp(m):return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
def camera(c,m):
    im=Image.open(io.BytesIO(bytes(m.data)));im.load();assert im.size==(1280,1024)
    assert m.header.frame_id==str(c)
    received[str(c)].append((stamp(m)-1000000000)//100000000)
def imu(m):
    n=(stamp(m)-1000000000)//1250000
    assert m.linear_acceleration.x==n
    received['imu'].append(n)
def event(k,e):lost[k]=e.total_count
for i in range(4):
    subscriptions.append(node.create_subscription(CompressedImage,'/dds_probe/camera'+str(i),
      lambda m,i=i:camera(i,m),q,event_callbacks=SubscriptionEventCallbacks(message_lost=lambda e,i=i:event(str(i),e))))
subscriptions.append(node.create_subscription(Imu,'/dds_probe/imu',imu,q,
  event_callbacks=SubscriptionEventCallbacks(message_lost=lambda e:event('imu',e))))
print('IMU subscriber actual QoS:',subscriptions[-1].qos_profile,flush=True)
with (out/'publisher.log').open('x') as log:
    publisher_env=os.environ.copy()
    if a.publisher_profile:publisher_env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(pathlib.Path(a.publisher_profile).resolve())
    process=subprocess.Popen([a.binary,str(a.camera_depth),str(a.imu_depth),str(out/'sample.jpg'),str(a.ack_chunk)],env=publisher_env,stdout=log,stderr=subprocess.STDOUT)
    start=time.monotonic()
    while process.poll() is None and time.monotonic()-start<50:
        rclpy.spin_once(node,timeout_sec=.01)
    assert process.poll() is not None,'publisher exceeded bounded deadline'
    deadline=time.monotonic()+1
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
report=dict(camera_depth=a.camera_depth,imu_depth=a.imu_depth,rc=process.returncode,
            seconds=time.monotonic()-start,jpeg_bytes=(out/'sample.jpg').stat().st_size,lost_events=lost,streams={})
for k,v in received.items():
    expected=5760 if k=='imu' else 72
    report['streams'][k]=dict(count=len(v),missing=sorted(set(range(expected))-set(v)),
                             duplicate=len(v)-len(set(v)),ordered=all(b>a for a,b in zip(v,v[1:])))
(out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if process.returncode==0 and all(not v['missing'] and not v['duplicate'] and v['ordered'] for v in report['streams'].values()) else 1)
