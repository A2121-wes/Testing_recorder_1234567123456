sed -i "s/if 'GOOD' in d\['status'\] or 'COMPLETE' in d\['status'\]:/if d['lat'] != 0.0:/" /home/ncalm/novatel_recorder.py



python3 /home/ncalm/novatel_recorder.py --receiver_id 0 --port /dev/ttyUSB1 --baud 115200 --output /home/ncalm/flight_data --no_camera