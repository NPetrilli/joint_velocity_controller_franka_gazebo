import rosbag

# Apri il file rosbag
bag = rosbag.Bag('/home/vandalsnike/Results/tf_2024-09-08-23-40-56.bag')

# Leggi i messaggi TF
for topic, msg, t in bag.read_messages(topics=['/tf']):
    for transform in msg.transforms:
        if transform.header.frame_id == 'panda_link0' and transform.child_frame_id == 'panda_EE':
            # Estrai la trasformazione
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            print("Posizione: ", translation)
            print("Orientamento (quaternion): ", rotation)

bag.close()
