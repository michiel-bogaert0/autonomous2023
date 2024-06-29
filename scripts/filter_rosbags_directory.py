import os
import subprocess


def filter_rosbags(directory):
    for filename in os.listdir(directory):
        if filename.endswith(".bag"):
            print(f"Processing {filename}")

            input_bag = os.path.join(directory, filename)
            output_bag = os.path.join(
                directory, filename.replace(".bag", "_filtered.bag")
            )

            command = f"rosbag filter {input_bag} {output_bag} \"topic not in ['/ugr/car/sensors/cam0/image']\""

            try:
                subprocess.run(command, shell=True, check=True)
                print(f"Filtered {input_bag} and saved as {output_bag}")
            except subprocess.CalledProcessError as e:
                print(f"Failed to filter {input_bag}: {e}")


directory = os.getcwd()
filter_rosbags(directory)
