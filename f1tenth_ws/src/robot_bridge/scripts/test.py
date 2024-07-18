import subprocess

def run_ros2_node(package, executable):
    try:
        # Construct the command
        command = ['ros2', 'run', package, executable]
        
        # Execute the command
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        
        # Print the output
        print("Output:", result.stdout)
        print("Error:", result.stderr)
        
    except subprocess.CalledProcessError as e:
        print(f"Command '{e.cmd}' returned non-zero exit status {e.returncode}.")
        print(f"Error output: {e.stderr}")

# Example usage
run_ros2_node('demo_nodes_cpp', 'talker')
