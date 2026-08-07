ros2_pi_project/
├── canbat_node.py        # Your first python script
├── gpio_node.py          # Your second python script
├── start.sh              # Bash script to launch both nodes
├── Dockerfile            # Container build instructions
└── docker-compose.yml    # Container runtime configuration



After all files in place run

'chmod +x start.sh'

'docker compose up -d --build'

This will take a moment.

check using:
'docker logs -f ros2_hardware_nodes'

