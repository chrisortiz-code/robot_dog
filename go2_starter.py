"""
Unitree Go2 Starter Script
---------------------------
Connect to the Go2 via ethernet and control it with simple commands.

Usage:
    # Default (auto-detect network interface):
    python go2_starter.py

    # Specify network interface explicitly:
    python go2_starter.py eth0

Setup:
    1. Connect ethernet cable from your PC to the Go2's ethernet port
    2. Set your PC's ethernet IP to 192.168.123.x (e.g. 192.168.123.99)
       with subnet mask 255.255.255.0
    3. The Go2 is at 192.168.123.161 by default
    4. Make sure the dog is powered on and standing in a clear area
"""

import sys
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


# ---------------------------------------------------------------------------
# Available commands - type these in the interactive prompt
# ---------------------------------------------------------------------------
COMMANDS = {
    "stand_up":   "Stand up from sitting position",
    "stand_down": "Sit down",
    "forward":    "Walk forward (0.3 m/s for 2 sec)",
    "backward":   "Walk backward",
    "left":       "Strafe left",
    "right":      "Strafe right",
    "turn_left":  "Rotate left in place",
    "turn_right": "Rotate right in place",
    "stop":       "Stop all movement",
    "recover":    "Recovery stand (use if fallen over)",
    "balance":    "Balanced stand mode",
    "damp":       "Emergency damp - kills motors (robot will collapse!)",
    "help":       "Show this command list",
    "quit":       "Exit the program",
}


def show_help():
    print("\n--- Available Commands ---")
    for cmd, desc in COMMANDS.items():
        print(f"  {cmd:14s} {desc}")
    print()


def run_command(client: SportClient, cmd: str):
    cmd = cmd.strip().lower()

    if cmd == "stand_up":
        client.StandUp()
    elif cmd == "stand_down":
        client.StandDown()
    elif cmd == "forward":
        client.Move(0.3, 0, 0)
        time.sleep(2)
        client.StopMove()
    elif cmd == "backward":
        client.Move(-0.3, 0, 0)
        time.sleep(2)
        client.StopMove()
    elif cmd == "left":
        client.Move(0, 0.3, 0)
        time.sleep(2)
        client.StopMove()
    elif cmd == "right":
        client.Move(0, -0.3, 0)
        time.sleep(2)
        client.StopMove()
    elif cmd == "turn_left":
        client.Move(0, 0, 0.5)
        time.sleep(2)
        client.StopMove()
    elif cmd == "turn_right":
        client.Move(0, 0, -0.5)
        time.sleep(2)
        client.StopMove()
    elif cmd == "stop":
        client.StopMove()
    elif cmd == "recover":
        client.RecoveryStand()
    elif cmd == "balance":
        client.BalanceStand()
    elif cmd == "damp":
        confirm = input("  WARNING: This kills motors and the robot will collapse. Type 'yes' to confirm: ")
        if confirm.strip().lower() == "yes":
            client.Damp()
        else:
            print("  Cancelled.")
    elif cmd == "help":
        show_help()
    elif cmd == "quit":
        print("Stopping movement and exiting...")
        client.StopMove()
        sys.exit(0)
    else:
        print(f"  Unknown command: '{cmd}'. Type 'help' for available commands.")


def main():
    print("=" * 50)
    print("  Unitree Go2 Starter Controller")
    print("=" * 50)
    print()
    print("WARNING: Make sure the area around the robot is clear!")
    input("Press Enter to connect...")

    # Initialize DDS communication
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # Create and initialize sport client
    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()

    print("Connected! Type 'help' to see commands.\n", flush=True)
    show_help()

    while True:
        try:
            sys.stdout.flush()
            cmd = input("go2> ")
            if not cmd.strip():
                continue
            run_command(client, cmd)
            time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nStopping movement and exiting...")
            client.StopMove()
            break


if __name__ == "__main__":
    main()
